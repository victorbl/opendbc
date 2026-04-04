import copy
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import CarStateBase
from opendbc.car.rivian.values import DBC, GEAR_MAP, RivianFlags
from opendbc.car.common.conversions import Conversions as CV

ButtonType = structs.CarState.ButtonEvent.Type

GearShifter = structs.CarState.GearShifter


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.last_speed = 30

    self.acm_lka_hba_cmd: dict | None = None
    self.sccm_wheel_touch: dict | None = None
    self.vdm_adas_status: dict | None = None

    # Button engagement state (left click toggle)
    self._button_engaged = False
    self._left_click_prev = 0
    self._left_click_debounce = 0  # mirror panda's 6-frame debounce
    self._prev_scroll_position = 255  # for scroll direction detection
    self.right_button_right_click = 0

    # ACM feature status for engagement guard + carcontroller transition cancel
    self.acm_feature_status = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_adas = can_parsers[Bus.adas]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp.vl["ESP_Status"]["ESP_Vehicle_Speed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.standstill = abs(ret.vEgoRaw) < 0.01
    conversion = CV.KPH_TO_MS if cp_adas.vl["Cluster"]["Cluster_Unit"] == 0 else CV.MPH_TO_MS
    ret.vEgoCluster = cp_adas.vl["Cluster"]["Cluster_VehicleSpeed"] * conversion

    # Gas pedal
    ret.gasPressed = cp.vl["VDM_PropStatus"]["VDM_AcceleratorPedalPosition"] > 0

    # Brake pedal
    ret.brake = cp.vl["ESPiB3"]["ESPiB3_pMC1"] / 250.0  # pressure in Bar
    ret.brakePressed = cp.vl["iBESP2"]["iBESP2_BrakePedalApplied"] == 1

    # Steering wheel
    ret.steeringAngleDeg = cp.vl["EPAS_AdasStatus"]["EPAS_InternalSas"]
    ret.steeringRateDeg = cp.vl["EPAS_AdasStatus"]["EPAS_SteeringAngleSpeed"]
    ret.steeringTorque = cp.vl["EPAS_SystemStatus"]["EPAS_TorsionBarTorque"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > 1.0, 5)

    ret.steerFaultTemporary = cp.vl["EPAS_AdasStatus"]["EPAS_EacErrorCode"] != 0

    # Cruise state
    speed = min(int(cp_adas.vl["ACM_tsrCmd"]["ACM_tsrSpdDisClsMain"]), 85)
    self.last_speed = speed if speed != 0 else self.last_speed
    self.acm_feature_status = cp_cam.vl["ACM_Status"]["ACM_FeatureStatus"]
    # cruiseState.enabled is set after button logic below (when longitudinal)
    # For non-longitudinal mode, use ACM status directly
    if not self.CP.openpilotLongitudinalControl:
      ret.cruiseState.enabled = self.acm_feature_status == 1
    # TODO: find cruise set speed on CAN
    ret.cruiseState.speed = self.last_speed * CV.MPH_TO_MS  # detected speed limit
    ret.cruiseState.available = True  # cp.vl["VDM_AdasSts"]["VDM_AdasInterfaceStatus"] == 1
    ret.cruiseState.standstill = cp.vl["VDM_AdasSts"]["VDM_AdasVehicleHoldStatus"] == 1

    # ACM_Status->ACM_FaultSupervisorState normally 1, appears to go to 3 when either:
    # 1. car in park/not in drive (normal)
    # 2. something (message from another ECU) ACM relies on is faulty
    #  * ACM_FaultStatus will stay 0 since ACM itself isn't faulted
    # TODO: ACM_FaultStatus hasn't been seen high yet, but log anyway
    ret.accFaulted = (cp_cam.vl["ACM_Status"]["ACM_FaultStatus"] == 1 or
                      # VDM_AdasFaultStatus=Brk_Intv is the default for some reason
                      # VDM_AdasFaultStatus=Cntr_Fault isn't fully understood, but we've seen it in the wild
                      # VDM_AdasFaultStatus=Imps_Cmd was seen when sending it rapidly changing ACC enable commands, or when ACC command drops out
                      cp.vl["VDM_AdasSts"]["VDM_AdasFaultStatus"] in (2, 3))  # 2=Cntr_Fault, 3=Imps_Cmd

    # Gear
    ret.gearShifter = GEAR_MAP.get(int(cp.vl["VDM_PropStatus"]["VDM_Prndl_Status"]), GearShifter.unknown)

    # Doors
    ret.doorOpen = any(cp_adas.vl["IndicatorLights"][door] != 2 for door in ("RearDriverDoor", "FrontPassengerDoor", "DriverDoor", "RearPassengerDoor"))

    # Blinkers
    ret.leftBlinker = cp_adas.vl["IndicatorLights"]["TurnLightLeft"] in (1, 2)
    ret.rightBlinker = cp_adas.vl["IndicatorLights"]["TurnLightRight"] in (1, 2)

    # Seatbelt
    ret.seatbeltUnlatched = cp.vl["RCM_Status"]["RCM_Status_IND_WARN_BELT_DRIVER"] != 0

    # AEB
    ret.stockAeb = cp_cam.vl["ACM_AebRequest"]["ACM_EnableRequest"] != 0

    # Messages needed by carcontroller
    self.acm_lka_hba_cmd = copy.copy(cp_cam.vl["ACM_lkaHbaCmd"])
    self.sccm_wheel_touch = copy.copy(cp.vl["SCCM_WheelTouch"])

    self.vdm_adas_status = copy.copy(cp.vl["VDM_AdasSts"])

    # Button events and BSM (harness upgrade)
    button_events = []
    if self.CP.flags & RivianFlags.HARNESS_UPGRADE:
      cp_park = can_parsers[Bus.alt]

      if self.CP.openpilotLongitudinalControl:
        # --- Left button: engage/disengage toggle ---
        cur_left = cp_park.vl["WheelButtons_Fwd"]["RightButton_LeftClick"]

        # Mirror panda's 6-frame debounce (~300ms at 20Hz)
        if self._left_click_debounce > 0:
          self._left_click_debounce -= 1

        left_rising = (cur_left == 2) and (self._left_click_prev != 2) and (self._left_click_debounce == 0)
        self._left_click_prev = cur_left

        # Reset on brake BEFORE toggle logic
        if ret.brakePressed:
          self._button_engaged = False

        if left_rising:
          if (not self._button_engaged
              and ret.gearShifter == GearShifter.drive
              and self.acm_feature_status == 0):
            # Engage — only when in Drive AND stock ACC not active
            self._button_engaged = True
            button_events.append(structs.CarState.ButtonEvent(pressed=False, type=ButtonType.decelCruise))
            self._left_click_debounce = 6
          elif self._button_engaged:
            # Disengage
            self._button_engaged = False
            button_events.append(structs.CarState.ButtonEvent(pressed=True, type=ButtonType.cancel))
            self._left_click_debounce = 6

        # Set cruiseState.enabled AFTER toggle + brake reset
        ret.cruiseState.enabled = self._button_engaged

        # --- Scroll wheel: speed adjustment (only when OP engaged, not on engagement frame) ---
        cur_scroll = cp_park.vl["WheelButtons_Fwd"]["RightButton_Scroll"]

        if self._button_engaged and not left_rising and cur_scroll != 255 and self._prev_scroll_position != 255:
          delta = cur_scroll - self._prev_scroll_position
          if delta != 0:
            if delta > 127:
              delta -= 255
            elif delta < -127:
              delta += 255
            if delta > 0:
              button_events.append(structs.CarState.ButtonEvent(pressed=False, type=ButtonType.accelCruise))
            elif delta < 0:
              button_events.append(structs.CarState.ButtonEvent(pressed=False, type=ButtonType.decelCruise))

        if cur_scroll != 255:
          self._prev_scroll_position = cur_scroll

        # --- Right button: cycle driving personality ---
        prev_right = self.right_button_right_click
        self.right_button_right_click = cp_park.vl["WheelButtons_Fwd"]["RightButton_RightClick"]
        if prev_right == 2 and self.right_button_right_click != 2:
          button_events.append(structs.CarState.ButtonEvent(pressed=False, type=ButtonType.gapAdjustCruise))

      # BSM from park assist bus
      if self.CP.enableBsm:
        ret.leftBlindspot = cp_park.vl["BSM_BlindSpotIndicator_Fwd"]["BSM_BlindSpotIndicator_Left"] != 0
        ret.rightBlindspot = cp_park.vl["BSM_BlindSpotIndicator_Fwd"]["BSM_BlindSpotIndicator_Right"] != 0

    ret.buttonEvents = button_events

    return ret

  @staticmethod
  def get_can_parsers(CP):
    parsers = {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
      Bus.adas: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 1),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 2),
    }
    if CP.flags & RivianFlags.HARNESS_UPGRADE:
      parsers[Bus.alt] = CANParser(DBC[CP.carFingerprint][Bus.alt], [], 1)
    return parsers
