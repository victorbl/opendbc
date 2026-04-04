import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.rivian.riviancan import create_lka_steering, create_longitudinal, create_wheel_touch, create_adas_status
from opendbc.car.rivian.values import CarControllerParams


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.apply_torque_last = 0
    self.packer = CANPacker(dbc_names[Bus.pt])

    self.cancel_frames = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []

    apply_torque = 0
    steer_max = round(float(np.interp(CS.out.vEgoRaw, CarControllerParams.STEER_MAX_LOOKUP[0],
                                      CarControllerParams.STEER_MAX_LOOKUP[1])))
    if CC.latActive:
      new_torque = int(round(CC.actuators.torque * steer_max))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, CarControllerParams, steer_max)

    # send steering command
    self.apply_torque_last = apply_torque
    can_sends.append(create_lka_steering(self.packer, self.frame, CS.acm_lka_hba_cmd, apply_torque, CC.enabled, CC.latActive))

    if self.frame % 5 == 0:
      can_sends.append(create_wheel_touch(self.packer, CS.sccm_wheel_touch, CC.enabled))

    # Longitudinal control
    if self.CP.openpilotLongitudinalControl:
      # Only send 0x160 when engaged — ACM is sole source otherwise (dynamic relay)
      if CC.enabled:
        accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        can_sends.append(create_longitudinal(self.packer, self.frame, accel, True))

      # Stalk management via 0x162 (always sent — relay blocks stock 0x162)
      interface_status = None
      stalk_override = None

      if CC.enabled:
        # ENGAGED: suppress stalk so ACM never sees pulls
        stalk_override = 0  # IDLE

        # Transition cancel: if ACM was active when we engaged
        if CS.acm_feature_status >= 1:
          interface_status = 1 if self.cancel_frames < 5 else 0
          self.cancel_frames += 1
        else:
          self.cancel_frames = 0
      else:
        # DISENGAGED: forward real stalk values — stock ACC works
        self.cancel_frames = 0

      can_sends.append(create_adas_status(self.packer, CS.vdm_adas_status, interface_status, stalk_override))
    else:
      interface_status = None
      if CC.cruiseControl.cancel:
        # if there is a noEntry, we need to send a status of "available" before the ACM will accept "unavailable"
        # send "available" right away as the VDM itself takes a few frames to acknowledge
        interface_status = 1 if self.cancel_frames < 5 else 0
        self.cancel_frames += 1
      else:
        self.cancel_frames = 0

      can_sends.append(create_adas_status(self.packer, CS.vdm_adas_status, interface_status))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / steer_max
    new_actuators.torqueOutputCan = apply_torque

    self.frame += 1
    return new_actuators, can_sends
