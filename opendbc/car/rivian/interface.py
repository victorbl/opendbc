from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.rivian.carcontroller import CarController
from opendbc.car.rivian.carstate import CarState
from opendbc.car.rivian.radar_interface import RadarInterface
from opendbc.car.rivian.values import RivianFlags, RivianSafetyFlags


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "rivian"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.rivian)]

    # GEN2 (2025+) doesn't have SCCM_WheelTouch on the bus
    if 0x321 not in fingerprint[0]:
      ret.flags |= RivianFlags.GEN2.value

    ret.steerActuatorDelay = 0.15
    ret.steerLimitTimer = 0.4
    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)

    ret.steerControlType = structs.CarParams.SteerControlType.torque
    ret.radarUnavailable = True

    # Detect longitudinal harness upgrade via forwarded park assist messages on bus 1
    ret.alphaLongitudinalAvailable = False
    if 0x131a in fingerprint[1]:
      ret.flags |= RivianFlags.HARNESS_UPGRADE.value
      ret.alphaLongitudinalAvailable = True
      ret.radarUnavailable = False
      ret.enableBsm = True

    if alpha_long and ret.alphaLongitudinalAvailable:
      ret.openpilotLongitudinalControl = True
      ret.pcmCruise = False
      ret.safetyConfigs[0].safetyParam |= RivianSafetyFlags.LONG_CONTROL.value

    ret.longitudinalActuatorDelay = 0.35
    ret.vEgoStopping = 0.25
    ret.stopAccel = 0

    return ret
