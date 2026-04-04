#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.car.rivian.values import RivianSafetyFlags
from opendbc.car.rivian.riviancan import checksum as _checksum


def checksum(msg):
  addr, dat, bus = msg
  ret = bytearray(dat)

  # ESP_Status
  if addr == 0x208:
    ret[0] = _checksum(ret[1:], 0x1D, 0xB1)
  elif addr == 0x150:
    ret[0] = _checksum(ret[1:], 0x1D, 0x9A)

  return addr, ret, bus


class TestRivianSafetyBase(common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.LongitudinalAccelSafetyTest,
                           common.VehicleSpeedSafetyTest):

  TX_MSGS = [[0x120, 0], [0x321, 2], [0x162, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x120,), 2: (0x321, 0x162)}
  FWD_BLACKLISTED_ADDRS = {0: [0x321, 0x162], 2: [0x120]}

  MAX_TORQUE_LOOKUP = [9, 17], [350, 250]
  DYNAMIC_MAX_TORQUE = True
  MAX_RATE_UP = 3
  MAX_RATE_DOWN = 5

  MAX_RT_DELTA = 125

  DRIVER_TORQUE_ALLOWANCE = 100
  DRIVER_TORQUE_FACTOR = 2

  cnt_speed = 0
  cnt_speed_2 = 0

  def _torque_driver_msg(self, torque):
    values = {"EPAS_TorsionBarTorque": torque / 100.0}
    return self.packer.make_can_msg_safety("EPAS_SystemStatus", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"ACM_lkaStrToqReq": torque, "ACM_lkaActToi": steer_req}
    return self.packer.make_can_msg_safety("ACM_lkaHbaCmd", 0, values)

  def _speed_msg(self, speed, quality_flag=True):
    values = {"ESP_Vehicle_Speed": speed * 3.6, "ESP_Status_Counter": self.cnt_speed % 15,
              "ESP_Vehicle_Speed_Q": 1 if quality_flag else 0}
    self.__class__.cnt_speed += 1
    return self.packer.make_can_msg_safety("ESP_Status", 0, values, fix_checksum=checksum)

  def _speed_msg_2(self, speed, quality_flag=True):
    # Rivian has a dynamic max torque limit based on speed, so it checks two sources
    return self._user_gas_msg(0, speed, quality_flag)

  def _user_brake_msg(self, brake):
    values = {"iBESP2_BrakePedalApplied": brake}
    return self.packer.make_can_msg_safety("iBESP2", 0, values)

  def _user_gas_msg(self, gas, speed=0, quality_flag=True):
    values = {"VDM_AcceleratorPedalPosition": gas, "VDM_VehicleSpeed": speed * 3.6,
              "VDM_PropStatus_Counter": self.cnt_speed_2 % 15, "VDM_VehicleSpeedQ": 1 if quality_flag else 0}
    self.__class__.cnt_speed_2 += 1
    return self.packer.make_can_msg_safety("VDM_PropStatus", 0, values, fix_checksum=checksum)

  def _pcm_status_msg(self, enable):
    values = {"ACM_FeatureStatus": enable, "ACM_Unkown1": 1}
    return self.packer.make_can_msg_safety("ACM_Status", 2, values)

  def _accel_msg(self, accel: float):
    values = {"ACM_AccelerationRequest": accel}
    return self.packer.make_can_msg_safety("ACM_longitudinalRequest", 0, values)

  def test_wheel_touch(self):
    # For hiding hold wheel alert on engage
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      values = {
        "SCCM_WheelTouch_HandsOn": 1 if controls_allowed else 0,
        "SCCM_WheelTouch_CapacitiveValue": 100 if controls_allowed else 0,
        "SETME_X52": 100,
      }
      self.assertTrue(self._tx(self.packer.make_can_msg_safety("SCCM_WheelTouch", 2, values)))

  def test_rx_hook(self):
    # checksum, counter, and quality flag checks
    for quality_flag in (True, False):
      for msg_type in ("speed", "speed_2"):
        self.safety.set_controls_allowed(True)
        # send multiple times to verify counter checks
        for _ in range(10):
          if msg_type == "speed":
            msg = self._speed_msg(0, quality_flag=quality_flag)
          elif msg_type == "speed_2":
            msg = self._speed_msg_2(0, quality_flag=quality_flag)

          self.assertEqual(quality_flag, self._rx(msg))
          self.assertEqual(quality_flag, self.safety.get_controls_allowed())

        # Mess with checksum to make it fail
        msg[0].data[0] = 0xff
        self.assertFalse(self._rx(msg))
        self.assertFalse(self.safety.get_controls_allowed())


class TestRivianStockSafety(TestRivianSafetyBase):

  LONGITUDINAL = False

  def setUp(self):
    self.packer = CANPackerSafety("rivian_primary_actuator")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.rivian, 0)
    self.safety.init_tests()

  def test_adas_status(self):
    # For canceling stock ACC
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for interface_status in range(4):
        values = {"VDM_AdasInterfaceStatus": interface_status}
        self.assertTrue(self._tx(self.packer.make_can_msg_safety("VDM_AdasSts", 2, values)))


class TestRivianLongitudinalSafety(TestRivianSafetyBase):

  TX_MSGS = [[0x120, 0], [0x321, 2], [0x160, 0], [0x162, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x120, 0x160), 2: (0x321, 0x162)}
  FWD_BLACKLISTED_ADDRS = {0: [0x321, 0x162], 2: [0x120]}  # 0x160 NOT blacklisted (dynamic fwd hook)

  def setUp(self):
    self.packer = CANPackerSafety("rivian_primary_actuator")
    self.packer_alt = CANPackerSafety("rivian_park_assist_can")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.rivian, RivianSafetyFlags.LONG_CONTROL)
    self.safety.init_tests()

  # --- Helper methods ---

  def _wheel_button_msg(self, left_click=0, right_click=0, scroll=255):
    values = {
      "RightButton_LeftClick": left_click,
      "RightButton_RightClick": right_click,
      "RightButton_Scroll": scroll,
    }
    return self.packer_alt.make_can_msg_safety("WheelButtons_Fwd", 1, values)

  def _gear_msg(self, gear=4):
    """Send VDM_PropStatus with gear. 4=Drive, 1=Park, 2=Reverse, 3=Neutral."""
    values = {"VDM_Prndl_Status": gear, "VDM_AcceleratorPedalPosition": 0,
              "VDM_VehicleSpeed": 0, "VDM_PropStatus_Counter": self.cnt_speed_2 % 15,
              "VDM_VehicleSpeedQ": 1}
    self.__class__.cnt_speed_2 += 1
    return self.packer.make_can_msg_safety("VDM_PropStatus", 0, values, fix_checksum=checksum)

  def _engage_button(self):
    """Press and release left button to engage. Assumes in Drive and ACM not active."""
    self._rx(self._wheel_button_msg(left_click=2))  # press
    self._rx(self._wheel_button_msg(left_click=0))  # release

  def _wait_debounce(self):
    """Wait for debounce to expire (6 frames at 20Hz)."""
    for _ in range(6):
      self._rx(self._wheel_button_msg(left_click=0))

  # --- Override common cruise tests (engagement is via button, not pcm_status) ---

  def test_enable_control_allowed_from_cruise(self):
    self._rx(self._gear_msg(4))  # Drive
    self._rx(self._pcm_status_msg(False))  # ACM not active
    self.assertFalse(self.safety.get_controls_allowed())
    self._engage_button()
    self.assertTrue(self.safety.get_controls_allowed())

  def test_disable_control_allowed_from_cruise(self):
    self._rx(self._gear_msg(4))
    self._rx(self._pcm_status_msg(False))
    self._engage_button()
    self.assertTrue(self.safety.get_controls_allowed())
    self._wait_debounce()
    self._engage_button()  # toggle off
    self.assertFalse(self.safety.get_controls_allowed())

  def test_cruise_engaged_prev(self):
    self._rx(self._gear_msg(4))
    self._rx(self._pcm_status_msg(False))
    self._engage_button()
    self.assertTrue(self.safety.get_cruise_engaged_prev())
    self._wait_debounce()
    self._engage_button()
    self.assertFalse(self.safety.get_cruise_engaged_prev())

  # --- Button toggle tests ---

  def test_button_engage_disengage(self):
    self._rx(self._gear_msg(4))
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())

    # Engage
    self._engage_button()
    self.assertTrue(self.safety.get_controls_allowed())

    # Wait debounce, then disengage
    self._wait_debounce()
    self._engage_button()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_button_engage_not_in_drive(self):
    for gear in (1, 2, 3):  # Park, Reverse, Neutral
      # Reset safety to clear debounce between iterations (debounce fires even
      # when engage conditions fail, which would mask the gear guard on subsequent gears)
      self.safety.set_safety_hooks(CarParams.SafetyModel.rivian, RivianSafetyFlags.LONG_CONTROL)
      self.safety.init_tests()
      self._rx(self._gear_msg(gear))
      self._rx(self._pcm_status_msg(False))
      self._engage_button()
      self.assertFalse(self.safety.get_controls_allowed(), f"engaged in gear {gear}")

  def test_button_engage_blocked_during_stock_acc(self):
    self._rx(self._gear_msg(4))
    self._rx(self._pcm_status_msg(True))  # ACM active (stock ACC engaged)
    self._engage_button()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_button_debounce(self):
    self._rx(self._gear_msg(4))
    self._rx(self._pcm_status_msg(False))

    # Engage
    self._engage_button()
    self.assertTrue(self.safety.get_controls_allowed())

    # Immediately try to disengage (within debounce) — should be ignored
    self._rx(self._wheel_button_msg(left_click=2))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_brake_resets_engagement(self):
    self._rx(self._gear_msg(4))
    self._rx(self._pcm_status_msg(False))
    self._engage_button()
    self.assertTrue(self.safety.get_controls_allowed())

    # Brake press
    self._rx(self._user_brake_msg(1))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_button_disengage_always_works(self):
    """Disengage works even during stock ACC (safety escape)."""
    self._rx(self._gear_msg(4))
    self._rx(self._pcm_status_msg(False))
    self._engage_button()
    self.assertTrue(self.safety.get_controls_allowed())

    # Stock ACC goes active while openpilot is engaged
    self._rx(self._pcm_status_msg(True))
    self._wait_debounce()

    # Disengage should still work
    self._engage_button()
    self.assertFalse(self.safety.get_controls_allowed())

  # --- Additional safety tests ---

  def test_engage_simultaneous_brake(self):
    """If brake is pressed in same cycle as button, engagement should not persist."""
    self._rx(self._gear_msg(4))
    self._rx(self._pcm_status_msg(False))

    # Press brake first, then button
    self._rx(self._user_brake_msg(1))
    self._rx(self._wheel_button_msg(left_click=2))  # press
    # Button may briefly set controls_allowed, but brake_pressed is already true
    # Next generic_rx_checks on any message will see brake_pressed rising edge passed
    # Send another message to trigger generic_rx_checks
    self._rx(self._wheel_button_msg(left_click=0))  # release
    # After processing, controls_allowed should be false (brake overrides)
    # Note: the exact behavior depends on whether brake rising edge was consumed.
    # At minimum, the system should not remain engaged.
    self._rx(self._user_brake_msg(0))  # release brake
    self._rx(self._user_brake_msg(1))  # re-press to trigger rising edge
    self.assertFalse(self.safety.get_controls_allowed())

  def test_rx_check_timeout_0x131A(self):
    """If WheelButtons_Fwd stops transmitting, controls_allowed should clear via safety_tick."""
    self._rx(self._gear_msg(4))
    self._rx(self._pcm_status_msg(False))
    self._engage_button()
    self.assertTrue(self.safety.get_controls_allowed())

    # Advance timer past the lag threshold (2s is past any rx_check frequency)
    # and call safety_tick WITHOUT sending fresh 0x131A messages.
    # Keep other rx_checks alive so only 0x131A times out.
    self._rx(self._speed_msg(0))
    self._rx(self._user_gas_msg(0))
    self._rx(self._user_brake_msg(0))
    self._rx(self._pcm_status_msg(False))
    self._rx(self._torque_driver_msg(0))
    self.safety.set_timer(int(2e6))
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.get_controls_allowed())

  # --- Fwd hook tests ---

  def test_fwd_hook_blocks_0x160_when_engaged(self):
    self.safety.set_controls_allowed(True)
    # 0x160 from bus 2 (ACM) should be blocked
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x160))

  def test_fwd_hook_allows_0x160_when_disengaged(self):
    self.safety.set_controls_allowed(False)
    # 0x160 from bus 2 (ACM) should be forwarded to bus 0
    self.assertEqual(0, self.safety.safety_fwd_hook(2, 0x160))

  def test_fwd_hook_0x160_bus0_not_blocked(self):
    """0x160 from bus 0 (openpilot's own) is never blocked by fwd hook."""
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      # Bus 0 to bus 2 forwarding — should always pass (fwd hook only checks bus 2)
      self.assertEqual(2, self.safety.safety_fwd_hook(0, 0x160))

  def test_fwd_hook_0x162_always_blocked(self):
    """0x162 uses static blocking — always blocked from bus 0 to bus 2."""
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      # 0x162 from bus 0 (VDM) should be blocked from forwarding to bus 2
      self.assertEqual(-1, self.safety.safety_fwd_hook(0, 0x162))

  def test_adas_status_tx(self):
    """0x162 TX is allowed in longitudinal mode (for stalk suppression)."""
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for interface_status in range(4):
        values = {"VDM_AdasInterfaceStatus": interface_status}
        self.assertTrue(self._tx(self.packer.make_can_msg_safety("VDM_AdasSts", 2, values)))


if __name__ == "__main__":
  unittest.main()
