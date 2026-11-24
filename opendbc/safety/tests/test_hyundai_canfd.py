#!/usr/bin/env python3
from opendbc.testing import parameterized_class
import unittest

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety.tests.hyundai_common import HyundaiButtonBase, HyundaiLongitudinalBase
from opendbc.safety.tests.libsafety import libsafety_py

# All combinations of radar/camera-SCC and gas/hybrid/EV cars
ALL_GAS_EV_HYBRID_COMBOS = [
  # Radar SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 0, "EXTRA_SAFETY_PARAM": 0},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "EXTRA_SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "EXTRA_SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS},
  # Camera SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 2, "EXTRA_SAFETY_PARAM": HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "EXTRA_SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "EXTRA_SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.CAMERA_SCC},
]


class TestHyundaiCanfdBase(HyundaiButtonBase, common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

  DBC = "hyundai_canfd_generated"
  SAFETY_MODEL = CarParams.SafetyModel.hyundaiCanfd

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  STANDSTILL_THRESHOLD = 12  # 0.375 kph
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 3
  MAX_TORQUE_LOOKUP = [0], [270]

  MAX_RT_DELTA = 112

  DRIVER_TORQUE_ALLOWANCE = 250
  DRIVER_TORQUE_FACTOR = 2

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 89
  MAX_INVALID_STEERING_FRAMES = 2

  PT_BUS = 0
  SCC_BUS = 2
  STEER_BUS = 0
  STEER_MSG = ""
  GAS_MSG = ("", "")
  BUTTONS_TX_BUS = 1

  def _torque_driver_msg(self, torque):
    values = {"MDPS_StrTqSnsrVal": torque}
    return self.packer.make_can_msg_safety("MDPS", self.PT_BUS, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"StrTqReqVal": torque, "ActToiSta": steer_req}
    return self.packer.make_can_msg_safety(self.STEER_MSG, self.STEER_BUS, values)

  def _speed_msg(self, speed, wheels=("FL", "FR", "RL", "RR")):
    values = {f"WHL_Spd{pos}Val": speed * 0.03125 for pos in wheels}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.PT_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"DriverBraking": brake}
    return self.packer.make_can_msg_safety("TCS", self.PT_BUS, values)

  def _user_gas_msg(self, gas):
    values = {self.GAS_MSG[1]: gas}
    return self.packer.make_can_msg_safety(self.GAS_MSG[0], self.PT_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACCMode": 1 if enable else 0}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def _button_msg(self, buttons, main_button=0, bus=None):
    if bus is None:
      bus = self.PT_BUS
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", bus, values)

  def test_vehicle_moving_single_wheel(self):
    # vehicle is moving if any checked wheel is above standstill
    for wheel in ("FL", "FR", "RL", "RR"):
      self.assertTrue(self._rx(self._speed_msg(self.STANDSTILL_THRESHOLD + 1, wheels=(wheel,))))
      self.assertTrue(self.safety.get_vehicle_moving())

  def test_other_gas_msgs_ignored(self):
    # the gas messages share one RX slot, but only the configured one is parsed
    for msg, sig in (("ACCELERATOR", "ACCELERATOR_PEDAL"), ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"),
                     ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED")):
      if msg != self.GAS_MSG[0]:
        self._reset_safety_hooks()
        self.assertTrue(self._rx(self.packer.make_can_msg_safety(msg, self.PT_BUS, {sig: 1})))
        self.assertFalse(self.safety.get_gas_pressed_prev())

  def test_cruise_engaged_prev(self):
    super().test_cruise_engaged_prev()
    # driver override is engaged
    self._rx(self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, {"ACCMode": 2}))
    self.assertTrue(self.safety.get_cruise_engaged_prev())


class TestHyundaiCanfdLFASteeringBase(TestHyundaiCanfdBase):

  TX_MSGS = [[0x12A, 0], [0x1A0, 1], [0x1CF, 0], [0x1E0, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0)}  # LFA, LFAHDA_CLUSTER
  FWD_BLACKLISTED_ADDRS = {2: [0x12A, 0x1E0]}

  STEER_MSG = "LFA"
  BUTTONS_TX_BUS = 2
  EXTRA_SAFETY_PARAM: int

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ in ("TestHyundaiCanfdLFASteering", "TestHyundaiCanfdLFASteeringAltButtons"):
      raise unittest.SkipTest

  @property
  def SAFETY_PARAM(self):
    return self.EXTRA_SAFETY_PARAM


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteering(TestHyundaiCanfdLFASteeringBase):
  pass


class TestHyundaiCanfdLFASteeringAltButtonsBase(TestHyundaiCanfdLFASteeringBase):

  EXTRA_SAFETY_PARAM: int

  @property
  def SAFETY_PARAM(self):
    return HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.EXTRA_SAFETY_PARAM

  def _button_msg(self, buttons, main_button=0, bus=1):
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _acc_cancel_msg(self, cancel, accel_raw=0, accel_val=0):
    values = {"ACCMode": 4 if cancel else 0, "aReqRaw": accel_raw, "aReqValue": accel_val}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def test_button_sends(self):
    """
      No button send allowed with alt buttons.
    """
    for enabled in (True, False):
      for btn in range(8):
        self.safety.set_controls_allowed(enabled)
        self.assertFalse(self._tx(self._button_msg(btn)))

  def test_acc_cancel(self):
    # FIXME: the CANFD_ALT_BUTTONS cars are the only ones that use SCC_CONTROL to cancel, why can't we use buttons?
    for enabled in (True, False):
      self.safety.set_controls_allowed(enabled)
      self.assertTrue(self._tx(self._acc_cancel_msg(True)))
      self.assertFalse(self._tx(self._acc_cancel_msg(True, accel_raw=1)))
      self.assertFalse(self._tx(self._acc_cancel_msg(True, accel_val=1)))
      self.assertFalse(self._tx(self._acc_cancel_msg(False)))


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringAltButtons(TestHyundaiCanfdLFASteeringAltButtonsBase):
  pass


class TestHyundaiCanfdLKASteeringEV(TestHyundaiCanfdBase):

  SAFETY_PARAM = HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4)}  # LKAS, CAM_0x2A4
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")


# TODO: Handle ICE and HEV configurations once we see cars that use the new messages
class TestHyundaiCanfdLKASteeringAltEV(TestHyundaiCanfdBase):

  SAFETY_PARAM = HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CANFD_LKA_STEER_MSG_ALT

  TX_MSGS = [[0x110, 0], [0x1CF, 1], [0x362, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x110, 0x362)}  # LKAS_ALT, CAM_0x362
  FWD_BLACKLISTED_ADDRS = {2: [0x110, 0x362]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS_ALT"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")


class TestHyundaiCanfdLKASteeringLongEV(HyundaiLongitudinalBase, TestHyundaiCanfdLKASteeringEV):

  SAFETY_PARAM = HyundaiSafetyFlags.CANFD_LKA_STEER_MSG | HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.EV_GAS

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0], [0x51, 0], [0x730, 1], [0x12a, 1], [0x160, 1],
             [0x1e0, 1], [0x1a0, 1], [0x1ea, 1], [0x200, 1], [0x345, 1], [0x1da, 1]]

  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4), 1: (0x1a0,)}  # LKAS, CAM_0x2A4, SCC_CONTROL

  DISABLED_ECU_UDS_MSG = (0x730, 1)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 1)

  STEER_MSG = "LFA"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
  STEER_BUS = 1

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0, accel_value=None):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel if accel_value is None else accel_value,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", 1, values)

  def test_accel_signals_checked_independently(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._accel_msg(0, accel_value=0)))
    self.assertFalse(self._tx(self._accel_msg(0, accel_value=5.0)))
    self.assertFalse(self._tx(self._accel_msg(5.0, accel_value=0)))


class TestHyundaiCanfdAngleSteering(HyundaiButtonBase, common.AngleSteeringSafetyTest, common.CarSafetyTest):

  DBC = "hyundai_canfd_generated"
  SAFETY_MODEL = CarParams.SafetyModel.hyundaiCanfd
  SAFETY_PARAM = HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.CANFD_ANGLE_STEERING

  TX_MSGS = [[0xCB, 0], [0xEA, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0xCB,), 2: (0xEA,)}
  FWD_BLACKLISTED_ADDRS = {0: [0xEA], 2: [0xCB]}

  PT_BUS = 0
  SCC_BUS = 2
  BUTTONS_TX_BUS = 2
  STANDSTILL_THRESHOLD = 12

  STEER_ANGLE_MAX = 175.0
  STEER_ANGLE_TEST_MAX = 55.0
  DEG_TO_CAN = 10.0
  ANGLE_RATE_BP = [0.0, 5.0, 25.0]
  ANGLE_RATE_UP = [1.2, 0.8, 0.2]
  ANGLE_RATE_DOWN = [1.5, 1.0, 0.3]
  LATERAL_FREQUENCY = 100

  def setUp(self):
    self.angle_counter = 0
    self.angle_frame = 0
    super().setUp()
    self.stock_lfa = {
      "FCA_ESA_ActvSta": 0,
      "BYTE7_RESERVED": 0,
      "FCA_ESA_TqBstGainVal": 0,
      **{f"BYTE{index}": 0 for index in range(9, 24)},
    }
    self._rx(self._stock_angle_msg())
    self.stock_mdps = {
      "COUNTER": 0,
      "MDPS_LkaToiActvSta": 1,
      "MDPS_LkaToiUnblSta": 0,
      "MDPS_EstStrAnglVal": 0,
      "MDPS_StrTqSnsrVal": 0,
      "MDPS_ADASAciActvSta_Lv2": 1,
      "MDPS_ADAS_AciFltSig_Lv2": 0,
    }
    self._rx(self.packer.make_can_msg_safety("MDPS", self.PT_BUS, self.stock_mdps))

  def _angle_cmd_msg(self, angle, enabled, increment_timer=True, state=None, gain=None, counter=None, reserved=None):
    if increment_timer:
      self.safety.set_timer(self.angle_frame * int(1e6 / self.LATERAL_FREQUENCY))
      self.angle_frame += 1
    if counter is None:
      self.angle_counter = (self.angle_counter + 1) & 0xFF
      counter = self.angle_counter
    values = self.stock_lfa | {
      "COUNTER": counter,
      "ADAS_ActvACISta": 0,
      "ADAS_ActvACILvl2Sta": (2 if enabled else 1) if state is None else state,
      "ADAS_StrAnglReqVal": angle,
      "ADAS_ACIAnglTqRedcGainVal": (1.0 if enabled else 0.0) if gain is None else gain,
    }
    if reserved is not None:
      values["BYTE7_RESERVED"] = reserved
    return self.packer.make_can_msg_safety("LFA_ALT", 0, values)

  def _angle_meas_msg(self, angle, **kwargs):
    values = {"MDPS_EstStrAnglVal": angle, "MDPS_StrTqSnsrVal": 0}
    values.update(kwargs)
    return self.packer.make_can_msg_safety("MDPS", self.PT_BUS, values)

  def _stock_angle_msg(self):
    return self.packer.make_can_msg_safety("LFA_ALT", self.SCC_BUS, self.stock_lfa)

  def _camera_mdps_msg(self, enabled, values=None, template=None):
    stock_mdps = self.stock_mdps if template is None else template
    camera_values = stock_mdps | {
      "MDPS_LkaToiActvSta": 0,
      "MDPS_LkaToiUnblSta": 1,
      "MDPS_ADASAciActvSta_Lv2": 2 if enabled else 1,
      "MDPS_ADAS_AciFltSig_Lv2": 4 if stock_mdps["MDPS_ADAS_AciFltSig_Lv2"] == 0
                                      else stock_mdps["MDPS_ADAS_AciFltSig_Lv2"],
    }
    if values is not None:
      camera_values.update(values)
    return self.packer.make_can_msg_safety("MDPS", 2, camera_values)

  def _speed_msg(self, speed, wheels=("FL", "FR", "RL", "RR")):
    values = {f"WHL_Spd{pos}Val": speed * 3.6 for pos in wheels}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.PT_BUS, values)

  def _vehicle_moving_msg(self, speed):
    values = {f"WHL_Spd{pos}Val": speed * 0.03125 for pos in ("FL", "FR", "RL", "RR")}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.PT_BUS, values)

  def _user_brake_msg(self, brake):
    return self.packer.make_can_msg_safety("TCS", self.PT_BUS,
                                           {"DriverBraking": brake})

  def _user_gas_msg(self, gas):
    return self.packer.make_can_msg_safety("ACCELERATOR", self.PT_BUS,
                                           {"ACCELERATOR_PEDAL": gas})

  def _pcm_status_msg(self, enable):
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS,
                                           {"ACCMode": 1 if enable else 0})

  def _button_msg(self, buttons, main_button=0, bus=None):
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", self.PT_BUS, {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    })

  def test_button_sends(self):
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for button in range(8):
        self.assertFalse(self._tx(self._button_msg(button)))

  def test_angle_reserved_and_state_fields(self):
    self._reset_angle_measurement(0)
    self.safety.set_controls_allowed(True)
    self._set_prev_desired_angle(0)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True, state=3)))
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True, gain=0.996)))
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True, reserved=1)))

  def test_stock_fca_payload_must_be_preserved(self):
    self.stock_lfa |= {"FCA_ESA_ActvSta": 1, "FCA_ESA_TqBstGainVal": 0.4, "BYTE9": 0x5A}
    self.assertTrue(self._rx(self._stock_angle_msg()))
    self._reset_angle_measurement(0)
    self.safety.set_controls_allowed(True)
    self._set_prev_desired_angle(0)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
    self.stock_lfa["BYTE9"] = 0
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))

  def test_bad_angle_crc_rejected(self):
    self._reset_angle_measurement(0)
    self.safety.set_controls_allowed(True)
    msg = self._angle_cmd_msg(0, True)
    dat = bytearray(msg.data)
    dat[0] ^= 1
    self.assertFalse(self._tx(libsafety_py.make_CANPacket(msg.addr, msg.bus, bytes(dat))))

  def test_angle_counter_jump_rejected_then_resynchronized(self):
    self._reset_angle_measurement(0)
    self.safety.set_controls_allowed(True)
    self._set_prev_desired_angle(0)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True, counter=10)))
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True, counter=12)))
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True, counter=13)))

  def test_rejected_protocol_frame_does_not_advance_rate_state(self):
    self._reset_angle_measurement(0)
    self.safety.set_controls_allowed(True)
    self._set_prev_desired_angle(0)
    self.assertFalse(self._tx(self._angle_cmd_msg(1.2, True, reserved=1)))
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._angle_cmd_msg(2.4, True)))

  def test_angle_measured_error(self):
    self._reset_angle_measurement(0)
    self.safety.set_controls_allowed(True)
    self._set_prev_desired_angle(10.0)
    self.assertTrue(self._tx(self._angle_cmd_msg(10.0, True)))
    self.safety.set_controls_allowed(True)
    self._set_prev_desired_angle(10.1)
    self.assertFalse(self._tx(self._angle_cmd_msg(10.1, True)))

  def test_driver_override_blocks_active_angle(self):
    self._reset_angle_measurement(0)
    self._rx(self._angle_meas_msg(0, MDPS_StrTqSnsrVal=251))
    self.safety.set_controls_allowed(True)
    self._set_prev_desired_angle(0)
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))

  def test_camera_mdps_mirror_contract(self):
    self._reset_angle_measurement(0)
    self._rx(self.packer.make_can_msg_safety("MDPS", self.PT_BUS, self.stock_mdps))
    self.safety.set_controls_allowed(True)
    self._set_prev_desired_angle(0)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
    self.assertTrue(self._tx(self._camera_mdps_msg(True)))

  def test_camera_mdps_mirror_rejects_non_contract_changes(self):
    for values in ({"MDPS_PaStrAnglVal": 1.0}, {"MDPS_LkaToiActvSta": 1},
                   {"MDPS_ADASAciActvSta_Lv2": 1}, {"MDPS_ADAS_AciFltSig_Lv2": 0}):
      self._reset_safety_hooks()
      self._rx(self._stock_angle_msg())
      self._rx(self.packer.make_can_msg_safety("MDPS", self.PT_BUS, self.stock_mdps))
      self.safety.set_controls_allowed(True)
      self._set_prev_desired_angle(0)
      self.assertTrue(self._tx(self._angle_cmd_msg(0, True, counter=1)))
      self.assertFalse(self._tx(self._camera_mdps_msg(True, values)))

  def test_camera_mdps_mirror_preserves_real_fault(self):
    self._reset_safety_hooks()
    self.stock_mdps["MDPS_ADAS_AciFltSig_Lv2"] = 1
    self._rx(self._stock_angle_msg())
    self._rx(self.packer.make_can_msg_safety("MDPS", self.PT_BUS, self.stock_mdps))
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False, counter=1)))
    self.assertTrue(self._tx(self._camera_mdps_msg(False)))

  def test_camera_mdps_mirror_accepts_one_frame_lag_only(self):
    self._reset_safety_hooks()
    previous = self.stock_mdps | {"COUNTER": 10, "MDPS_EstStrAnglVal": 1.0}
    current = self.stock_mdps | {"COUNTER": 11, "MDPS_EstStrAnglVal": 2.0}
    self._rx(self._stock_angle_msg())
    self._rx(self.packer.make_can_msg_safety("MDPS", self.PT_BUS, previous))
    self._rx(self.packer.make_can_msg_safety("MDPS", self.PT_BUS, current))
    self.safety.set_controls_allowed(True)
    self._set_prev_desired_angle(2.0)
    self.assertTrue(self._tx(self._angle_cmd_msg(2.0, True, counter=1)))
    self.assertTrue(self._tx(self._camera_mdps_msg(True, {"COUNTER": 20}, previous)))

    newest = self.stock_mdps | {"COUNTER": 12, "MDPS_EstStrAnglVal": 3.0}
    self._rx(self.packer.make_can_msg_safety("MDPS", self.PT_BUS, newest))
    self.assertFalse(self._tx(self._camera_mdps_msg(True, {"COUNTER": 21}, previous)))
    self.assertTrue(self._tx(self._camera_mdps_msg(True, {"COUNTER": 22}, newest)))

  def test_active_angle_requires_measured_angle_inside_sixty_degrees(self):
    for measured, desired in ((60.0, 59.0), (65.0, 60.0), (-60.0, -59.0), (-65.0, -60.0)):
      self._reset_angle_measurement(measured)
      self.safety.set_controls_allowed(True)
      self._set_prev_desired_angle(desired)
      self.assertFalse(self._tx(self._angle_cmd_msg(desired, True)))
      self.assertTrue(self._tx(self._angle_cmd_msg(measured, False)))

  def test_inactive_high_angle_clamps_to_command_limit(self):
    for measured, commanded in ((300.0, self.STEER_ANGLE_MAX), (-300.0, -self.STEER_ANGLE_MAX)):
      self._reset_angle_measurement(measured)
      self.safety.set_controls_allowed(False)
      self.assertTrue(self._tx(self._angle_cmd_msg(commanded, False)))

  def test_angle_cmd_when_disabled(self):
    # Inactive frames must track the measured angle exactly. Active frames are
    # additionally constrained to a 10 degree envelope around the measurement.
    for controls_allowed in (True, False):
      for angle_meas in range(-90, 91, 10):
        self._reset_angle_measurement(angle_meas)
        for angle_cmd in range(-90, 91, 10):
          self.safety.set_controls_allowed(controls_allowed)
          self._set_prev_desired_angle(angle_cmd)
          self.assertEqual(angle_cmd == angle_meas,
                           self._tx(self._angle_cmd_msg(angle_cmd, False)))

          self.safety.set_controls_allowed(controls_allowed)
          self._set_prev_desired_angle(angle_cmd)
          should_tx = controls_allowed and abs(angle_meas) < 60 and abs(angle_cmd) <= 60 and abs(angle_cmd - angle_meas) <= 10
          self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(angle_cmd, True)))

  def test_mdps_invalid_and_fault_clear_controls(self):
    for values in ({"MDPS_EstStrAnglVal": 3276.6}, {"MDPS_LkaFailSta": 1},
                   {"MDPS_ADAS_AciFltSig": 1}, {"MDPS_ADAS_AciFltSig_Lv2": 1}):
      self._reset_safety_hooks()
      self.safety.set_controls_allowed(True)
      self._rx(self._angle_meas_msg(0, **values))
      self.assertFalse(self.safety.get_controls_allowed())

  def test_mdps_lv2_fault_semantics(self):
    # 2 is an external ADAS-source fault, not an MDPS measurement or actuator
    # fault. MDPS-internal faults and temporarily-unavailable states remain
    # immediate disengagement conditions.
    for status in range(8):
      self._reset_safety_hooks()
      self.safety.set_controls_allowed(True)
      self._rx(self._angle_meas_msg(0, MDPS_ADAS_AciFltSig_Lv2=status))
      self.assertEqual(status in (0, 2), self.safety.get_controls_allowed(), status)


class TestHyundaiCanfdAngleSteeringLong(HyundaiLongitudinalBase, TestHyundaiCanfdAngleSteering):

  SAFETY_PARAM = (HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC |
                  HyundaiSafetyFlags.CANFD_ANGLE_STEERING | HyundaiSafetyFlags.LONG)

  TX_MSGS = [[0xCB, 0], [0xEA, 2], [0x1A0, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0xCB, 0x1A0), 2: (0xEA,)}
  FWD_BLACKLISTED_ADDRS = {0: [0xEA], 2: [0xCB, 0x1A0]}

  DISABLED_ECU_UDS_MSG = (0x7D0, 0)
  DISABLED_ECU_ACTUATION_MSG = (0x1A0, 0)

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0, accel_value=None):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel if accel_value is None else accel_value,
      "ACCMode": 1,
      "MainMode_ACC": 1,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", 0, values)

  def test_accel_signals_checked_independently(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._accel_msg(0, accel_value=0)))
    self.assertFalse(self._tx(self._accel_msg(0, accel_value=5.0)))
    self.assertFalse(self._tx(self._accel_msg(5.0, accel_value=0)))

  def test_tester_present_allowed(self, ecu_disable=True):
    # This profile coexists with the stock ADAS/radar ECUs and must never send
    # diagnostic keepalive or disable traffic.
    super().test_tester_present_allowed(ecu_disable=False)


# Tests longitudinal for ICE, hybrid, EV cars with LFA steering
class TestHyundaiCanfdLFASteeringLongBase(HyundaiLongitudinalBase, TestHyundaiCanfdLFASteeringBase):

  FWD_BLACKLISTED_ADDRS = {2: [0x12a, 0x1e0, 0x1a0, 0x160]}

  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0, 0x1a0, 0x160)}  # LFA, LFAHDA_CLUSTER, SCC_CONTROL, ADRV_0x160

  DISABLED_ECU_UDS_MSG = (0x7D0, 0)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 0)

  @property
  def SAFETY_PARAM(self):
    return HyundaiSafetyFlags.LONG | self.EXTRA_SAFETY_PARAM

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0, accel_value=None):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel if accel_value is None else accel_value,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", 0, values)

  def test_accel_signals_checked_independently(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._accel_msg(0, accel_value=0)))
    self.assertFalse(self._tx(self._accel_msg(0, accel_value=5.0)))
    self.assertFalse(self._tx(self._accel_msg(5.0, accel_value=0)))

  def test_tester_present_allowed(self, ecu_disable: bool = True):
    super().test_tester_present_allowed(ecu_disable=not self.EXTRA_SAFETY_PARAM & HyundaiSafetyFlags.CAMERA_SCC)


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLong(TestHyundaiCanfdLFASteeringLongBase):
  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLong":
      raise unittest.SkipTest


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLongAltButtons(TestHyundaiCanfdLFASteeringLongBase, TestHyundaiCanfdLFASteeringAltButtonsBase):
  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongAltButtons":
      raise unittest.SkipTest

  @property
  def SAFETY_PARAM(self):
    return HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.EXTRA_SAFETY_PARAM

  def test_acc_cancel(self):
    # Alt buttons does not use SCC_CONTROL to cancel if longitudinal
    pass


if __name__ == "__main__":
  unittest.main()
