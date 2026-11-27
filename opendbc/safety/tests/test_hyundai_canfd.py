#!/usr/bin/env python3
from collections.abc import Callable
from opendbc.testing import parameterized_class
import unittest

import numpy as np

from opendbc.car.hyundai.carcontroller import CarController, HyundaiCanFDAngleParams
from opendbc.car.hyundai.tests import test_hyundai as hyundai_car_tests
from opendbc.car.hyundai.values import CAR, DBC
from opendbc.car.structs import CarControl, CarState
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.structs import CarParams
import opendbc.safety.tests.common as common
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from opendbc.safety.tests.hyundai_common import Buttons, HyundaiAlwaysOnLateralBase, HyundaiButtonBase, HyundaiLongitudinalBase
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


class HyundaiCanfdPedalResumeBase(HyundaiLongitudinalBase):
  """
    ALT_EXP_PEDAL_RESUME: the driver's brake or accelerator release re-allows controls while
    TCS reports cruise available (ACCEnable 0) and no CANCEL was pressed since the last SET/RES.
    Brake, CANCEL and cruise unavailable still exit controls. Needs openpilot longitudinal.
  """

  PT_BUS: int
  _after_safety_reset: Callable[[], None]
  _moving_msg: Callable[..., libsafety_py.CANPacket]
  _user_gas_msg: Callable[..., libsafety_py.CANPacket]

  def setUp(self):
    super().setUp()
    self._pedal_resume()

  def _pedal_resume(self, alt_exp=ALTERNATIVE_EXPERIENCE.PEDAL_RESUME, safety_param=None):
    # fresh safety state as in setUp (init_tests keeps the relay malfunction checks active)
    self.safety.set_safety_hooks(self.SAFETY_MODEL, self.SAFETY_PARAM if safety_param is None else safety_param)
    self.safety.init_tests()
    self._after_safety_reset()
    self.safety.set_alternative_experience(alt_exp)

  def _tcs_msg(self, brake, acc_enable=0):
    return self.packer.make_can_msg_safety("TCS", self.PT_BUS, {"DriverBraking": brake, "ACCEnable": acc_enable})

  def _brake_tap(self, acc_enable=0):
    self._rx(self._tcs_msg(1, acc_enable))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._tcs_msg(0, acc_enable))

  def _gas_tap(self):
    self._rx(self._user_gas_msg(1))
    self._rx(self._user_gas_msg(0))

  def test_pedal_resume_brake_release(self):
    for moving in (False, True):
      self._pedal_resume()
      self._rx(self._moving_msg(moving))
      self._rx(self._tcs_msg(0))
      self.assertFalse(self.safety.get_controls_allowed())
      self._brake_tap()
      self.assertTrue(self.safety.get_controls_allowed())

  def test_pedal_resume_gas_release(self):
    for moving in (False, True):
      self._pedal_resume()
      self._rx(self._moving_msg(moving))
      self._rx(self._tcs_msg(0))
      self._rx(self._user_gas_msg(1))
      self.assertFalse(self.safety.get_controls_allowed())
      self._rx(self._user_gas_msg(0))
      self.assertTrue(self.safety.get_controls_allowed())

  def test_pedal_resume_needs_release(self):
    self._rx(self._tcs_msg(0))
    for _ in range(10):
      self._rx(self._tcs_msg(0))
      self._rx(self._user_gas_msg(0))
      self._rx(self._button_msg(Buttons.NONE))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._tcs_msg(1))
    self._rx(self._user_gas_msg(1))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_pedal_resume_gas_release_while_braking(self):
    self._rx(self._tcs_msg(1))
    self._gas_tap()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_pedal_resume_cruise_unavailable(self):
    for acc_enable in (1, 2, 3):
      self._pedal_resume()
      self._brake_tap(acc_enable)
      self.assertFalse(self.safety.get_controls_allowed())
      self._gas_tap()
      self.assertFalse(self.safety.get_controls_allowed())
      # unavailable exits controls, also after a SET engagement
      self._rx(self._tcs_msg(0))
      self._cruise_engage()
      self.assertTrue(self.safety.get_controls_allowed())
      self._rx(self._tcs_msg(0, acc_enable))
      self.assertFalse(self.safety.get_controls_allowed())

  def test_pedal_resume_unavailable_before_tcs(self):
    # cruise counts as unavailable until the first TCS frame
    self._gas_tap()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_pedal_resume_brake_exits(self):
    self._rx(self._moving_msg(True))
    self._brake_tap()
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._tcs_msg(1))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_pedal_resume_cancel_blocks_until_set_resume(self):
    for unblock in (Buttons.SET, Buttons.RESUME):
      self._pedal_resume()
      self._brake_tap()
      self.assertTrue(self.safety.get_controls_allowed())
      self._rx(self._button_msg(Buttons.CANCEL))
      self.assertFalse(self.safety.get_controls_allowed())
      self._rx(self._button_msg(Buttons.NONE))
      self._brake_tap()
      self.assertFalse(self.safety.get_controls_allowed())
      self._gas_tap()
      self.assertFalse(self.safety.get_controls_allowed())
      # the main and LFA buttons do not clear the block
      self._rx(self._button_msg(Buttons.NONE, main_button=1, lfa_button=1))
      self._rx(self._button_msg(Buttons.NONE))
      self._brake_tap()
      self.assertFalse(self.safety.get_controls_allowed())
      # CANCEL straight to SET/RES and held: not yet a SET/RES release
      self._rx(self._button_msg(Buttons.CANCEL))
      self._rx(self._button_msg(unblock))
      self.assertFalse(self.safety.get_controls_allowed())
      self._brake_tap()
      self.assertFalse(self.safety.get_controls_allowed())
      self._rx(self._button_msg(Buttons.NONE))
      self.assertTrue(self.safety.get_controls_allowed())
      self.safety.set_controls_allowed(False)
      self._brake_tap()
      self.assertTrue(self.safety.get_controls_allowed())

  def test_pedal_resume_main_button_does_not_block(self):
    self._rx(self._button_msg(Buttons.NONE, main_button=1))
    self._rx(self._button_msg(Buttons.NONE))
    self._brake_tap()
    self.assertTrue(self.safety.get_controls_allowed())

  def test_pedal_resume_needs_flag(self):
    self._pedal_resume(ALTERNATIVE_EXPERIENCE.DEFAULT)
    self._brake_tap()
    self.assertFalse(self.safety.get_controls_allowed())
    self._gas_tap()
    self.assertFalse(self.safety.get_controls_allowed())
    # without the flag, cruise unavailable does not exit controls (unchanged)
    self._cruise_engage()
    self._rx(self._tcs_msg(0, 1))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_pedal_resume_needs_longitudinal(self):
    self._pedal_resume(safety_param=self.SAFETY_PARAM & ~HyundaiSafetyFlags.LONG)
    self._rx(self._tcs_msg(0))
    self._brake_tap()
    self._gas_tap()
    self.assertFalse(self.safety.get_controls_allowed())


class TestHyundaiCanfdBase(HyundaiButtonBase, HyundaiAlwaysOnLateralBase, common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest,
                           common.SteerRequestCutSafetyTest):

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
  LFA_BUTTON = True

  def _lateral_cmd_allowed(self):
    self._set_prev_torque(0)
    return self._tx(self._torque_cmd_msg(self.MAX_RATE_UP))

  def _stale_counter_msg(self):
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.PT_BUS, {"COUNTER": 0})

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

  def _button_msg(self, buttons, main_button=0, bus=None, lfa_button=0):
    if bus is None:
      bus = self.PT_BUS
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
      "LDA_BTN": lfa_button,
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

  def _button_msg(self, buttons, main_button=0, bus=1, lfa_button=0):
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
      "LDA_BTN": lfa_button,
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


class TestHyundaiCanfdAngleSteering(HyundaiButtonBase, HyundaiAlwaysOnLateralBase, common.AngleSteeringSafetyTest, common.CarSafetyTest):

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
  LFA_BUTTON = True

  STEER_ANGLE_MAX = 175.0
  DEG_TO_CAN = 10.0
  LATERAL_FREQUENCY = 100
  # speeds where the vehicle model jerk step stays inside the 10 degree measured error
  VM_TEST_SPEEDS = (5.0, 8.0, 12.0, 20.0, 30.0, 40.0)

  def setUp(self):
    self.angle_counter = 0
    self.angle_frame = 0
    self.VM = VehicleModel(CarInterface.get_non_essential_params("HYUNDAI_IONIQ_9"))
    super().setUp()
    self.stock_lfa = {
      "ADAS_ActvACILvl2Sta": 1,
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

  def _camera_mdps_msg(self, state, values=None, template=None):
    stock_mdps = self.stock_mdps if template is None else template
    camera_values = stock_mdps | {
      "MDPS_LkaToiActvSta": 0,
      "MDPS_LkaToiUnblSta": 1,
      "MDPS_ADASAciActvSta_Lv2": state,
      "MDPS_ADAS_AciFltSig_Lv2": 4 if stock_mdps["MDPS_ADAS_AciFltSig_Lv2"] == 0
                                      else stock_mdps["MDPS_ADAS_AciFltSig_Lv2"],
    }
    if values is not None:
      camera_values.update(values)
    return self.packer.make_can_msg_safety("MDPS", 2, camera_values)

  def _speed_msg(self, speed, wheels=("FL", "FR", "RL", "RR")):
    values = {f"WHL_Spd{pos}Val": speed * 3.6 for pos in wheels}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.PT_BUS, values)

  def _set_vm_speed(self, speed):
    # safety fudges the speed by -1 m/s; return the speed it uses, after wheel speed and sample rounding
    kph = round((speed + 1.0) * 3.6 / 0.03125) * 0.03125
    self._reset_speed_measurement(kph / 3.6)
    return max(common.round_speed(kph / 3.6) - 1.0, 1.0)

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

  def _button_msg(self, buttons, main_button=0, bus=None, lfa_button=0):
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", self.PT_BUS, {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
      "LDA_BTN": lfa_button,
    })

  def _moving_msg(self, moving):
    return self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 1 if moving else 0)

  def _stale_counter_msg(self):
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.PT_BUS, {"COUNTER": 0})

  def _after_safety_reset(self):
    self._rx(self._stock_angle_msg())
    self._rx(self.packer.make_can_msg_safety("MDPS", self.PT_BUS, self.stock_mdps))

  def _lateral_cmd_allowed(self):
    self._reset_angle_measurement(0)
    self._set_prev_desired_angle(0)
    return self._tx(self._angle_cmd_msg(0, True))

  def test_always_on_lateral_mdps_fault(self):
    self._always_on()
    self._engage_without_cruise()
    self.assertTrue(self._lateral_cmd_allowed())
    # an MDPS fault blocks active control without cruise, inactive frames still pass
    self._rx(self._angle_meas_msg(0, MDPS_LkaFailSta=1))
    self.assertTrue(self.safety.get_lateral_engaged())
    self._set_prev_desired_angle(0)
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))
    # an invalid angle too
    self._rx(self._angle_meas_msg(0x7FFF / 10.0))
    self._set_prev_desired_angle(0)
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))
    # the fault clears with the next good frame
    self.assertTrue(self._lateral_cmd_allowed())

  def test_always_on_lateral_mdps_fault_with_controls_allowed(self):
    # with controls_allowed the stock behavior is unchanged: the fault drops controls_allowed
    self._always_on()
    self._cruise_engage()
    self._rx(self._angle_meas_msg(0, MDPS_LkaFailSta=1))
    self.assertFalse(self.safety.get_controls_allowed())
    self.safety.set_controls_allowed(True)
    self._set_prev_desired_angle(0)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

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
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True, gain=0.996)))
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True, gain=0.004)))
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True, gain=0.0)))
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True, gain=1.004)))
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True, state=3)))
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
    # at 25 m/s the vehicle model allows one 0.3 degree step per frame
    speed = self._set_vm_speed(25.0)
    step = int(get_max_angle_delta_vm(speed, self.VM, HyundaiCanFDAngleParams) * self.DEG_TO_CAN + 1) / self.DEG_TO_CAN
    self._reset_angle_measurement(0)
    self.safety.set_controls_allowed(True)
    self._set_prev_desired_angle(0)
    self.assertFalse(self._tx(self._angle_cmd_msg(step, True, reserved=1)))
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._angle_cmd_msg(2 * step, True)))

  def test_angle_measured_error(self):
    self._reset_angle_measurement(0)
    self.safety.set_controls_allowed(True)
    self._set_prev_desired_angle(10.0)
    self.assertTrue(self._tx(self._angle_cmd_msg(10.0, True)))
    self.safety.set_controls_allowed(True)
    self._set_prev_desired_angle(10.1)
    self.assertFalse(self._tx(self._angle_cmd_msg(10.1, True)))

  def test_driver_torque_does_not_block_active_angle(self):
    # the controller lowers the MDPS torque ceiling instead of releasing; panda has no driver torque check
    for torque in (251, 600, -251, -600):
      self._reset_angle_measurement(0)
      self._rx(self._angle_meas_msg(0, MDPS_StrTqSnsrVal=torque))
      self.safety.set_controls_allowed(True)
      self._set_prev_desired_angle(0)
      self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
      self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))

  def test_camera_mdps_mirror_contract(self):
    self._reset_angle_measurement(0)
    self._rx(self.packer.make_can_msg_safety("MDPS", self.PT_BUS, self.stock_mdps))
    self.safety.set_controls_allowed(True)
    self._set_prev_desired_angle(0)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
    self.assertTrue(self._tx(self._camera_mdps_msg(1, {"COUNTER": 10})))
    self.assertFalse(self._tx(self._camera_mdps_msg(2, {"COUNTER": 11})))

    self.stock_lfa["ADAS_ActvACILvl2Sta"] = 2
    self.assertTrue(self._rx(self._stock_angle_msg()))
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False)))
    self.assertTrue(self._tx(self._camera_mdps_msg(2, {"COUNTER": 12})))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_camera_mdps_mirror_rejects_non_contract_changes(self):
    for values in ({"MDPS_PaStrAnglVal": 1.0}, {"MDPS_LkaToiActvSta": 1},
                   {"MDPS_ADASAciActvSta_Lv2": 2}, {"MDPS_ADAS_AciFltSig_Lv2": 0}):
      self._reset_safety_hooks()
      self._rx(self._stock_angle_msg())
      self._rx(self.packer.make_can_msg_safety("MDPS", self.PT_BUS, self.stock_mdps))
      self.safety.set_controls_allowed(True)
      self._set_prev_desired_angle(0)
      self.assertTrue(self._tx(self._angle_cmd_msg(0, True, counter=1)))
      self.assertFalse(self._tx(self._camera_mdps_msg(1, values)))

  def test_camera_mdps_mirror_preserves_real_fault(self):
    self._reset_safety_hooks()
    self.stock_mdps |= {"MDPS_ADAS_AciFltSig_Lv2": 1, "MDPS_ADASAciActvSta_Lv2": 3}
    self.stock_lfa["ADAS_ActvACILvl2Sta"] = 2
    self._rx(self._stock_angle_msg())
    self._rx(self.packer.make_can_msg_safety("MDPS", self.PT_BUS, self.stock_mdps))
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._angle_cmd_msg(0, False, counter=1)))
    self.assertTrue(self._tx(self._camera_mdps_msg(3, {"COUNTER": 10})))
    self.assertFalse(self._tx(self._camera_mdps_msg(2, {"COUNTER": 11})))
    self.assertFalse(self._tx(self._camera_mdps_msg(3, {"COUNTER": 12, "MDPS_ADAS_AciFltSig_Lv2": 4})))

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
    self.assertTrue(self._tx(self._camera_mdps_msg(1, {"COUNTER": 20}, previous)))

    newest = self.stock_mdps | {"COUNTER": 12, "MDPS_EstStrAnglVal": 3.0}
    self._rx(self.packer.make_can_msg_safety("MDPS", self.PT_BUS, newest))
    self.assertFalse(self._tx(self._camera_mdps_msg(1, {"COUNTER": 21}, previous)))
    self.assertTrue(self._tx(self._camera_mdps_msg(1, {"COUNTER": 22}, newest)))

  def test_active_angle_allowed_at_high_measured_angle(self):
    for sign in (1, -1):
      for measured, desired in ((60.0, 59.0), (65.0, 60.0), (120.0, 125.0), (175.0, 170.0), (175.0, 175.0)):
        self._reset_angle_measurement(sign * measured)
        self.safety.set_controls_allowed(True)
        self._set_prev_desired_angle(sign * desired)
        self.assertTrue(self._tx(self._angle_cmd_msg(sign * desired, True)))
        self.assertTrue(self._tx(self._angle_cmd_msg(sign * measured, False)))

  def test_active_angle_absolute_limit(self):
    # at low speed the vehicle model allows more than 175 degrees; the absolute limit still holds
    for sign in (1, -1):
      self._reset_angle_measurement(sign * 175.0)
      self.safety.set_controls_allowed(True)
      self._set_prev_desired_angle(sign * 175.0)
      self.assertTrue(self._tx(self._angle_cmd_msg(sign * 175.0, True)))
      self.assertFalse(self._tx(self._angle_cmd_msg(sign * 175.1, True)))

  def test_angle_cmd_when_enabled(self):
    # the speed table is replaced by the vehicle model limits, see test_lateral_accel_limit and test_lateral_jerk_limit
    pass

  def test_carcontroller_frames_pass_safety(self):
    # 0xCB frames from the real controller: speeds 0~35 m/s, requests up to 300 degrees, driver torque up to 800,
    # and a driver turning past 175 + 8 degrees. The wheel follows the sent angle with a lag.
    fingerprint = hyundai_car_tests.TestHyundaiFingerprint._ioniq9_fingerprint()
    car_fw = hyundai_car_tests.TestHyundaiFingerprint._ioniq9_fw()
    CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, car_fw, False, False, False)
    frames = 900
    t = np.arange(frames) * 0.01
    for speed in (0.0, 3.0, 8.0, 15.0, 25.0, 35.0):
      self._reset_safety_hooks()
      self._after_safety_reset()
      self.angle_frame = 0
      self.safety.set_controls_allowed(True)
      self._reset_speed_measurement(speed)
      controller = CarController(DBC[CP.carFingerprint], CP)
      out = CarState()
      out.vEgo = speed
      out.vEgoRaw = speed
      out.cruiseState.enabled = True
      CS = hyundai_car_tests.SimpleNamespace(out=out, lfa_alt=self.stock_lfa, mdps=self.stock_mdps, is_metric=True)
      CC = CarControl()
      CC.enabled = True
      CC.latActive = True
      measured = 0.0
      blocked = []
      active_frames = 0
      max_sent = 0.0
      for i in range(frames):
        request = 300.0 * np.sin(2 * np.pi * t[i] / 9.0)
        torque = float(np.interp(t[i], [0, 2, 2.5, 4, 4.5, 6, 6.5, 9], [0, 0, 225, 225, 800, 800, -800, -800]))
        driver_past_limit = 5.0 <= t[i] < 5.5 and speed < 5.0
        CS.out.steeringTorque = torque
        CS.out.steeringPressed = abs(torque) > 250
        CS.out.steeringAngleDeg = float(measured)
        CC.actuators.steeringAngleDeg = float(request)
        _, msgs = controller.update(CC.as_reader(), CS, 0)
        address, dat, _ = next(msg for msg in msgs if msg[0] == 0xCB)
        self.safety.set_timer(self.angle_frame * int(1e6 / self.LATERAL_FREQUENCY))
        self.angle_frame += 1
        if not self._tx(libsafety_py.make_CANPacket(address, 0, bytes(dat))):
          blocked.append((speed, i))
        active_frames += (dat[3] >> 4) == 2
        # the wheel follows the sent angle; a driver can turn it past the command limit
        sent = controller.apply_angle_last
        max_sent = max(max_sent, abs(sent))
        measured = 190.0 * np.sign(request) if driver_past_limit else measured + 0.2 * (sent - measured)
        self._rx(self._angle_meas_msg(round(measured, 1), MDPS_StrTqSnsrVal=torque))
      self.assertEqual(blocked, [], speed)
      self.assertGreater(active_frames, frames * 0.8, speed)
      if speed < 5.0:
        self.assertEqual(max_sent, 175.0)

  def test_lateral_accel_limit(self):
    for speed in self.VM_TEST_SPEEDS:
      vm_speed = self._set_vm_speed(speed)
      max_angle_can = int(get_max_angle_vm(vm_speed, self.VM, HyundaiCanFDAngleParams) * self.DEG_TO_CAN + 1)
      limit_can = min(max_angle_can, int(self.STEER_ANGLE_MAX * self.DEG_TO_CAN))
      for sign in (1, -1):
        for angle_can, should_tx in ((limit_can, True), (limit_can + 1, False)):
          angle = sign * angle_can / self.DEG_TO_CAN
          self._reset_angle_measurement(angle)
          self.safety.set_controls_allowed(True)
          self._set_prev_desired_angle(angle)
          self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(angle, True)), (speed, angle))

  def test_lateral_jerk_limit(self):
    for speed in self.VM_TEST_SPEEDS:
      vm_speed = self._set_vm_speed(speed)
      delta_can = int(get_max_angle_delta_vm(vm_speed, self.VM, HyundaiCanFDAngleParams) * self.DEG_TO_CAN + 1)
      self.assertLess(delta_can, 100)
      # start inside the lateral accel limit
      start_max = min(30.0, round(get_max_angle_vm(vm_speed, self.VM, HyundaiCanFDAngleParams) / 2.0))
      for start in (0.0, start_max, -start_max):
        for sign in (1, -1):
          for step_can, should_tx in ((delta_can, True), (delta_can + 1, False)):
            self._reset_angle_measurement(start)
            self.safety.set_controls_allowed(True)
            self._set_prev_desired_angle(start)
            angle = start + sign * step_can / self.DEG_TO_CAN
            self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(angle, True)), (speed, start, angle))

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
          should_tx = controls_allowed and abs(angle_cmd - angle_meas) <= 10
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

  # *** CCNC_0x161: host copy in place of the stock camera cluster frame ***

  CCNC_PERIOD_US = 50000  # stock frame every 50 ms
  CCNC_HOST_DELAY_US = 20000

  def _ccnc_msg(self, counter, alerts_2=0, sounds_2=0):
    return self.packer.make_can_msg_safety("CCNC_0x161", 0, {"COUNTER": counter, "ALERTS_2": alerts_2, "SOUNDS_2": sounds_2,
                                                             "LKA_ICON": 4, "SETSPEED_SPEED": 80})

  def _ccnc_stock_forwarded(self, t_us):
    self.safety.set_timer(t_us)
    return self.safety.safety_fwd_hook(2, 0x161) == 0

  def _ccnc_copy(self, t_us, counter, msg=None):
    self.safety.set_timer(t_us)
    return self._tx(self._ccnc_msg(counter) if msg is None else msg)

  def _ccnc_cycles(self, start_frame, frames, copy=True):
    # one stock frame per period, the host copy follows each stock frame; returns (forwarded, copy sent) per frame
    out = []
    for frame in range(start_frame, start_frame + frames):
      t = frame * self.CCNC_PERIOD_US
      forwarded = self._ccnc_stock_forwarded(t)
      sent = self._ccnc_copy(t + self.CCNC_HOST_DELAY_US, frame & 0xFF) if copy else None
      out.append((forwarded, sent))
    return out

  def test_ccnc_copy_replaces_stock_while_steering(self):
    self.safety.set_controls_allowed(True)
    cycles = self._ccnc_cycles(1, 30)
    # first stock frame goes out before the host is known; its late copy is dropped so no counter is sent twice
    self.assertEqual(cycles[0], (True, False))
    # afterwards each stock frame is replaced by exactly one host copy
    self.assertEqual(cycles[1:], [(False, True)] * 29)

  def test_ccnc_copy_sent_only_once_per_stock_frame(self):
    self.safety.set_controls_allowed(True)
    self._ccnc_cycles(1, 3)
    self.assertFalse(self._ccnc_stock_forwarded(4 * self.CCNC_PERIOD_US))
    self.assertTrue(self._ccnc_copy(4 * self.CCNC_PERIOD_US + 10000, 4))
    self.assertFalse(self._ccnc_copy(4 * self.CCNC_PERIOD_US + 20000, 4))

  def test_ccnc_stock_when_not_steering(self):
    self.safety.set_controls_allowed(False)
    self.assertEqual(self._ccnc_cycles(1, 20), [(True, False)] * 20)
    # stock is forwarded with no host copy at all
    self.safety.set_controls_allowed(True)
    self.assertEqual(self._ccnc_cycles(21, 20, copy=False), [(True, None)] * 20)

  def test_ccnc_stock_after_disengage(self):
    self.safety.set_controls_allowed(True)
    self.assertEqual(self._ccnc_cycles(1, 5)[1:], [(False, True)] * 4)
    self.safety.set_controls_allowed(False)
    self.assertEqual(self._ccnc_cycles(6, 10), [(True, False)] * 10)

  def test_ccnc_copy_with_always_on_lateral(self):
    self._always_on()
    self._engage_without_cruise()
    self.assertEqual(self._ccnc_cycles(1, 5)[1:], [(False, True)] * 4)
    self._rx(self._user_brake_msg(True))
    self.assertEqual(self._ccnc_cycles(6, 3), [(True, False)] * 3)

  def test_ccnc_stale_copy_forwards_stock(self):
    max_age = 150000
    for gap, forwarded in ((max_age - 1, False), (max_age, True), (max_age + 50000, True)):
      self._reset_safety_hooks()
      self._after_safety_reset()
      self.safety.set_controls_allowed(True)
      self._ccnc_cycles(1, 3)
      last_copy = 3 * self.CCNC_PERIOD_US + self.CCNC_HOST_DELAY_US
      self.assertEqual(self._ccnc_stock_forwarded(last_copy + gap), forwarded, gap)

  def test_ccnc_bad_checksum_copy_forwards_stock(self):
    self.safety.set_controls_allowed(True)
    self._ccnc_cycles(1, 3)
    t = 4 * self.CCNC_PERIOD_US
    self.assertFalse(self._ccnc_stock_forwarded(t))
    msg = self._ccnc_msg(4)
    dat = bytearray(msg.data)[:32]
    dat[0] ^= 1
    self.assertFalse(self._ccnc_copy(t + self.CCNC_HOST_DELAY_US, 4, libsafety_py.make_CANPacket(msg.addr, msg.bus, bytes(dat))))
    # the held-back frame is lost once, then stock is forwarded until a valid copy arrives
    self.assertEqual(self._ccnc_cycles(5, 5, copy=False), [(True, None)] * 5)
    self.assertEqual(self._ccnc_cycles(10, 3), [(True, False), (False, True), (False, True)])

  def test_ccnc_late_copy_forwards_stock(self):
    self.safety.set_controls_allowed(True)
    self._ccnc_cycles(1, 3)
    t = 4 * self.CCNC_PERIOD_US
    self.assertFalse(self._ccnc_stock_forwarded(t))
    # the copy of frame 4 has not arrived when frame 5 comes: frame 5 is forwarded and the late copy is dropped
    self.assertTrue(self._ccnc_stock_forwarded(t + self.CCNC_PERIOD_US))
    self.assertFalse(self._ccnc_copy(t + self.CCNC_PERIOD_US + 5000, 4))
    self.assertEqual(self._ccnc_cycles(6, 3), [(False, True)] * 3)

  def test_ccnc_copy_needs_stock_frame(self):
    self.safety.set_controls_allowed(True)
    # host copies alone never reach the cluster without a held-back stock frame
    for frame in range(1, 10):
      self.assertFalse(self._ccnc_copy(frame * self.CCNC_PERIOD_US, frame))

  def test_ccnc_other_buses_and_modes_forwarded(self):
    self.safety.set_controls_allowed(True)
    self._ccnc_cycles(1, 3)
    self.safety.set_timer(4 * self.CCNC_PERIOD_US)
    self.assertEqual(self.safety.safety_fwd_hook(0, 0x161), 2)
    self.assertEqual(self.safety.safety_fwd_hook(2, 0x162), 0)


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


class TestHyundaiCanfdAngleSteeringLongPedalResume(HyundaiCanfdPedalResumeBase, TestHyundaiCanfdAngleSteeringLong):
  pass


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


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLongPedalResume(HyundaiCanfdPedalResumeBase, TestHyundaiCanfdLFASteeringLongBase):
  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongPedalResume":
      raise unittest.SkipTest


if __name__ == "__main__":
  unittest.main()
