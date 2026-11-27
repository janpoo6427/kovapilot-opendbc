from collections.abc import Callable

from opendbc.safety import ALTERNATIVE_EXPERIENCE
import opendbc.safety.tests.common as common
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import make_msg


class Buttons:
  NONE = 0
  RESUME = 1
  SET = 2
  CANCEL = 4


PREV_BUTTON_SAMPLES = 8
ENABLE_BUTTONS = (Buttons.RESUME, Buttons.SET, Buttons.CANCEL)


class HyundaiButtonBase(common.CarSafetyTest):
  BUTTONS_TX_BUS = 0  # tx on this bus, rx on 0
  SCC_BUS = 0  # rx on this bus

  _button_msg: Callable[..., libsafety_py.CANPacket]

  def test_button_sends(self):
    """
      Only RES and CANCEL buttons are allowed
      - RES allowed while controls allowed
      - CANCEL allowed while cruise is enabled
    """
    self.safety.set_controls_allowed(0)
    self.assertFalse(self._tx(self._button_msg(Buttons.RESUME, bus=self.BUTTONS_TX_BUS)))
    self.assertFalse(self._tx(self._button_msg(Buttons.SET, bus=self.BUTTONS_TX_BUS)))

    self.safety.set_controls_allowed(1)
    self.assertTrue(self._tx(self._button_msg(Buttons.RESUME, bus=self.BUTTONS_TX_BUS)))
    self.assertFalse(self._tx(self._button_msg(Buttons.SET, bus=self.BUTTONS_TX_BUS)))

    for enabled in (True, False):
      self._rx(self._pcm_status_msg(enabled))
      self.assertEqual(enabled, self._tx(self._button_msg(Buttons.CANCEL, bus=self.BUTTONS_TX_BUS)))

  def test_enable_control_allowed_from_cruise(self):
    """
      Hyundai non-longitudinal only enables on PCM rising edge and recent button press. Tests PCM enabling with:
      - disallowed: No buttons
      - disallowed: Buttons that don't enable cruise
      - allowed: Buttons that do enable cruise
      - allowed: Main button with all above combinations
    """
    for main_button in (0, 1):
      for btn in range(8):
        for _ in range(PREV_BUTTON_SAMPLES):  # reset
          self._rx(self._button_msg(Buttons.NONE))

        self._rx(self._pcm_status_msg(False))
        self.assertFalse(self.safety.get_controls_allowed())
        self._rx(self._button_msg(btn, main_button=main_button))
        self._rx(self._pcm_status_msg(True))
        controls_allowed = btn in ENABLE_BUTTONS or main_button
        self.assertEqual(controls_allowed, self.safety.get_controls_allowed())
        self.safety.set_controls_allowed(False)
        self._rx(self._pcm_status_msg(True))
        self.assertFalse(self.safety.get_controls_allowed())

  def test_sampling_cruise_buttons(self):
    """
      Test that we allow controls on recent button press, but not as button leaves sliding window
    """
    self._rx(self._button_msg(Buttons.SET))
    for i in range(2 * PREV_BUTTON_SAMPLES):
      self._rx(self._pcm_status_msg(False))
      self.assertFalse(self.safety.get_controls_allowed())
      self._rx(self._pcm_status_msg(True))
      controls_allowed = i < PREV_BUTTON_SAMPLES
      self.assertEqual(controls_allowed, self.safety.get_controls_allowed())
      self._rx(self._button_msg(Buttons.NONE))


class HyundaiAlwaysOnLateralBase(common.CarSafetyTest):
  """
    ALT_EXP_ALWAYS_ON_LATERAL: after a driver engagement (cruise, or the LFA button where the
    safety mode reads it), lateral commands stay allowed without cruise while the car moves
    and the RX checks are valid, also while the driver brakes (the brake still exits controls).
  """
  LFA_BUTTON = False  # the safety mode reads the LFA button as an engagement

  _button_msg: Callable[..., libsafety_py.CANPacket]

  def _lateral_cmd_allowed(self) -> bool:
    raise NotImplementedError

  def _stale_counter_msg(self):
    return self._button_msg(Buttons.NONE)

  def _moving_msg(self, moving):
    return self._speed_msg(self.STANDSTILL_THRESHOLD + 1 if moving else 0)

  def _cruise_engage(self):
    self._rx(self._button_msg(Buttons.SET))
    self._rx(self._pcm_status_msg(True))

  def _cruise_disengage(self):
    self._rx(self._pcm_status_msg(False))

  def _always_on(self, enabled=True):
    self.safety.set_alternative_experience(ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL if enabled else ALTERNATIVE_EXPERIENCE.DEFAULT)
    self._reset_safety_hooks()
    self._after_safety_reset()
    self._rx(self._moving_msg(True))

  def _after_safety_reset(self):
    pass

  def _engage_without_cruise(self):
    self._cruise_engage()
    self.assertTrue(self.safety.get_controls_allowed())
    self._cruise_disengage()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_always_on_lateral_off_needs_controls_allowed(self):
    self._always_on(False)
    self._engage_without_cruise()
    self.assertFalse(self.safety.get_lateral_engaged())
    self.assertFalse(self._lateral_cmd_allowed())
    self._cruise_engage()
    self.assertTrue(self._lateral_cmd_allowed())

  def test_always_on_lateral_needs_driver_engagement(self):
    self._always_on()
    self.assertFalse(self.safety.get_lateral_engaged())
    self.assertFalse(self._lateral_cmd_allowed())
    # setting the flag only is not an engagement
    self.safety.set_controls_allowed(True)
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._lateral_cmd_allowed())

  def test_always_on_lateral_outlives_cruise(self):
    self._always_on()
    self._engage_without_cruise()
    self.assertTrue(self.safety.get_lateral_engaged())
    self.assertTrue(self._lateral_cmd_allowed())
    # a new cruise engagement keeps it engaged
    self._cruise_engage()
    self.assertTrue(self._lateral_cmd_allowed())
    self._cruise_disengage()
    self.assertTrue(self._lateral_cmd_allowed())

  def test_always_on_lateral_stays_while_braking(self):
    self._always_on()
    self._engage_without_cruise()
    self._rx(self._user_brake_msg(True))
    self.assertTrue(self._lateral_cmd_allowed())
    self.assertTrue(self.safety.get_lateral_engaged())
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._user_brake_msg(False))
    self.assertTrue(self._lateral_cmd_allowed())

  def test_always_on_lateral_brake_exits_cruise_keeps_lateral(self):
    self._always_on()
    self._cruise_engage()
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_longitudinal_allowed())
    self.assertTrue(self._lateral_cmd_allowed())
    # braking to a stop still blocks lateral at standstill
    self._rx(self._moving_msg(False))
    self.assertFalse(self._lateral_cmd_allowed())
    self._rx(self._moving_msg(True))
    self.assertTrue(self._lateral_cmd_allowed())

  def test_always_on_lateral_off_blocked_while_braking(self):
    self._always_on(False)
    self._cruise_engage()
    self.assertTrue(self._lateral_cmd_allowed())
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self._lateral_cmd_allowed())
    self._rx(self._user_brake_msg(False))
    self.assertFalse(self._lateral_cmd_allowed())

  def test_always_on_lateral_blocked_at_standstill(self):
    self._always_on()
    self._engage_without_cruise()
    self._rx(self._moving_msg(False))
    self.assertFalse(self._lateral_cmd_allowed())
    self._rx(self._moving_msg(True))
    self.assertTrue(self._lateral_cmd_allowed())

  def test_always_on_lateral_rx_lag_disengages(self):
    self._always_on()
    self._engage_without_cruise()
    # no RX check was received: the tick marks them invalid and lateral needs a new engagement
    self.safety.set_timer(int(2e6))
    self.safety.safety_tick()
    self.assertFalse(self.safety.get_lateral_engaged())
    self.assertFalse(self._lateral_cmd_allowed())
    self.safety.set_lateral_engaged(True)
    self.assertFalse(self._lateral_cmd_allowed())  # RX checks are still invalid

  def test_always_on_lateral_invalid_rx_disengages(self):
    self._always_on()
    self._engage_without_cruise()
    for _ in range(10):  # repeated counters invalidate the message
      self._rx(self._stale_counter_msg())
    self.assertFalse(self.safety.get_lateral_engaged())
    self.assertFalse(self._lateral_cmd_allowed())

  def test_always_on_lateral_safety_mode_reset(self):
    self._always_on()
    self._engage_without_cruise()
    self._reset_safety_hooks()
    self._after_safety_reset()
    self._rx(self._moving_msg(True))
    self.assertFalse(self.safety.get_lateral_engaged())
    self.assertFalse(self._lateral_cmd_allowed())

  def test_always_on_lateral_lfa_button(self):
    for always_on in (False, True):
      with self.subTest(always_on=always_on):
        self._always_on(always_on)
        self._rx(self._button_msg(Buttons.NONE, lfa_button=1) if self.LFA_BUTTON else self._button_msg(Buttons.NONE))
        engaged = always_on and self.LFA_BUTTON
        self.assertEqual(engaged, self.safety.get_lateral_engaged())
        self.assertFalse(self.safety.get_controls_allowed())
        self.assertEqual(engaged, self._lateral_cmd_allowed())


class HyundaiLongitudinalBase(common.LongitudinalAccelSafetyTest):

  DISABLED_ECU_UDS_MSG: tuple[int, int]
  DISABLED_ECU_ACTUATION_MSG: tuple[int, int]

  _button_msg: Callable[..., libsafety_py.CANPacket]

  # override these tests from CarSafetyTest, hyundai longitudinal uses button enable
  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_sampling_cruise_buttons(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def test_button_sends(self):
    pass

  def _pcm_status_msg(self, enable):
    raise Exception

  def _cruise_engage(self):
    self._rx(self._button_msg(Buttons.SET))
    self._rx(self._button_msg(Buttons.NONE))

  def _cruise_disengage(self):
    self._rx(self._button_msg(Buttons.CANCEL))
    self._rx(self._button_msg(Buttons.NONE))

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    raise NotImplementedError

  def test_set_resume_buttons(self):
    """
      SET and RESUME enter controls allowed on their falling edge.
    """
    for btn_prev in range(8):
      for btn_cur in range(8):
        self._rx(self._button_msg(Buttons.NONE))
        self.safety.set_controls_allowed(0)
        for _ in range(10):
          self._rx(self._button_msg(btn_prev))
          self.assertFalse(self.safety.get_controls_allowed())

        # should enter controls allowed on falling edge and not transitioning to cancel
        should_enable = btn_cur != btn_prev and \
                        btn_cur != Buttons.CANCEL and \
                        btn_prev in (Buttons.RESUME, Buttons.SET)

        self._rx(self._button_msg(btn_cur))
        self.assertEqual(should_enable, self.safety.get_controls_allowed())

  def test_cancel_button(self):
    self.safety.set_controls_allowed(1)
    self._rx(self._button_msg(Buttons.CANCEL))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_tester_present_allowed(self, ecu_disable: bool = True):
    """
      Ensure tester present diagnostic message is allowed to keep ECU knocked out
      for longitudinal control.
    """

    addr, bus = self.DISABLED_ECU_UDS_MSG
    for should_tx, msg in ((True, b"\x02\x3E\x80\x00\x00\x00\x00\x00"),
                           (False, b"\x03\xAA\xAA\x00\x00\x00\x00\x00")):
      tester_present = libsafety_py.make_CANPacket(addr, bus, msg)
      self.assertEqual(should_tx and ecu_disable, self._tx(tester_present))

  def test_disabled_ecu_alive(self):
    """
      If the ECU knockout failed, make sure the relay malfunction is shown
    """

    addr, bus = self.DISABLED_ECU_ACTUATION_MSG
    self.assertFalse(self.safety.get_relay_malfunction())
    self._rx(make_msg(bus, addr, 8))
    self.assertTrue(self.safety.get_relay_malfunction())
