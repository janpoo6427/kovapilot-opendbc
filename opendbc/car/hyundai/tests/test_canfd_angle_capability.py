import copy
import unittest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import CarParams
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, HyundaiFlags, HyundaiSafetyFlags

# Messages a CAN-FD angle steering car sends (role, address, length), written out here so the
# test states the expected capability independently of the profile in values.py.
ANGLE_MESSAGES = (
  ("CAM", 0xCB, 24), ("CAM", 0x1A0, 32),
  ("ECAN", 0x35, 32), ("ECAN", 0xA0, 24), ("ECAN", 0x125, 16), ("ECAN", 0x175, 24), ("ECAN", 0x1CF, 8), ("ECAN", 0xEA, 24),
)
LONGITUDINAL_MESSAGES = (
  ("CAM", 0x1A0, 32), ("CAM", 0x12A, 16),
  ("ECAN", 0x35, 32), ("ECAN", 0xA0, 24), ("ECAN", 0x175, 24), ("ECAN", 0x1CF, 8),
)
ANGLE_SAFETY_BITS = HyundaiSafetyFlags.CANFD_ANGLE_STEERING | HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.EV_GAS


def angle_fingerprint(messages=ANGLE_MESSAGES + LONGITUDINAL_MESSAGES):
  fingerprint = gen_empty_fingerprint()
  CAN = CanBus(None, fingerprint)
  for role, address, dlc in messages:
    fingerprint[getattr(CAN, role)][address] = dlc
  return fingerprint


def canfd_ev_platforms():
  return [p for p in CAR if p.config.flags & HyundaiFlags.CANFD and p.config.flags & HyundaiFlags.EV]


class TestCanFDAngleCapability(unittest.TestCase):
  def assert_angle(self, CP, long_requested):
    self.assertTrue(CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING)
    self.assertTrue(CP.flags & HyundaiFlags.CANFD_LONGITUDINAL_CANDIDATE)
    self.assertEqual(CP.steerControlType, CarParams.SteerControlType.angle)
    self.assertFalse(CP.dashcamOnly)
    self.assertEqual(len(CP.safetyConfigs), 1)
    self.assertEqual(CP.safetyConfigs[0].safetyModel, CarParams.SafetyModel.hyundaiCanfd)
    param = CP.safetyConfigs[0].safetyParam
    self.assertEqual(param & ANGLE_SAFETY_BITS, ANGLE_SAFETY_BITS)
    self.assertEqual(bool(param & HyundaiSafetyFlags.LONG), long_requested)
    self.assertTrue(CP.alphaLongitudinalAvailable)
    self.assertEqual(CP.openpilotLongitudinalControl, long_requested)
    self.assertEqual(CP.pcmCruise, not long_requested)

  def test_angle_capability_follows_messages_on_every_canfd_ev_platform(self):
    # Any CAN-FD EV that sends the angle steering messages gets angle steering, with no firmware list.
    platforms = canfd_ev_platforms()
    self.assertGreater(len(platforms), 5)
    for platform in platforms:
      for long_requested in (False, True):
        with self.subTest(platform=platform, long=long_requested):
          CP = CarInterface.get_params(platform, angle_fingerprint(), [], long_requested, False, False)
          self.assert_angle(CP, long_requested)

  def test_angle_capability_does_not_need_known_firmware(self):
    unknown_fw = [
      CarParams.CarFw(ecu=CarParams.Ecu.fwdRadar, address=0x7D0, subAddress=0, fwVersion=b'\xf1\x00unknown radar'),
      CarParams.CarFw(ecu=CarParams.Ecu.fwdCamera, address=0x7C4, subAddress=0, fwVersion=b'\xf1\x00unknown camera'),
      CarParams.CarFw(ecu=CarParams.Ecu.adas, address=0x730, subAddress=0, fwVersion=b'\xf1\x00unknown adas'),
    ]
    for car_fw in ([], unknown_fw):
      for long_requested in (False, True):
        with self.subTest(fw=len(car_fw), long=long_requested):
          CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, angle_fingerprint(), car_fw, long_requested, False, False)
          self.assert_angle(CP, long_requested)
          self.assertEqual(CP.safetyConfigs[0].safetyParam, 1037 if long_requested else 1033)

  def test_same_messages_same_result_on_every_platform(self):
    # The platform name must not change the angle steering / longitudinal decision or the panda safety parameter.
    reference = {}
    for long_requested in (False, True):
      CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, angle_fingerprint(), [], long_requested, False, False)
      reference[long_requested] = (CP.flags & (HyundaiFlags.CANFD_ANGLE_STEERING | HyundaiFlags.CANFD_LONGITUDINAL_CANDIDATE),
                                   CP.safetyConfigs[0].safetyParam & (ANGLE_SAFETY_BITS | HyundaiSafetyFlags.LONG),
                                   CP.openpilotLongitudinalControl, CP.alphaLongitudinalAvailable)
    for platform in canfd_ev_platforms():
      for long_requested in (False, True):
        with self.subTest(platform=platform, long=long_requested):
          CP = CarInterface.get_params(platform, angle_fingerprint(), [], long_requested, False, False)
          got = (CP.flags & (HyundaiFlags.CANFD_ANGLE_STEERING | HyundaiFlags.CANFD_LONGITUDINAL_CANDIDATE),
                 CP.safetyConfigs[0].safetyParam & (ANGLE_SAFETY_BITS | HyundaiSafetyFlags.LONG),
                 CP.openpilotLongitudinalControl, CP.alphaLongitudinalAvailable)
          self.assertEqual(got, reference[long_requested])

  def test_lfa_torque_messages_without_angle_command_stay_torque(self):
    # A CAN-FD EV with LFA (torque) steering sends everything above except the camera angle command 0xCB
    # (the 2022-09-10 KIA_EV6 LFA test route, for example). It must keep torque steering.
    messages = tuple(m for m in ANGLE_MESSAGES + LONGITUDINAL_MESSAGES if m != ("CAM", 0xCB, 24))
    for platform in canfd_ev_platforms():
      with self.subTest(platform=platform):
        CP = CarInterface.get_params(platform, angle_fingerprint(messages), [], True, False, False)
        self.assertFalse(CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING)
        self.assertFalse(CP.flags & HyundaiFlags.CANFD_LONGITUDINAL_CANDIDATE)
        self.assertEqual(CP.steerControlType, CarParams.SteerControlType.torque)
        self.assertFalse(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_ANGLE_STEERING)

  def test_lka_steering_cars_never_select_angle(self):
    # panda angle steering refuses LKA steering message cars, so the angle flag must not be set there.
    fingerprint = angle_fingerprint()
    fingerprint[CanBus(None, fingerprint).CAM][0x50] = 16
    for platform in canfd_ev_platforms():
      with self.subTest(platform=platform):
        CP = CarInterface.get_params(platform, copy.deepcopy(fingerprint), [], False, False, False)
        self.assertFalse(CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING)
        self.assertFalse(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_ANGLE_STEERING)


if __name__ == "__main__":
  unittest.main()
