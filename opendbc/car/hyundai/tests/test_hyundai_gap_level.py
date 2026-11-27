"""KOVA: CarState.pcmCruiseGap is the car\x27s own cruise gap level from the stock SCC frame. CAN-FD camera SCC,
rlog 0000079c segment 49 (2026-11-27): each gap button press stepped the camera SCC_CONTROL DISTANCE_SETTING
4 -> 3 -> 2 -> 1 -> 4 (a four level car), also while openpilot longitudinal was engaged."""
import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.tests.test_hyundai_blindspot import STEP_NS, ioniq9
from opendbc.car.hyundai.tests.test_hyundai_cluster_speed import ioniq9_long
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags


def run(CI, packer, bus, distance, frames=3):
  CS = None
  for i in range(frames):
    msg = packer.make_can_msg("SCC_CONTROL", bus, {"COUNTER": i, "MainMode_ACC": 1, "DISTANCE_SETTING": distance})
    CS = CI.update([((i + 1) * STEP_NS, [CanData(*msg)])])
  return CS


class TestCruiseGapLevel(unittest.TestCase):
  def test_camera_scc_longitudinal(self):
    # openpilot longitudinal on a camera SCC car: the stock camera frame on the camera bus gives the level
    for level in (0, 1, 2, 3, 4):
      CP, CI, packer = ioniq9_long()
      self.assertTrue(CP.openpilotLongitudinalControl)
      self.assertTrue(CP.flags & HyundaiFlags.CANFD_LONGITUDINAL_CANDIDATE)
      CS = run(CI, packer, CanBus(CP).CAM, level)
      self.assertEqual(CS.pcmCruiseGap, level)

  def test_camera_scc_longitudinal_ignores_ecan(self):
    # an SCC_CONTROL on the ECAN is not the stock camera frame
    CP, CI, packer = ioniq9_long()
    CS = run(CI, packer, CanBus(CP).ECAN, 4)
    self.assertEqual(CS.pcmCruiseGap, 0)

  def test_stock_longitudinal(self):
    for level in (1, 3, 4):
      CP, CI, packer = ioniq9()
      self.assertFalse(CP.openpilotLongitudinalControl)
      CS = run(CI, packer, CanBus(CP).CAM, level)
      self.assertEqual(CS.pcmCruiseGap, level)

  def test_no_new_message_for_other_longitudinal(self):
    # openpilot longitudinal without a stock SCC frame template: nothing is read and no message is added to the
    # parsers (CAN validity unchanged), the level stays unknown
    fingerprint = gen_empty_fingerprint()
    CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_5, fingerprint, [], True, False, False)
    self.assertTrue(CP.flags & HyundaiFlags.CANFD)
    if not CP.openpilotLongitudinalControl or CP.flags & HyundaiFlags.CANFD_LONGITUDINAL_CANDIDATE:
      self.skipTest("fixture is not openpilot longitudinal without the stock frame template")
    CI = CarInterface(CP)
    packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])
    CS = run(CI, packer, CanBus(CP).ECAN, 4)
    self.assertEqual(CS.pcmCruiseGap, 0)
    for cp in CI.can_parsers.values():
      self.assertNotIn(0x1A0, cp.addresses)


if __name__ == "__main__":
  unittest.main()
