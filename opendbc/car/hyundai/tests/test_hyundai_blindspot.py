"""KOVA: turn-signal lamps and blind-spot freshness in Hyundai CarState (auto lane change inputs)."""
import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai.carstate import BLINDSPOT_FRESH_NS
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.tests import test_hyundai
from opendbc.car.hyundai.values import CAR, DBC, HyundaiFlags

STEP_NS = 10_000_000


def ioniq9(bsm=True):
  helper = test_hyundai.TestHyundaiFingerprint
  fingerprint = helper._ioniq9_fingerprint()
  if bsm:
    fingerprint[CanBus(None, fingerprint).ECAN][0x1BA] = 24
  CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, helper._ioniq9_fw(), False, False, False)
  return CP, CarInterface(CP), CANPacker(DBC[CP.carFingerprint][Bus.pt])


def seltos(bsm=True):
  fingerprint = gen_empty_fingerprint()
  if bsm:
    fingerprint[0][0x58B] = 8
  CP = CarInterface.get_params(CAR.KIA_SELTOS, fingerprint, [], False, False, False)
  return CP, CarInterface(CP), CANPacker(DBC[CP.carFingerprint][Bus.pt])


class TestBlinkerLamps(unittest.TestCase):
  def run_frames(self, CI, packer, values, frames=5):
    CS = None
    for i in range(frames):
      msg = packer.make_can_msg("BLINKERS", CanBus(CI.CP).ECAN, values)
      CS = CI.update([(i * STEP_NS, [CanData(*msg)])])
    return CS

  def test_ioniq9_alt_lamp(self):
    # IONIQ 9 drive 2026-11-27: USE_ALT_LAMP 1, only RIGHT_LAMP_ALT flashed during a right signal
    _, CI, packer = ioniq9()
    CS = self.run_frames(CI, packer, {"USE_ALT_LAMP": 1, "RIGHT_LAMP_ALT": 1})
    self.assertEqual((CS.leftBlinker, CS.rightBlinker), (False, True))
    _, CI, packer = ioniq9()
    CS = self.run_frames(CI, packer, {"USE_ALT_LAMP": 1, "LEFT_LAMP_ALT": 1})
    self.assertEqual((CS.leftBlinker, CS.rightBlinker), (True, False))

  def test_alt_lamp_ignored_without_use_alt(self):
    _, CI, packer = ioniq9()
    CS = self.run_frames(CI, packer, {"USE_ALT_LAMP": 0, "RIGHT_LAMP_ALT": 1})
    self.assertEqual((CS.leftBlinker, CS.rightBlinker), (False, False))
    _, CI, packer = ioniq9()
    CS = self.run_frames(CI, packer, {"USE_ALT_LAMP": 0, "LEFT_LAMP": 1})
    self.assertEqual((CS.leftBlinker, CS.rightBlinker), (True, False))


class TestBlindspotFresh(unittest.TestCase):
  def test_ioniq9_camera_bus(self):
    CP, CI, packer = ioniq9()
    self.assertTrue(CP.flags & HyundaiFlags.HAS_BSM)
    cam = CanBus(CP).CAM
    t = STEP_NS
    CS = None
    for _ in range(10):  # 0x1BA only on the camera bus, as on the IONIQ 9 camera-SCC harness
      CS = CI.update([(t, [CanData(*packer.make_can_msg("ADAS_CMD_50_50ms", cam, {"BCW_LtIndSta": 1}))])])
      last = t
      t += 5 * STEP_NS
    self.assertTrue(CS.leftBlindspot)
    self.assertFalse(CS.rightBlindspot)
    self.assertTrue(CS.blindspotFresh)
    # frames stop: the last value stays, but it is no longer fresh
    other = CanData(*packer.make_can_msg("BLINKERS", CanBus(CP).ECAN, {}))
    while t <= last + BLINDSPOT_FRESH_NS:
      CS = CI.update([(t, [other])])
      self.assertTrue(CS.blindspotFresh)
      t += STEP_NS
    CS = CI.update([(t, [other])])
    self.assertTrue(CS.leftBlindspot)
    self.assertFalse(CS.blindspotFresh)

  def test_ioniq9_newest_bus_wins(self):
    CP, CI, packer = ioniq9()
    CS = CI.update([(STEP_NS, [CanData(*packer.make_can_msg("ADAS_CMD_50_50ms", CanBus(CP).CAM, {"BCW_RtIndSta": 1}))])])
    self.assertTrue(CS.rightBlindspot)
    CS = CI.update([(2 * STEP_NS, [CanData(*packer.make_can_msg("ADAS_CMD_50_50ms", CanBus(CP).ECAN, {"BCW_RtIndSta": 0}))])])
    self.assertFalse(CS.rightBlindspot)
    self.assertTrue(CS.blindspotFresh)

  def test_no_bsm_never_fresh(self):
    CP, CI, packer = ioniq9(bsm=False)
    self.assertFalse(CP.flags & HyundaiFlags.HAS_BSM)
    CS = CI.update([(STEP_NS, [CanData(*packer.make_can_msg("ADAS_CMD_50_50ms", CanBus(CP).CAM, {"BCW_LtIndSta": 1}))])])
    self.assertFalse(CS.blindspotFresh)
    self.assertFalse(CS.leftBlindspot)

  def test_can_lca11(self):
    CP, CI, packer = seltos()
    self.assertTrue(CP.flags & HyundaiFlags.HAS_BSM)
    t = STEP_NS
    CS = None
    for _ in range(10):
      CS = CI.update([(t, [CanData(*packer.make_can_msg("LCA11", 0, {"CF_Lca_IndRight": 1}))])])
      last = t
      t += 2 * STEP_NS
    self.assertTrue(CS.rightBlindspot)
    self.assertTrue(CS.blindspotFresh)
    other = CanData(*packer.make_can_msg("CLU11", 0, {}))
    while t <= last + BLINDSPOT_FRESH_NS:
      CS = CI.update([(t, [other])])
      self.assertTrue(CS.blindspotFresh)
      t += STEP_NS
    CS = CI.update([(t, [other])])
    self.assertFalse(CS.blindspotFresh)
    CP, CI, packer = seltos(bsm=False)
    CS = CI.update([(STEP_NS, [CanData(*packer.make_can_msg("LCA11", 0, {"CF_Lca_IndRight": 1}))])])
    self.assertFalse(CS.blindspotFresh)
    self.assertFalse(CS.rightBlindspot)


if __name__ == "__main__":
  unittest.main()
