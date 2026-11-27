"""KOVA: the cluster set speed (CCNC_0x161 SETSPEED_SPEED) follows the openpilot set speed while openpilot
longitudinal is engaged (CAN-FD angle steering car, rlog 0000079c 2026-11-27: cluster 30 km/h, openpilot 40 km/h)."""
import random
import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.tests.test_hyundai_blindspot import STEP_NS, ioniq9
from opendbc.car.hyundai.tests.test_hyundai_hands_on import CHECKSUM_BITS, DBC_NAME, bits, random_ccnc
from opendbc.car.hyundai.values import CAR, DBC
from opendbc.car.hyundai.tests import test_hyundai

# SETSPEED_SPEED is bits 200..207
SET_SPEED_BITS = set(range(200, 208))
# cruise display fields shown from the openpilot cruise state: SETSPEED_HUD, DISTANCE_CAR, DISTANCE, SETSPEED, LFA_ICON
CRUISE_DISPLAY_BITS = set(range(181, 184)) | set(range(189, 192)) | set(range(196, 200)) | set(range(208, 212)) | set(range(224, 228))
GRAY = {"SETSPEED": 1, "SETSPEED_HUD": 1, "DISTANCE_CAR": 1, "LFA_ICON": 1}
ENGAGED = {"SETSPEED": 3, "SETSPEED_HUD": 2, "DISTANCE_CAR": 2}
STOCK_OFF = {"SETSPEED": 0, "SETSPEED_HUD": 0, "DISTANCE_CAR": 0, "DISTANCE": 0, "SETSPEED_SPEED": 255, "LFA_ICON": 0}


def ioniq9_long():
  helper = test_hyundai.TestHyundaiFingerprint
  fingerprint = helper._ioniq9_fingerprint()
  fingerprint[CanBus(None, fingerprint).ECAN][0x1BA] = 24
  CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, helper._ioniq9_fw(), True, False, False)
  return CP, CarInterface(CP), CANPacker(DBC[CP.carFingerprint][Bus.pt])


def car_control(enabled, lat_active, set_speed_kph, bars=0):
  CC = structs.CarControl()
  CC.enabled = enabled
  CC.latActive = lat_active
  CC.longActive = enabled
  CC.hudControl.setSpeed = set_speed_kph * CV.KPH_TO_MS
  CC.hudControl.leadDistanceBars = bars
  return CC.as_reader()


def run(CI, packer, stock, CC, frame=0, cruise_available=True):
  t = (frame + 1) * STEP_NS
  msgs = [CanData(*packer.make_can_msg("CCNC_0x161", CanBus(CI.CP).CAM, stock))] if stock is not None else []
  # TCS ACCEnable 0: the car allows cruise (CarState.cruiseState.available)
  msgs.append(CanData(*packer.make_can_msg("TCS", CanBus(CI.CP).ECAN, {"ACCEnable": 0 if cruise_available else 1})))
  CI.update([(t, msgs)])
  _, sends = CI.apply(CC, t)
  return sends


def signals(sends, addr, name):
  parser = CANParser(DBC_NAME, [(name, 0)], sends[0][2] if sends else 0)
  for i, s in enumerate(sends):
    if s[0] == addr:
      parser.update([(i + 1, [(s[0], s[1], s[2])])])
      return dict(parser.vl[name])
  return None


class TestCcncSetSpeed(unittest.TestCase):
  def test_long_fixture(self):
    CP, _, _ = ioniq9_long()
    self.assertTrue(CP.openpilotLongitudinalControl)

  def test_set_speed_while_long_engaged(self):
    # steering and longitudinal engaged: the copy shows the openpilot set speed, stock camera speed 30 replaced
    for set_kph, shown in ((40.0, 40), (39.6, 40), (40.4, 40), (57.0, 57), (120.0, 120)):
      _, CI, packer = ioniq9_long()
      sends = run(CI, packer, {"COUNTER": 1, "SETSPEED_SPEED": 30, "SETSPEED": 3, "DISTANCE": 4, "ALERTS_2": 1},
                  car_control(True, True, set_kph))
      copy = signals([s for s in sends if s[0] == 0x161], 0x161, "CCNC_0x161")
      self.assertIsNotNone(copy, set_kph)
      self.assertEqual(copy["SETSPEED_SPEED"], shown, set_kph)
      # icon state, gap and every other field stay stock; hands-on warning cleared while steering
      self.assertEqual((copy["SETSPEED"], copy["DISTANCE"], copy["ALERTS_2"], copy["COUNTER"]), (3, 4, 0, 1))

  def test_set_speed_matches_scc_control(self):
    # the cluster number is packed exactly like the SCC_CONTROL VSetDis sent in the same frame
    for set_kph in (30.0, 30.5, 44.49, 44.5, 99.9):
      _, CI, packer = ioniq9_long()
      sends = run(CI, packer, {"COUNTER": 1, "SETSPEED_SPEED": 10}, car_control(True, True, set_kph), frame=1)
      copy = signals([s for s in sends if s[0] == 0x161], 0x161, "CCNC_0x161")
      scc = signals([s for s in sends if s[0] == 0x1a0 and s[2] == CanBus(CI.CP).ECAN], 0x1a0, "SCC_CONTROL")
      self.assertIsNotNone(scc, set_kph)
      self.assertEqual(copy["SETSPEED_SPEED"], scc["VSetDis"], set_kph)

  def test_copy_while_long_engaged_without_steering(self):
    # longitudinal engaged while lateral pauses (standstill): copy with the set speed only, hands-on warning and
    # LFA icon stay stock
    _, CI, packer = ioniq9_long()
    sends = run(CI, packer, {"COUNTER": 2, "SETSPEED_SPEED": 30, "ALERTS_2": 2, "SOUNDS_2": 3, "LFA_ICON": 1},
                car_control(True, False, 40.0))
    copy = signals([s for s in sends if s[0] == 0x161], 0x161, "CCNC_0x161")
    self.assertIsNotNone(copy)
    self.assertEqual((copy["SETSPEED_SPEED"], copy["ALERTS_2"], copy["SOUNDS_2"], copy["LFA_ICON"]), (40, 2, 3, 1))

  def test_only_set_speed_bits_change(self):
    rng = random.Random(0x1610)
    _, CI, packer = ioniq9_long()
    for frame in range(0, 200, 5):
      stock = random_ccnc(packer, rng, bus=CanBus(CI.CP).CAM)
      stock_values = signals([stock], 0x161, "CCNC_0x161")
      stock_values.pop("CHECKSUM")
      stock_values["ALERTS_2"] = 0  # no hands-on warning in this frame
      set_kph = float(rng.randrange(30, 150))
      t = (frame + 1) * STEP_NS
      _, dat_stock, _ = packer.make_can_msg("CCNC_0x161", CanBus(CI.CP).CAM, stock_values)
      CI.update([(t, [CanData(0x161, dat_stock, CanBus(CI.CP).CAM)])])
      _, sends = CI.apply(car_control(True, False, set_kph), t)
      copies = [s for s in sends if s[0] == 0x161]
      self.assertEqual(len(copies), 1)
      before, after = bits(dat_stock), bits(copies[0][1])
      changed = {i for i in range(len(before)) if before[i] != after[i]}
      # KOVA 11/27 launch: the cruise icons now show the openpilot state too (engaged here, LFA gray without steering)
      allowed = SET_SPEED_BITS | CRUISE_DISPLAY_BITS | CHECKSUM_BITS
      self.assertTrue(changed <= allowed, sorted(changed - allowed))
      self.assertEqual(sum(after[200 + i] << i for i in range(8)), int(set_kph))
      copy = signals(copies, 0x161, "CCNC_0x161")
      shown = {k: copy[k] for k in ("SETSPEED", "SETSPEED_HUD", "DISTANCE_CAR", "LFA_ICON", "DISTANCE")}
      self.assertEqual(shown, {**ENGAGED, "LFA_ICON": 1, "DISTANCE": stock_values["DISTANCE"]})

  def test_stock_set_speed_without_long_engaged(self):
    # stock longitudinal, or openpilot longitudinal not engaged (always-on lateral only): stock set speed
    from opendbc.safety import ALTERNATIVE_EXPERIENCE
    CP, _, packer = ioniq9()
    CI = CarInterface(CP)
    sends = run(CI, packer, {"COUNTER": 1, "SETSPEED_SPEED": 30}, car_control(True, True, 40.0))
    self.assertEqual(signals([s for s in sends if s[0] == 0x161], 0x161, "CCNC_0x161")["SETSPEED_SPEED"], 30)

    # KOVA 11/27 launch: openpilot longitudinal available but not engaged (always-on lateral steering): the gray
    # openpilot cruise with its set speed, LFA green while steering (was: stock set speed 30)
    CP, _, packer = ioniq9_long()
    CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL
    CI = CarInterface(CP)
    sends = run(CI, packer, {"COUNTER": 1, "SETSPEED_SPEED": 30}, car_control(False, True, 40.0))
    copy = signals([s for s in sends if s[0] == 0x161], 0x161, "CCNC_0x161")
    self.assertEqual((copy["SETSPEED_SPEED"], copy["SETSPEED"], copy["SETSPEED_HUD"], copy["LFA_ICON"]), (40, 1, 1, 2))
    # with cruise not available: the stock set speed
    sends = run(CI, packer, {"COUNTER": 2, "SETSPEED_SPEED": 30}, car_control(False, True, 40.0), 1, cruise_available=False)
    self.assertEqual(signals([s for s in sends if s[0] == 0x161], 0x161, "CCNC_0x161")["SETSPEED_SPEED"], 30)

  def test_no_copy_when_nothing_engaged(self):
    # KOVA 11/27 launch: only with cruise not available (TCS ACCEnable); available gives the gray copy below
    _, CI, packer = ioniq9_long()
    # the first TCS frame only arms the parser
    run(CI, packer, None, car_control(False, False, 40.0), 0, cruise_available=False)
    for frame in range(5, 45, 5):
      sends = run(CI, packer, {"COUNTER": frame // 5, "SETSPEED_SPEED": 30}, car_control(False, False, 40.0), frame,
                  cruise_available=False)
      self.assertEqual([s for s in sends if s[0] == 0x161], [])

  def test_gray_cruise_while_available(self):
    # from ignition on, openpilot longitudinal available and not engaged: gray cruise and LFA icons, no set speed yet
    # (255 as the stock camera), the openpilot gap bars; warnings, sounds and every other signal stay stock
    _, CI, packer = ioniq9_long()
    stock = {**STOCK_OFF, "COUNTER": 3, "ALERTS_2": 1, "SOUNDS_2": 2, "FCA_ICON": 1, "LKA_ICON": 3}
    sends = run(CI, packer, stock, car_control(False, False, 255.0, bars=4))
    copies = [s for s in sends if s[0] == 0x161]
    self.assertEqual(len(copies), 1)
    copy = signals(copies, 0x161, "CCNC_0x161")
    shown = {k: copy[k] for k in ("SETSPEED", "SETSPEED_HUD", "DISTANCE_CAR", "LFA_ICON", "SETSPEED_SPEED", "DISTANCE")}
    self.assertEqual(shown, {**GRAY, "SETSPEED_SPEED": 255, "DISTANCE": 4})
    self.assertEqual((copy["ALERTS_2"], copy["SOUNDS_2"], copy["FCA_ICON"], copy["LKA_ICON"], copy["COUNTER"]), (1, 2, 1, 3, 3))
    # after a disengagement the last openpilot set speed stays shown, gray
    sends = run(CI, packer, {**stock, "COUNTER": 4}, car_control(False, False, 57.0), 1)
    copy = signals([s for s in sends if s[0] == 0x161], 0x161, "CCNC_0x161")
    self.assertEqual((copy["SETSPEED_SPEED"], copy["SETSPEED"], copy["DISTANCE"]), (57, 1, 0))

  def test_gray_cruise_changes_only_display_bits(self):
    # the gray copy differs from the stock frame only in the bits panda allows without control
    rng = random.Random(0x1611)
    _, CI, packer = ioniq9_long()
    for frame in range(0, 200, 5):
      stock = random_ccnc(packer, rng, bus=CanBus(CI.CP).CAM)
      stock_values = signals([stock], 0x161, "CCNC_0x161")
      stock_values.pop("CHECKSUM")
      t = (frame + 1) * STEP_NS
      _, dat_stock, _ = packer.make_can_msg("CCNC_0x161", CanBus(CI.CP).CAM, stock_values)
      CI.update([(t, [CanData(0x161, dat_stock, CanBus(CI.CP).CAM)])])
      _, sends = CI.apply(car_control(False, False, float(rng.randrange(30, 150)), bars=rng.randrange(0, 5)), t)
      copies = [s for s in sends if s[0] == 0x161]
      self.assertEqual(len(copies), 1)
      before, after = bits(dat_stock), bits(copies[0][1])
      changed = {i for i in range(len(before)) if before[i] != after[i]}
      allowed = SET_SPEED_BITS | CRUISE_DISPLAY_BITS | CHECKSUM_BITS
      self.assertTrue(changed <= allowed, sorted(changed - allowed))
      copy = signals(copies, 0x161, "CCNC_0x161")
      self.assertEqual({k: copy[k] for k in GRAY}, GRAY)

  def test_engaged_cruise_icons(self):
    # openpilot longitudinal engaged without the stock camera cruise on (no MAIN press): engaged icons and gap bars
    for lat_active, lfa in ((True, 2), (False, 1)):
      _, CI, packer = ioniq9_long()
      sends = run(CI, packer, {**STOCK_OFF, "COUNTER": 1}, car_control(True, lat_active, 8.0, bars=3))
      copy = signals([s for s in sends if s[0] == 0x161], 0x161, "CCNC_0x161")
      shown = {k: copy[k] for k in ("SETSPEED", "SETSPEED_HUD", "DISTANCE_CAR", "LFA_ICON", "DISTANCE", "SETSPEED_SPEED")}
      self.assertEqual(shown, {**ENGAGED, "LFA_ICON": lfa, "DISTANCE": 3, "SETSPEED_SPEED": 8}, lat_active)

  def test_one_copy_per_stock_frame_while_long_engaged(self):
    _, CI, packer = ioniq9_long()
    counts = []
    for frame in range(40):
      stock = {"COUNTER": frame // 5, "SETSPEED_SPEED": 30} if frame % 5 == 0 else None
      sends = run(CI, packer, stock, car_control(True, False, 40.0), frame)
      counts.append(len([s for s in sends if s[0] == 0x161]))
    self.assertEqual(counts, [1 if frame % 5 == 0 else 0 for frame in range(40)])


if __name__ == "__main__":
  unittest.main()
