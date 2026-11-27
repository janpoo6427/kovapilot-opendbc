"""KOVA: remove the stock camera hands-on warning (CCNC_0x161 ALERTS_2 1/2) while openpilot steers (CAN-FD angle)."""
import random
import unittest

from opendbc.can import CANDefine, CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.carcontroller import CCNC_COPY_MAX_AGE_NS
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.tests.test_hyundai_blindspot import STEP_NS, ioniq9, seltos

DBC_NAME = "hyundai_canfd_generated"
# ALERTS_2 is bits 134..138, SOUNDS_2 bits 164..167
EDITED_BITS = set(range(134, 139)) | set(range(164, 168))
# LFA_ICON is bits 224..227
LFA_ICON_BITS = set(range(224, 228))
CHECKSUM_BITS = set(range(16))


def bits(dat):
  return [(dat[i // 8] >> (i % 8)) & 1 for i in range(len(dat) * 8)]


def random_ccnc(packer, rng, bus=2, **fixed):
  sigs = packer.dbc.name_to_msg["CCNC_0x161"].sigs
  values = {}
  for name, sig in sigs.items():
    if name == "CHECKSUM":
      continue
    raw = rng.randrange(1 << sig.size)
    if sig.is_signed:
      raw -= (raw >> (sig.size - 1)) * (1 << sig.size)
    values[name] = raw * sig.factor + sig.offset
  values.update(fixed)
  return packer.make_can_msg("CCNC_0x161", bus, values)


def copy_of(packer, dat, lateral_active=False):
  parser = CANParser(DBC_NAME, [("CCNC_0x161", 0)], 2)
  parser.update([(1, [(0x161, dat, 2)])])
  return hyundaicanfd.create_ccnc_hands_on_cleared(packer, CanBus(None, gen_empty_fingerprint()), parser.vl["CCNC_0x161"],
                                                   lateral_active)


class TestCcncCopy(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker(DBC_NAME)
    self.rng = random.Random(0x161)

  def test_hands_on_signals(self):
    values = CANDefine(DBC_NAME).dv["CCNC_0x161"]["ALERTS_2"]
    self.assertEqual({v: values[v] for v in hyundaicanfd.CCNC_HANDS_ON_ALERTS},
                     {1: "KEEP_HANDS_ON_STEERING_WHEEL", 2: "KEEP_HANDS_ON_STEERING_WHEEL_RED"})
    for name, start in (("ALERTS_2", 134), ("SOUNDS_2", 164)):
      sig = self.packer.dbc.name_to_msg["CCNC_0x161"].sigs[name]
      self.assertEqual((sig.start_bit, sig.size), (start, 5 if name == "ALERTS_2" else 4))

  def test_other_frames_copied_byte_for_byte(self):
    for _ in range(500):
      alerts_2 = self.rng.choice([0, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 21, 22, 31])
      addr, dat, _ = random_ccnc(self.packer, self.rng, ALERTS_2=alerts_2)
      out_addr, out, bus = copy_of(self.packer, dat)
      self.assertEqual((out_addr, bus), (addr, CanBus(None, gen_empty_fingerprint()).ECAN))
      self.assertEqual(out, dat)

  def test_hands_on_warning_and_chime_removed_only(self):
    for alerts_2 in hyundaicanfd.CCNC_HANDS_ON_ALERTS:
      for _ in range(200):
        _, dat, _ = random_ccnc(self.packer, self.rng, ALERTS_2=alerts_2)
        _, out, _ = copy_of(self.packer, dat)
        before, after = bits(dat), bits(out)
        changed = {i for i in range(len(before)) if before[i] != after[i]}
        self.assertTrue(changed <= (EDITED_BITS | CHECKSUM_BITS), changed - EDITED_BITS - CHECKSUM_BITS)
        self.assertEqual(out[2], dat[2])  # stock COUNTER kept
        self.assertEqual([after[i] for i in sorted(EDITED_BITS)], [0] * len(EDITED_BITS))

  def test_lfa_icon_signal(self):
    values = CANDefine(DBC_NAME).dv["CCNC_0x161"]["LFA_ICON"]
    self.assertEqual((values[1], values[hyundaicanfd.CCNC_LFA_ICON_ACTIVE]), ("GRAY", "GREEN"))
    sig = self.packer.dbc.name_to_msg["CCNC_0x161"].sigs["LFA_ICON"]
    self.assertEqual((sig.start_bit, sig.size), (224, 4))

  def test_lfa_icon_green_while_steering(self):
    for _ in range(500):
      alerts_2 = self.rng.choice([0, 1, 2, 3, 4, 5, 6, 10, 21, 22, 31])
      lfa_icon = self.rng.choice([0, 1, 2, 3])
      _, dat, _ = random_ccnc(self.packer, self.rng, ALERTS_2=alerts_2, LFA_ICON=lfa_icon)
      _, out, _ = copy_of(self.packer, dat, lateral_active=True)
      before, after = bits(dat), bits(out)
      changed = {i for i in range(len(before)) if before[i] != after[i]}
      allowed = LFA_ICON_BITS | CHECKSUM_BITS | (EDITED_BITS if alerts_2 in hyundaicanfd.CCNC_HANDS_ON_ALERTS else set())
      self.assertTrue(changed <= allowed, changed - allowed)
      self.assertEqual(out[2], dat[2])  # stock COUNTER kept
      self.assertEqual(sum(after[224 + i] << i for i in range(4)), hyundaicanfd.CCNC_LFA_ICON_ACTIVE)


def car_control(lat_active):
  CC = structs.CarControl()
  CC.enabled = lat_active
  CC.latActive = lat_active
  return CC.as_reader()


class TestCcncController(unittest.TestCase):
  def step(self, CI, packer, frame, lat_active, ccnc=None):
    msgs = []
    if ccnc is not None:
      msgs.append(CanData(*packer.make_can_msg("CCNC_0x161", CanBus(CI.CP).CAM, ccnc)))
    t = (frame + 1) * STEP_NS
    CI.update([(t, msgs)])
    _, sends = CI.apply(car_control(lat_active), t)
    return [s for s in sends if s[0] == 0x161]

  def test_ioniq9_copy_while_steering(self):
    _, CI, packer = ioniq9()
    sent = []
    for frame in range(40):
      ccnc = {"COUNTER": frame // 5, "ALERTS_2": 2, "SOUNDS_2": 3, "LKA_ICON": 4} if frame % 5 == 0 else None
      sent.append(self.step(CI, packer, frame, True, ccnc))
    copies = [s for frame_sends in sent for s in frame_sends]
    # one copy per new stock frame, on the ECAN, sent in the same frame the stock frame was read
    self.assertEqual([len(s) for s in sent], [1 if frame % 5 == 0 else 0 for frame in range(40)])
    self.assertTrue(all(s[2] == CanBus(CI.CP).ECAN for s in copies))
    parser = CANParser(DBC_NAME, [("CCNC_0x161", 0)], CanBus(CI.CP).ECAN)
    for i, s in enumerate(copies):
      parser.update([(i + 1, [(s[0], s[1], s[2])])])
      self.assertEqual((parser.vl["CCNC_0x161"]["COUNTER"], parser.vl["CCNC_0x161"]["ALERTS_2"],
                        parser.vl["CCNC_0x161"]["SOUNDS_2"], parser.vl["CCNC_0x161"]["LKA_ICON"]), (i, 0, 0, 4))

  def test_ioniq9_lfa_icon_green_while_steering(self):
    # stock camera sends LFA_ICON 1 (gray) while openpilot steers (IONIQ 9 rlog 0000079c); the copy shows it green,
    # also with always-on lateral (latActive without enabled)
    from opendbc.safety import ALTERNATIVE_EXPERIENCE
    for always_on, enabled, shown in ((False, True, True), (True, True, True), (True, False, True), (False, False, False)):
      CP, _, packer = ioniq9()
      if always_on:
        CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL
      CI = CarInterface(CP)
      CC = structs.CarControl()
      CC.enabled = enabled
      CC.latActive = True
      msg = CanData(*packer.make_can_msg("CCNC_0x161", CanBus(CP).CAM, {"COUNTER": 1, "LFA_ICON": 1, "LKA_ICON": 4}))
      CI.update([(STEP_NS, [msg])])
      _, sends = CI.apply(CC.as_reader(), STEP_NS)
      copies = [s for s in sends if s[0] == 0x161]
      self.assertEqual(len(copies), 1)
      parser = CANParser(DBC_NAME, [("CCNC_0x161", 0)], CanBus(CP).ECAN)
      parser.update([(1, [copies[0]])])
      self.assertEqual((parser.vl["CCNC_0x161"]["LFA_ICON"], parser.vl["CCNC_0x161"]["LKA_ICON"]),
                       (hyundaicanfd.CCNC_LFA_ICON_ACTIVE if shown else 1, 4), (always_on, enabled))

  def test_ioniq9_no_copy_when_not_steering(self):
    _, CI, packer = ioniq9()
    for frame in range(40):
      ccnc = {"COUNTER": frame // 5, "ALERTS_2": 1} if frame % 5 == 0 else None
      self.assertEqual(self.step(CI, packer, frame, False, ccnc), [])

  def test_ioniq9_no_stale_copy_on_engage(self):
    # a stock frame read while not steering is never sent once steering starts
    _, CI, packer = ioniq9()
    self.assertEqual(self.step(CI, packer, 0, False, {"COUNTER": 1, "ALERTS_2": 1}), [])
    for frame in range(1, 5):
      self.assertEqual(self.step(CI, packer, frame, True), [])
    self.assertEqual(len(self.step(CI, packer, 5, True, {"COUNTER": 2, "ALERTS_2": 1})), 1)

  def test_ioniq9_corrupt_stock_frame_not_copied(self):
    _, CI, packer = ioniq9()
    addr, dat, bus = packer.make_can_msg("CCNC_0x161", CanBus(CI.CP).CAM, {"COUNTER": 1, "ALERTS_2": 1})
    bad = bytes([dat[0] ^ 1]) + dat[1:]
    CI.update([(STEP_NS, [CanData(addr, bad, bus)])])
    _, sends = CI.apply(car_control(True), STEP_NS)
    self.assertEqual([s for s in sends if s[0] == 0x161], [])

  def test_ioniq9_late_frame_not_copied(self):
    # a stock frame read more than 20 ms before the controller runs would reach panda after the next stock frame
    _, CI, packer = ioniq9()
    msg = CanData(*packer.make_can_msg("CCNC_0x161", CanBus(CI.CP).CAM, {"COUNTER": 1, "ALERTS_2": 1}))
    CI.update([(STEP_NS, [msg])])
    _, sends = CI.apply(car_control(True), STEP_NS + CCNC_COPY_MAX_AGE_NS + 1)
    self.assertEqual([s for s in sends if s[0] == 0x161], [])
    msg = CanData(*packer.make_can_msg("CCNC_0x161", CanBus(CI.CP).CAM, {"COUNTER": 2, "ALERTS_2": 1}))
    CI.update([(3 * STEP_NS, [msg])])
    _, sends = CI.apply(car_control(True), 3 * STEP_NS + CCNC_COPY_MAX_AGE_NS)
    self.assertEqual(len([s for s in sends if s[0] == 0x161]), 1)

  def test_ccnc_never_affects_can_valid(self):
    _, CI, packer = ioniq9()
    cam = CI.can_parsers[Bus.cam]
    state = cam.message_states[0x161]
    self.assertTrue(state.ignore_alive and state.ignore_counter)

  def test_can_car_never_sends_ccnc(self):
    CP, CI, packer = seltos()
    for frame in range(20):
      CI.update([(frame * STEP_NS, [])])
      _, sends = CI.apply(car_control(True), frame * STEP_NS)
      self.assertEqual([s for s in sends if s[0] == 0x161], [])


if __name__ == "__main__":
  unittest.main()
