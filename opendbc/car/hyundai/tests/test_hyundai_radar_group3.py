"""KOVA: CAN-FD front radar object list ("group 3", 0x400-0x41D, 24 bytes, A-CAN, 20 Hz).

Used automatically when the fingerprint has every slot with length 24 (no user setting, janpoo 2026-11-28 15:41),
otherwise the car stays as before (model-only leads). Rlogs: Ioniq 9 2026-11-27/28.

KOVA: 그룹3 전방 레이더 물체 목록. 지문에 30개 자리가 모두 길이 24로 있으면 자동으로 쓰고, 없으면 지금처럼(모델만).
사용자 설정 없음(janpoo 11/28 15:41)."""
import copy
import unittest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint
from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.radar_interface import (GROUP3_DBC, GROUP3_MSG_COUNT, GROUP3_MSG_LEN, GROUP3_REAR_OFFSET,
                                                 GROUP3_START_ADDR, GROUP3_STARTUP_GRACE_S, RadarInterface)
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from opendbc.car.hyundai.tests import test_hyundai

CYCLE_NS = 50_000_000  # 20 Hz radar cycle
LOOP_NS = 10_000_000  # 100 Hz card loop
GROUP3_ADDRS = range(GROUP3_START_ADDR, GROUP3_START_ADDR + GROUP3_MSG_COUNT)
EMPTY = {"OBJECT_ID": 0, "LONG_DIST": 204.7, "LAT_DIST": 0.0, "REL_SPEED": 0.0, "OBJECT_LENGTH": 0.0}


def ioniq9_fingerprint(group3=True, bus=None, length=GROUP3_MSG_LEN, skip=None):
  fingerprint = test_hyundai.TestHyundaiFingerprint._ioniq9_fingerprint()
  if group3:
    bus = CanBus(None, fingerprint).ACAN if bus is None else bus
    for addr in GROUP3_ADDRS:
      if addr != skip:
        fingerprint[bus][addr] = length
  return fingerprint


def ioniq9_params(fingerprint, alpha_long=False):
  return CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, test_hyundai.TestHyundaiFingerprint._ioniq9_fw(),
                                 alpha_long, False, False)


class Group3Feed:
  """Feeds a RadarInterface with packed group 3 frames like card does (100 Hz loop, one radar cycle every 5 loops)."""
  def __init__(self, CP):
    self.CP = CP
    self.RI = RadarInterface(CP)
    self.packer = CANPacker(GROUP3_DBC)
    self.bus = CanBus(CP).ACAN
    self.t = 0

  def cycle_msgs(self, objects, skip=()):
    slots = list(objects) + [EMPTY] * (GROUP3_MSG_COUNT - len(objects))
    return [CanData(*self.packer.make_can_msg(addr, self.bus, slot)) for addr, slot in zip(GROUP3_ADDRS, slots, strict=True)
            if addr not in skip]

  def step(self, msgs):
    self.t += LOOP_NS
    return self.RI.update([(self.t, msgs)])

  def cycle(self, objects, skip=()):
    """One radar cycle (5 card loops), returns every non-None output."""
    outs = [self.step(self.cycle_msgs(objects, skip))]
    outs += [self.step([]) for _ in range(4)]
    return [o for o in outs if o is not None]

  def quiet(self, seconds):
    return [o for o in (self.step([]) for _ in range(round(seconds * 1e9 / LOOP_NS))) if o is not None]

  def warm_up(self):
    """Run past the startup grace with an empty radar."""
    outs = []
    while self.t < GROUP3_STARTUP_GRACE_S * 1e9 + CYCLE_NS:
      outs += self.cycle([])
    return outs


def obj(object_id, x, y=0.0, v=0.0, length=4.6):
  return {"OBJECT_ID": object_id, "LONG_DIST": x, "LAT_DIST": y, "REL_SPEED": v, "OBJECT_LENGTH": length}


class TestGroup3Fingerprint(unittest.TestCase):
  def test_detected_only_with_every_slot_length_24_on_acan(self):
    CP = ioniq9_params(ioniq9_fingerprint())
    self.assertTrue(CP.flags & HyundaiFlags.CANFD_RADAR_GROUP3)
    self.assertFalse(CP.radarUnavailable)

    base = ioniq9_fingerprint(group3=False)
    cam = CanBus(None, base).CAM
    ecan = CanBus(None, base).ECAN
    for fingerprint in (base,
                        ioniq9_fingerprint(skip=GROUP3_START_ADDR + 7),  # one slot missing
                        ioniq9_fingerprint(length=8),  # same addresses as 8 byte messages (seen on bus 0 and 2)
                        ioniq9_fingerprint(bus=ecan),
                        ioniq9_fingerprint(bus=cam)):
      CP = ioniq9_params(fingerprint)
      self.assertFalse(CP.flags & HyundaiFlags.CANFD_RADAR_GROUP3)
      self.assertTrue(CP.radarUnavailable)

  def test_only_group3_meaning_changes(self):
    # radarUnavailable has no other users here: safety param, longitudinal availability and
    # the other flags must stay as without the radar
    for alpha_long in (False, True):
      with_radar = ioniq9_params(ioniq9_fingerprint(), alpha_long)
      without = ioniq9_params(ioniq9_fingerprint(group3=False), alpha_long)
      self.assertEqual(with_radar.flags & ~HyundaiFlags.CANFD_RADAR_GROUP3.value, without.flags)
      self.assertEqual([c.to_dict() for c in with_radar.safetyConfigs], [c.to_dict() for c in without.safetyConfigs])
      for field in ("alphaLongitudinalAvailable", "openpilotLongitudinalControl", "pcmCruise", "dashcamOnly",
                    "steerControlType", "longitudinalActuatorDelay", "radarDelay"):
        self.assertEqual(getattr(with_radar, field), getattr(without, field), field)

  def test_non_canfd_cars_unchanged(self):
    fingerprint = gen_empty_fingerprint()
    for addr in GROUP3_ADDRS:
      fingerprint[1][addr] = GROUP3_MSG_LEN
    CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
    self.assertFalse(CP.flags & HyundaiFlags.CANFD_RADAR_GROUP3)
    self.assertTrue(CP.radarUnavailable)

  def test_other_canfd_platform_follows_fingerprint(self):
    for group3 in (True, False):
      fingerprint = gen_empty_fingerprint()
      if group3:
        for addr in GROUP3_ADDRS:
          fingerprint[1][addr] = GROUP3_MSG_LEN
      CP = CarInterface.get_params(CAR.KIA_EV6, fingerprint, [], False, False, False)
      self.assertEqual(bool(CP.flags & HyundaiFlags.CANFD_RADAR_GROUP3), group3)
      self.assertEqual(CP.radarUnavailable, not group3)

  def test_no_group3_keeps_no_parser(self):
    RI = RadarInterface(ioniq9_params(ioniq9_fingerprint(group3=False)))
    self.assertIsNone(RI.rcp)


class TestGroup3Tracks(unittest.TestCase):
  def setUp(self):
    self.feed = Group3Feed(ioniq9_params(ioniq9_fingerprint()))
    self.feed.warm_up()

  def points(self, objects, skip=()):
    outs = self.feed.cycle(objects, skip)
    self.assertEqual(len(outs), 1)
    self.assertFalse(outs[0].errors.canError)
    return {round(p.dRel, 2): p for p in outs[0].points}

  def test_decode_and_empty_slots(self):
    pts = self.points([EMPTY, obj(12, 30.0, 1.5, -2.0, 4.6), obj(0, 40.0), obj(13, 204.7)])
    self.assertEqual(len(pts), 1)
    p = list(pts.values())[0]
    self.assertAlmostEqual(p.dRel, 30.0 - 2.3 - GROUP3_REAR_OFFSET, places=2)
    self.assertAlmostEqual(p.yRel, 1.5, places=2)  # left positive, as yRel
    self.assertAlmostEqual(p.vRel, -2.0, places=2)

  def test_object_keeps_track_when_slot_moves(self):
    first = self.points([obj(5, 20.0, 0.2, -1.0), obj(9, 50.0, -3.5, 1.0)])
    second = self.points([obj(9, 50.05, -3.5, 1.0), EMPTY, EMPTY, obj(5, 19.95, 0.2, -1.0)])
    by_y = lambda pts: {round(p.yRel): p.trackId for p in pts.values()}  # noqa: E731
    self.assertEqual(by_y(first), by_y(second))

  def test_duplicate_id_in_one_cycle_dropped(self):
    pts = self.points([obj(7, 20.0), obj(7, 35.0), obj(8, 60.0)])
    self.assertEqual(len(pts), 1)
    self.assertAlmostEqual(list(pts.values())[0].dRel, 60.0 - 2.3 - GROUP3_REAR_OFFSET, places=2)

  def test_jump_gets_new_track(self):
    base = list(self.points([obj(3, 30.0, 0.0, 0.0)]).values())[0].trackId
    same = list(self.points([obj(3, 30.0, 0.5, 0.5)]).values())[0].trackId
    self.assertEqual(same, base)
    ids = {base}
    # distance, then lateral, then speed jump
    for jumped in (obj(3, 37.0, 0.5, 0.5), obj(3, 37.0, 3.5, 0.5), obj(3, 37.0, 3.5, 4.0)):
      new_id = list(self.points([jumped]).values())[0].trackId
      self.assertNotIn(new_id, ids)
      ids.add(new_id)

  def test_gone_object_not_kept(self):
    self.points([obj(4, 25.0)])
    self.assertEqual(self.points([]), {})

  def test_stale_slot_not_used(self):
    self.points([obj(4, 25.0), obj(6, 45.0)])
    # slot 0x401 (object 6) misses this cycle: its old value must not come back
    pts = self.points([obj(4, 25.0), obj(6, 45.0)], skip=(GROUP3_START_ADDR + 1,))
    self.assertEqual(len(pts), 1)


class TestGroup3Startup(unittest.TestCase):
  def test_pause_right_after_start_is_not_a_fault(self):
    feed = Group3Feed(ioniq9_params(ioniq9_fingerprint()))
    outs = feed.cycle([obj(1, 20.0)])
    # logged: up to 0.61 s pause right after start
    paused = feed.quiet(0.8)
    outs += paused + feed.cycle([obj(1, 20.0)])
    self.assertTrue(all(not o.errors.canError for o in outs))
    # radard keeps getting radar data during the pause (empty, at least ~16 Hz)
    self.assertGreaterEqual(len(paused), int(0.8 * 16))
    self.assertTrue(all(len(o.points) == 0 for o in paused))

  def test_pause_while_driving_is_handled_as_before(self):
    feed = Group3Feed(ioniq9_params(ioniq9_fingerprint()))
    feed.warm_up()
    self.assertFalse(feed.cycle([obj(1, 20.0)])[0].errors.canError)
    # as before (Mando radar): no radar data while the radar is gone, radard then marks the radar invalid
    self.assertEqual(feed.quiet(0.8), [])

  def test_missing_slot_is_can_error_only_after_grace(self):
    skip = (GROUP3_START_ADDR + 5,)
    feed = Group3Feed(ioniq9_params(ioniq9_fingerprint()))
    outs = feed.cycle([obj(1, 20.0)])
    for _ in range(20):  # 1 s without one slot, inside the grace
      outs += feed.cycle([obj(1, 20.0)], skip)
    self.assertTrue(all(not o.errors.canError for o in outs))

    feed = Group3Feed(ioniq9_params(ioniq9_fingerprint()))
    feed.warm_up()
    outs = []
    for _ in range(20):  # 1 s without one slot while driving: stock CAN error
      outs += feed.cycle([obj(1, 20.0)], skip)
    self.assertFalse(outs[0].errors.canError)
    self.assertTrue(outs[-1].errors.canError)

  def test_radar_never_comes(self):
    feed = Group3Feed(ioniq9_params(ioniq9_fingerprint()))
    feed.step([CanData(0x100, b"\x00" * 8, 0)])
    outs = feed.quiet(GROUP3_STARTUP_GRACE_S)
    self.assertGreater(len(outs), 0)
    self.assertTrue(all(not o.errors.canError and len(o.points) == 0 for o in outs))
    # after the grace: as before, no radar data
    self.assertEqual(feed.quiet(1.0), [])

  def test_logged_counter_pattern_is_not_a_fault(self):
    # logged while driving (0000079b--11, 00000798--62): on some slots the counter repeats, then skips by 2
    feed = Group3Feed(ioniq9_params(ioniq9_fingerprint()))
    feed.warm_up()
    outs = []
    for i in range(40):
      msgs = feed.cycle_msgs([obj(1, 20.0)])
      for j in (0, 1):
        values = dict(obj(1, 20.0) if j == 0 else EMPTY, FRAME_COUNTER=(i // 2) * 2 % 256)
        msgs[j] = CanData(*feed.packer.make_can_msg(GROUP3_START_ADDR + j, feed.bus, values))
      outs += [o for o in [feed.step(msgs)] + [feed.step([]) for _ in range(4)] if o is not None]
    self.assertEqual(len(outs), 40)
    self.assertTrue(all(not o.errors.canError for o in outs))
    self.assertTrue(all(len(o.points) == 1 for o in outs))

  def test_single_can_string_form(self):
    # test_models passes one (nanos, frames) at a time
    feed = Group3Feed(ioniq9_params(ioniq9_fingerprint()))
    feed.warm_up()
    feed.t += LOOP_NS
    out = feed.RI.update((feed.t, feed.cycle_msgs([obj(1, 20.0)])))
    self.assertIsNotNone(out)
    self.assertEqual(len(out.points), 1)
    self.assertFalse(out.errors.canError)

  def test_bad_checksum_frames_ignored(self):
    feed = Group3Feed(ioniq9_params(ioniq9_fingerprint()))
    feed.warm_up()
    msgs = feed.cycle_msgs([obj(2, 30.0)])
    bad = copy.copy(msgs)
    dat = bytearray(bad[0].dat)
    dat[0] ^= 0xff
    bad[0] = CanData(bad[0].address, bytes(dat), bad[0].src)
    out = feed.step(bad)
    self.assertIsNotNone(out)
    self.assertEqual(len(out.points), 0)


if __name__ == "__main__":
  unittest.main()
