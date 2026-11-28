import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import DBC, HyundaiFlags

RADAR_START_ADDR = 0x500
RADAR_MSG_COUNT = 32

# CAN-FD front radar object list ("group 3"): 30 slots of 24 bytes on the A-CAN at 20 Hz.
# Detected from the fingerprint only (all slots present with length 24, see interface.py).
GROUP3_DBC = "hyundai_canfd_radar_group3_generated"
GROUP3_START_ADDR = 0x400
GROUP3_MSG_COUNT = 30
GROUP3_MSG_LEN = 24
GROUP3_EMPTY_DIST = 204.7  # LONG_DIST raw 0x7ff marks an empty slot
GROUP3_DT = 0.05

# The same OBJECT_ID in the next cycle is the same track unless it moved more than this
# (logged Ioniq 9 continuations: 99.99% under 1.9 m lateral and 1.3 m/s).
GROUP3_MAX_DIST_JUMP = 5.0  # m, from the position predicted with the previous speed
GROUP3_MAX_LAT_JUMP = 2.5  # m
GROUP3_MAX_SPEED_JUMP = 3.0  # m/s

# LONG_DIST is the object center; dRel is to its rear. On logged Ioniq 9 drives (7 segments)
# LONG_DIST - OBJECT_LENGTH / 2 was 0.2-0.3 m (median, mean 0.23 m) longer than the stock SCC lead distance.
GROUP3_REAR_OFFSET = 0.2  # m

# Right after the car starts, the radar was logged to pause for up to 0.61 s (twice, 1-3.4 s
# after logging starts, before card is up). During the first seconds a pause is not a fault:
# keep publishing empty radar data and no CAN error. Afterwards a missing radar is handled as usual.
GROUP3_STARTUP_GRACE_S = 5.0
GROUP3_STARTUP_IDLE_FRAMES = 6  # card loop frames (100 Hz) without a radar cycle before an empty update

# POC for parsing corner radars: https://github.com/commaai/openpilot/pull/24221/


def get_radar_can_parser(CP):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  messages = [(f"RADAR_TRACK_{addr:x}", 50) for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT)]
  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


def get_group3_can_parser(CP):
  messages = [(f"RADAR_GROUP3_{addr:x}", 20) for addr in range(GROUP3_START_ADDR, GROUP3_START_ADDR + GROUP3_MSG_COUNT)]
  return CANParser(GROUP3_DBC, messages, CanBus(CP).ACAN)


def group3_present(fingerprint_bus: dict[int, int]) -> bool:
  return all(fingerprint_bus.get(addr) == GROUP3_MSG_LEN for addr in range(GROUP3_START_ADDR, GROUP3_START_ADDR + GROUP3_MSG_COUNT))


class Group3Tracker:
  """Joins group 3 objects across cycles by OBJECT_ID. Slots are only transport positions."""

  def __init__(self):
    self.next_track_id = 0
    self.tracks: dict[int, tuple[int, float, float, float]] = {}  # OBJECT_ID -> (trackId, x, y, v)

  def update(self, objects: list[tuple[int, float, float, float, float]]) -> list[structs.RadarData.RadarPoint]:
    """objects: (OBJECT_ID, LONG_DIST, LAT_DIST, REL_SPEED, OBJECT_LENGTH) from one cycle."""
    counts: dict[int, int] = {}
    for obj in objects:
      counts[obj[0]] = counts.get(obj[0], 0) + 1

    points = []
    tracks = {}
    for object_id, x, y, v, length in objects:
      # an OBJECT_ID seen twice in one cycle is ambiguous: drop both
      if object_id == 0 or counts[object_id] != 1 or not (0.0 <= x < GROUP3_EMPTY_DIST):
        continue
      if not all(math.isfinite(val) for val in (x, y, v, length)):
        continue

      prev = self.tracks.get(object_id)
      if prev is not None and (abs(x - (prev[1] + prev[3] * GROUP3_DT)) <= GROUP3_MAX_DIST_JUMP and
                               abs(y - prev[2]) <= GROUP3_MAX_LAT_JUMP and abs(v - prev[3]) <= GROUP3_MAX_SPEED_JUMP):
        track_id = prev[0]
      else:
        track_id = self.next_track_id
        self.next_track_id += 1
      tracks[object_id] = (track_id, x, y, v)

      pt = structs.RadarData.RadarPoint()
      pt.trackId = track_id
      pt.dRel = max(0.0, x - length / 2 - GROUP3_REAR_OFFSET)
      pt.yRel = y
      pt.vRel = v
      points.append(pt)

    self.tracks = tracks
    return points


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.group3 = bool(CP.flags & HyundaiFlags.CANFD_RADAR_GROUP3) and not CP.radarUnavailable
    if self.group3:
      self.rcp = get_group3_can_parser(CP)
      self.trigger_msg = GROUP3_START_ADDR + GROUP3_MSG_COUNT - 1
      self.tracker = Group3Tracker()
      self.start_nanos: int | None = None
      self.last_nanos = 0
      self.idle_frames = 0
    else:
      self.rcp = None if CP.radarUnavailable else get_radar_can_parser(CP)
      self.trigger_msg = RADAR_START_ADDR + RADAR_MSG_COUNT - 1

  def update(self, can_strings):
    if not self.group3:
      return super().update(can_strings)

    # card passes a list of (nanos, frames); a single (nanos, frames) is accepted too, as CANParser does
    if len(can_strings) and not isinstance(can_strings[0], list | tuple):
      can_strings = [can_strings]
    if len(can_strings):
      self.last_nanos = can_strings[-1][0]
      if self.start_nanos is None:
        self.start_nanos = can_strings[0][0]
    ret = super().update(can_strings)

    if self.start_nanos is not None and self.last_nanos - self.start_nanos >= GROUP3_STARTUP_GRACE_S * 1e9:
      return ret

    if ret is None:
      self.idle_frames += 1
      if self.idle_frames < GROUP3_STARTUP_IDLE_FRAMES:
        return None
      ret = structs.RadarData()
    self.idle_frames = 0
    ret.errors.canError = False
    return ret

  def _update(self, rcp, updated_messages):
    if self.group3:
      return self._update_group3(rcp, updated_messages)

    ret = structs.RadarData()
    if not rcp.can_valid:
      ret.errors.canError = True

    for addr in range(RADAR_START_ADDR, RADAR_START_ADDR + RADAR_MSG_COUNT):
      msg = rcp.vl[f"RADAR_TRACK_{addr:x}"]

      if addr not in self.pts:
        self.pts[addr] = structs.RadarData.RadarPoint()
        self.pts[addr].trackId = self.track_id
        self.track_id += 1

      valid = msg['STATE'] in (3, 4)
      if valid:
        azimuth = math.radians(msg['AZIMUTH'])
        self.pts[addr].dRel = math.cos(azimuth) * msg['LONG_DIST']
        self.pts[addr].yRel = 0.5 * -math.sin(azimuth) * msg['LONG_DIST']
        self.pts[addr].vRel = msg['REL_SPEED']

      else:
        del self.pts[addr]

    ret.points = list(self.pts.values())
    return ret

  def _update_group3(self, rcp, updated_messages):
    ret = structs.RadarData()
    if not rcp.can_valid:
      ret.errors.canError = True

    objects = []
    for addr in range(GROUP3_START_ADDR, GROUP3_START_ADDR + GROUP3_MSG_COUNT):
      # only slots refreshed in this cycle; a stale slot must not keep an old object alive
      if addr not in updated_messages:
        continue
      msg = rcp.vl[addr]
      objects.append((int(msg["OBJECT_ID"]), msg["LONG_DIST"], msg["LAT_DIST"], msg["REL_SPEED"], msg["OBJECT_LENGTH"]))

    ret.points = self.tracker.update(objects)
    return ret
