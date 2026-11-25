from opendbc.car.can_definitions import CanData


TRAILER_STATUS_ADDRESS = 0x437
TRAILER_STATUS_LENGTH = 8
# Provisional timing for log observation. Validate against the vehicle's frame cadence.
TRAILER_STATUS_MAX_AGE_NS = 2_000_000_000
TRAILER_DISCONNECT_GRACE_NS = 5_000_000_000


class TrailerStateObserver:
  def __init__(self, bus: int):
    self.bus = bus
    self.last_sample_ns: int | None = None
    self.disconnect_since_ns: int | None = None
    self.connected = False

  def _observe(self, sample_ns: int, connected: bool) -> None:
    if self.last_sample_ns is not None:
      if sample_ns <= self.last_sample_ns:
        return
      if sample_ns - self.last_sample_ns > TRAILER_STATUS_MAX_AGE_NS:
        self.disconnect_since_ns = None
    self.last_sample_ns = sample_ns

    if connected:
      self.connected = True
      self.disconnect_since_ns = None
    elif self.connected:
      if self.disconnect_since_ns is None:
        self.disconnect_since_ns = sample_ns
      elif sample_ns - self.disconnect_since_ns >= TRAILER_DISCONNECT_GRACE_NS:
        self.connected = False
        self.disconnect_since_ns = None

  def update(self, can_packets: list[tuple[int, list[CanData]]]) -> str:
    if not can_packets:
      return "unknown"

    now_ns = 0
    for sample_ns, frames in can_packets:
      now_ns = max(now_ns, sample_ns)
      for address, data, bus in frames:
        if address == TRAILER_STATUS_ADDRESS and bus == self.bus and len(data) == TRAILER_STATUS_LENGTH:
          self._observe(sample_ns, bool(data[0] & 1))

    if self.last_sample_ns is None or not 0 <= now_ns - self.last_sample_ns <= TRAILER_STATUS_MAX_AGE_NS:
      return "unknown"
    return "connected" if self.connected else "disconnected"
