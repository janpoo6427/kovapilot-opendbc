from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai.trailer import TrailerStateObserver


SECOND = 1_000_000_000


def update(observer: TrailerStateObserver, second: int, connected: bool | None = None) -> str:
  frames = [] if connected is None else [CanData(0x437, bytes([connected]) + bytes(7), 0)]
  return observer.update([(second * SECOND, frames)])


def test_trailer_attachment_and_disconnect_debounce():
  observer = TrailerStateObserver(0)
  assert update(observer, 1) == "unknown"
  assert update(observer, 2, False) == "disconnected"
  assert update(observer, 3, True) == "connected"
  for second in range(4, 9):
    assert update(observer, second, False) == "connected"
  assert update(observer, 9, False) == "disconnected"
  assert update(observer, 10, True) == "connected"


def test_stale_signal_is_unknown_and_cannot_confirm_disconnect():
  observer = TrailerStateObserver(0)
  assert update(observer, 1, True) == "connected"
  assert update(observer, 2, False) == "connected"
  assert update(observer, 4) == "connected"
  assert update(observer, 5) == "unknown"
  assert observer.update([]) == "unknown"
  for second in range(7, 12):
    assert update(observer, second, False) == "connected"
  assert update(observer, 12, False) == "disconnected"


def test_ignores_wrong_bus_address_and_payload_length():
  observer = TrailerStateObserver(0)
  frames = [CanData(0x437, bytes([1]) + bytes(7), 2), CanData(0x438, bytes([1]) + bytes(7), 0),
            CanData(0x437, bytes([1]) + bytes(6), 0), CanData(0x437, bytes([1]) + bytes(8), 0)]
  assert observer.update([(SECOND, frames)]) == "unknown"


def test_ignores_repeated_and_out_of_order_timestamps():
  observer = TrailerStateObserver(0)
  assert update(observer, 2, False) == "disconnected"
  assert update(observer, 2, True) == "disconnected"
  assert update(observer, 1, True) == "unknown"
  assert update(observer, 3) == "disconnected"
