"""KOVA: the cluster NAV icon (CCNC_0x161 NAV_ICON) with openpilot longitudinal: green while openpilot longitudinal is
engaged and navigation info is received (car navigation or a navigation app, hudControl.navigationActive), gray while
ready, engaged without navigation info, or the other way round (CAN-FD angle steering car).

KOVA: 우리 종방향 켜짐 + 내비 정보 받는 중이면 계기판 NAV 초록, 대기·내비 정보 없음이면 회색
(janpoo 2026-11-28 10:52·10:53, 크루즈 아이콘처럼)."""
import random
import unittest

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.tests.test_hyundai_blindspot import STEP_NS
from opendbc.car.hyundai.tests.test_hyundai_cluster_speed import (CRUISE_DISPLAY_BITS, SET_SPEED_BITS, STOCK_OFF, ioniq9_long,
                                                                 run, signals)
from opendbc.car.hyundai.tests.test_hyundai_hands_on import CHECKSUM_BITS, bits, random_ccnc

GRAY, GREEN = 1, 2
# NAV_ICON is bits 220..223
NAV_ICON_BITS = set(range(220, 224))
# stock camera NAV_ICON values (rlogs 00000798~0000079e 2026-11-27): hidden, gray (ready), green, white (engaged)
STOCK_NAV = (0, 1, 2, 4)


def car_control(enabled, lat_active, navigation_active, set_speed_kph=60.0, bars=3):
  CC = structs.CarControl()
  CC.enabled = enabled
  CC.latActive = lat_active
  CC.longActive = enabled
  CC.hudControl.setSpeed = set_speed_kph * CV.KPH_TO_MS
  CC.hudControl.leadDistanceBars = bars
  CC.hudControl.navigationActive = navigation_active
  return CC.as_reader()


def copy_nav(stock_nav, CC, cruise_available=True):
  _, CI, packer = ioniq9_long()
  # TCS first, then the stock frame (the cruise state follows the car one frame later)
  run(CI, packer, None, CC, 0, cruise_available=cruise_available)
  sends = run(CI, packer, {**STOCK_OFF, "COUNTER": 1, "NAV_ICON": stock_nav}, CC, 1, cruise_available=cruise_available)
  copies = [s for s in sends if s[0] == 0x161]
  return signals(copies, 0x161, "CCNC_0x161")["NAV_ICON"] if copies else None


class TestCcncNavIcon(unittest.TestCase):
  def test_green_while_engaged_with_navigation(self):
    for stock_nav in STOCK_NAV:
      for lat_active in (True, False):
        self.assertEqual(copy_nav(stock_nav, car_control(True, lat_active, True)), GREEN, (stock_nav, lat_active))

  def test_gray_while_engaged_without_navigation(self):
    for stock_nav in STOCK_NAV:
      for lat_active in (True, False):
        self.assertEqual(copy_nav(stock_nav, car_control(True, lat_active, False)), GRAY, (stock_nav, lat_active))

  def test_gray_while_ready(self):
    # cruise available, not engaged: gray with or without navigation info (also always-on lateral steering)
    for stock_nav in STOCK_NAV:
      for lat_active in (True, False):
        for navigation in (True, False):
          self.assertEqual(copy_nav(stock_nav, car_control(False, lat_active, navigation)), GRAY,
                           (stock_nav, lat_active, navigation))

  def test_stock_without_openpilot_cruise(self):
    # cruise not available: only the always-on lateral copy, NAV icon as the stock camera draws it
    for stock_nav in STOCK_NAV:
      self.assertEqual(copy_nav(stock_nav, car_control(False, True, True), cruise_available=False), stock_nav, stock_nav)
      self.assertIsNone(copy_nav(stock_nav, car_control(False, False, True), cruise_available=False), stock_nav)

  def test_only_nav_bits_change_with_navigation(self):
    # the same frame with and without navigation info differs only in the NAV icon, green only while engaged
    rng = random.Random(0x1612)
    for frame in range(40):
      enabled = frame % 2 == 0
      lat_active, set_speed = rng.random() < 0.5, float(rng.randrange(30, 150))
      sends = {}
      for navigation in (False, True):
        _, CI, packer = ioniq9_long()
        stock = random_ccnc(packer, random.Random(frame), bus=CanBus(CI.CP).CAM)
        stock_values = signals([stock], 0x161, "CCNC_0x161")
        stock_values.pop("CHECKSUM")
        stock_values["ALERTS_2"] = 0
        t = STEP_NS
        _, dat_stock, _ = packer.make_can_msg("CCNC_0x161", CanBus(CI.CP).CAM, stock_values)
        CI.update([(t, [CanData(0x161, dat_stock, CanBus(CI.CP).CAM)])])
        _, out = CI.apply(car_control(enabled, lat_active, navigation, set_speed), t)
        copies = [s for s in out if s[0] == 0x161]
        self.assertEqual(len(copies), 1)
        sends[navigation] = copies[0][1]
        before, after = bits(dat_stock), bits(copies[0][1])
        changed = {i for i in range(len(before)) if before[i] != after[i]}
        allowed = SET_SPEED_BITS | CRUISE_DISPLAY_BITS | CHECKSUM_BITS | set(range(184, 189)) | set(range(192, 196))
        self.assertTrue(changed <= allowed, sorted(changed - allowed))
      a, b = bits(sends[False]), bits(sends[True])
      diff = {i for i in range(len(a)) if a[i] != b[i]}
      self.assertTrue(diff <= NAV_ICON_BITS | CHECKSUM_BITS, sorted(diff))
      self.assertEqual(bool(diff), enabled, frame)
      nav = (signals([(0x161, sends[n], 0)], 0x161, "CCNC_0x161")["NAV_ICON"] for n in (False, True))
      self.assertEqual(tuple(nav), (GRAY, GREEN if enabled else GRAY), frame)


if __name__ == "__main__":
  unittest.main()
