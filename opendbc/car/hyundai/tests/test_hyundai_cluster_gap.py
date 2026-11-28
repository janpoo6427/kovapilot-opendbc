"""KOVA: the cluster gap display (CCNC_0x161 DISTANCE_SPACING, DISTANCE_LEAD) after an engage without the stock camera
buttons (automatic engage) matches the one after a stock camera button engage (CAN-FD angle steering car).

KOVA: 자동 인게이지(순정 카메라 버튼 없이 켜짐) 뒤에도 계기판 차간 표시가 버튼으로 켰을 때와 같다(rlog 0000079f 2026-11-28)."""
import random
import unittest

from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.tests.test_hyundai_blindspot import STEP_NS
from opendbc.car.hyundai.tests.test_hyundai_cluster_speed import CRUISE_DISPLAY_BITS, SET_SPEED_BITS, ioniq9_long, run, signals
from opendbc.car.hyundai.tests.test_hyundai_hands_on import CHECKSUM_BITS, bits, random_ccnc

GAP = ("DISTANCE", "DISTANCE_CAR", "DISTANCE_LEAD", "DISTANCE_SPACING")
# DISTANCE_LEAD bits 184..188, DISTANCE_SPACING bits 192..195
GAP_BITS = set(range(184, 189)) | set(range(192, 196))

# stock camera frames, rlog 0000079f segment 4 (2026-11-28):
# its own cruise engaged right after a MAIN press (260.12 s), the lead it follows white, spacing shown
STOCK_ENGAGED = {"SETSPEED": 3, "SETSPEED_HUD": 2, "DISTANCE_CAR": 2, "DISTANCE": 4, "DISTANCE_LEAD": 2,
                 "DISTANCE_SPACING": 1, "TARGET": 1, "TARGET_DISTANCE": 17.0, "SETSPEED_SPEED": 30}
# its own cruise only ready after an automatic engage (268.25 s, no button), a seen lead gray, no spacing
STOCK_READY = {"SETSPEED": 1, "SETSPEED_HUD": 1, "DISTANCE_CAR": 1, "DISTANCE": 4, "DISTANCE_LEAD": 1,
               "DISTANCE_SPACING": 0, "TARGET_DISTANCE": 204.6, "SETSPEED_SPEED": 30}
# MAIN off (segment 3, 204.25 s automatic engage)
STOCK_OFF = {"SETSPEED": 0, "SETSPEED_HUD": 0, "DISTANCE_CAR": 0, "DISTANCE": 0, "DISTANCE_LEAD": 0,
             "DISTANCE_SPACING": 0, "TARGET_DISTANCE": 204.6, "SETSPEED_SPEED": 255}


def car_control(enabled, lat_active, lead_visible, bars=3, set_speed_kph=40.0):
  CC = structs.CarControl()
  CC.enabled = enabled
  CC.latActive = lat_active
  CC.longActive = enabled
  CC.hudControl.setSpeed = set_speed_kph * CV.KPH_TO_MS
  CC.hudControl.leadDistanceBars = bars
  CC.hudControl.leadVisible = lead_visible
  return CC.as_reader()


def copy_of(stock, CC):
  _, CI, packer = ioniq9_long()
  sends = run(CI, packer, {**stock, "COUNTER": 1}, CC)
  return signals([s for s in sends if s[0] == 0x161], 0x161, "CCNC_0x161")


class TestCcncGapAfterAutomaticEngage(unittest.TestCase):
  def test_gap_matches_button_engage(self):
    # same openpilot state, the stock camera engaged (button) or not (automatic engage): same gap display
    for lat_active in (True, False):
      for bars in (1, 2, 3, 4):
        CC = car_control(True, lat_active, True, bars)
        button = copy_of(STOCK_ENGAGED, CC)
        for name, stock in (("ready", STOCK_READY), ("off", STOCK_OFF)):
          auto = copy_of(stock, CC)
          self.assertEqual({k: auto[k] for k in GAP}, {k: button[k] for k in GAP}, (lat_active, bars, name))
          self.assertEqual({k: auto[k] for k in GAP},
                           {"DISTANCE": bars, "DISTANCE_CAR": 2, "DISTANCE_LEAD": 2, "DISTANCE_SPACING": 1},
                           (lat_active, bars, name))

  def test_button_engage_copy_keeps_stock_gap(self):
    # the stock camera already draws its engaged gap: the copy shows it unchanged
    copy = copy_of(STOCK_ENGAGED, car_control(True, True, True, 4))
    self.assertEqual({k: copy[k] for k in GAP + ("TARGET", "TARGET_DISTANCE")},
                     {k: STOCK_ENGAGED[k] for k in GAP + ("TARGET", "TARGET_DISTANCE")})

  def test_no_followed_lead(self):
    # spacing shown while engaged; without a followed lead the lead icon stays as the stock camera sees it
    for stock_lead in (0, 1):
      copy = copy_of({**STOCK_READY, "DISTANCE_LEAD": stock_lead}, car_control(True, True, False))
      self.assertEqual((copy["DISTANCE_SPACING"], copy["DISTANCE_LEAD"]), (1, stock_lead), stock_lead)

  def test_ready_keeps_stock_gap(self):
    # cruise ready, not engaged: gray icons only, spacing and lead icon stay stock (panda allows only those without control)
    for stock_lead in (0, 1):
      copy = copy_of({**STOCK_READY, "DISTANCE_LEAD": stock_lead}, car_control(False, False, True))
      self.assertEqual((copy["SETSPEED"], copy["DISTANCE_SPACING"], copy["DISTANCE_LEAD"]), (1, 0, stock_lead), stock_lead)

  def test_only_display_bits_change_after_automatic_engage(self):
    rng = random.Random(0x1611)
    _, CI, packer = ioniq9_long()
    for frame in range(0, 200, 5):
      stock = random_ccnc(packer, rng, bus=CanBus(CI.CP).CAM)
      stock_values = signals([stock], 0x161, "CCNC_0x161")
      stock_values.pop("CHECKSUM")
      stock_values["ALERTS_2"] = 0  # no hands-on warning in this frame
      t = (frame + 1) * STEP_NS
      _, dat_stock, _ = packer.make_can_msg("CCNC_0x161", CanBus(CI.CP).CAM, stock_values)
      CI.update([(t, [CanData(0x161, dat_stock, CanBus(CI.CP).CAM)])])
      _, sends = CI.apply(car_control(True, False, rng.random() < 0.5, rng.randrange(1, 5)), t)
      copies = [s for s in sends if s[0] == 0x161]
      self.assertEqual(len(copies), 1)
      before, after = bits(dat_stock), bits(copies[0][1])
      changed = {i for i in range(len(before)) if before[i] != after[i]}
      allowed = SET_SPEED_BITS | CRUISE_DISPLAY_BITS | GAP_BITS | CHECKSUM_BITS
      self.assertTrue(changed <= allowed, sorted(changed - allowed))


if __name__ == "__main__":
  unittest.main()
