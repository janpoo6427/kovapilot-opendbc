"""KOVA: CAN-FD vEgoCluster is the instrument cluster speed (CRUISE_BUTTONS_ALT CLUSTER_SPEED, integer km/h).
IONIQ 9 drive 2026-11-28 (rlog 0000079f segment 20): wheel speed 86 km/h while the cluster showed 89."""
import unittest

from opendbc.car.can_definitions import CanData
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.carstate import CLUSTER_SPEED_CONFIRM_FRAMES
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.tests.test_hyundai_blindspot import STEP_NS, ioniq9

FRAMES = CLUSTER_SPEED_CONFIRM_FRAMES + 20


def wheels(kph):
  return {f"WHL_Spd{w}Val": kph for w in ("FL", "FR", "RL", "RR")}


class TestCanfdEgoClusterSpeed(unittest.TestCase):
  def drive(self, CI, packer, wheel_kph, cluster_kph, *, frames=FRAMES, start=0, unit=0, cluster=True):
    CS = None
    for i in range(start, start + frames):
      msgs = [CanData(*packer.make_can_msg("WHEEL_SPEEDS", CanBus(CI.CP).ECAN, wheels(wheel_kph)))]
      if cluster:
        msgs.append(CanData(*packer.make_can_msg("CRUISE_BUTTONS_ALT", CanBus(CI.CP).ECAN,
                                                 {"CLUSTER_SPEED": cluster_kph, "DISTANCE_UNIT": unit})))
      CS = CI.update([((i + 1) * STEP_NS, msgs)])
    return CS

  def test_cluster_speed_read(self):
    # the cluster shows 89 at a wheel speed of 86: vEgoCluster is the cluster number, vEgo stays the wheel speed
    _, CI, packer = ioniq9()
    CS = self.drive(CI, packer, 86.0, 89)
    self.assertAlmostEqual(CS.vEgoCluster * CV.MS_TO_KPH, 89.0, places=3)
    self.assertAlmostEqual(CS.vEgoRaw * CV.MS_TO_KPH, 86.0, delta=0.1)

  def test_cluster_speed_needs_confirmation(self):
    # trusted only after it matched the wheel speed for a while
    _, CI, packer = ioniq9()
    CS = self.drive(CI, packer, 86.0, 89, frames=CLUSTER_SPEED_CONFIRM_FRAMES - 10)
    self.assertAlmostEqual(CS.vEgoCluster, CS.vEgo, places=6)

  def test_without_signal_wheel_speed(self):
    _, CI, packer = ioniq9()
    CS = self.drive(CI, packer, 86.0, 0, cluster=False)
    self.assertAlmostEqual(CS.vEgoCluster, CS.vEgo, places=6)

  def test_implausible_value_wheel_speed(self):
    for cluster_kph in (0, 70, 120, 255):
      _, CI, packer = ioniq9()
      CS = self.drive(CI, packer, 86.0, cluster_kph)
      self.assertAlmostEqual(CS.vEgoCluster, CS.vEgo, places=6, msg=cluster_kph)
    # a trusted signal that turns implausible falls back at once
    _, CI, packer = ioniq9()
    self.drive(CI, packer, 86.0, 89)
    CS = self.drive(CI, packer, 86.0, 140, frames=1, start=FRAMES)
    self.assertAlmostEqual(CS.vEgoCluster, CS.vEgo, places=6)

  def test_stale_signal_wheel_speed(self):
    _, CI, packer = ioniq9()
    self.drive(CI, packer, 86.0, 89)
    CS = self.drive(CI, packer, 86.0, 89, frames=60, start=FRAMES, cluster=False)
    self.assertAlmostEqual(CS.vEgoCluster, CS.vEgo, places=6)

  def test_not_metric_wheel_speed(self):
    # DISTANCE_UNIT 1 (miles): the unit of CLUSTER_SPEED is unknown there, keep the wheel speed
    _, CI, packer = ioniq9()
    CS = self.drive(CI, packer, 86.0, 89, unit=1)
    self.assertAlmostEqual(CS.vEgoCluster, CS.vEgo, places=6)

  def test_standstill_and_low_speed(self):
    _, CI, packer = ioniq9()
    self.drive(CI, packer, 30.0, 33)
    CS = self.drive(CI, packer, 0.0, 0, start=FRAMES)
    self.assertEqual(CS.vEgoCluster, 0.0)
    CS = self.drive(CI, packer, 5.0, 6, start=2 * FRAMES)
    self.assertAlmostEqual(CS.vEgoCluster * CV.MS_TO_KPH, 6.0, places=3)


if __name__ == "__main__":
  unittest.main()
