import unittest
from types import SimpleNamespace

from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC


class TestHyundaiAngleController(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    fingerprint = gen_empty_fingerprint()
    CAN = CanBus(None, fingerprint)
    config = CAR.HYUNDAI_IONIQ_9.config.steering_config
    fingerprint[getattr(CAN, config.stock_bus.value)][config.command_address] = config.command_dlc
    fingerprint[CAN.ECAN][0x1CF] = 8
    cls.CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, [], False, False, False)

  @staticmethod
  def _stock_template():
    return {
      "CHECKSUM": 0,
      "COUNTER": 0,
      "ADAS_ActvACISta": 0,
      "ADAS_ActvACILvl2Sta": 1,
      "ADAS_StrAnglReqVal": 0,
      "ADAS_ACIAnglTqRedcGainVal": 0,
      "FCA_ESA_ActvSta": 0,
      "FCA_ESA_TqBstGainVal": 0,
    }

  @staticmethod
  def _car_state(angle=0.0, steering_pressed=False, template=None):
    out = structs.CarState(vEgo=10, vEgoRaw=10, steeringAngleDeg=angle, steeringTorque=0,
                           steeringPressed=steering_pressed)
    return SimpleNamespace(out=out.as_reader(), is_metric=True, lfa_alt=template,
                           lfa_block_msg={}, buttons_counter=0, cruise_info={})

  def test_active_angle_ramps_authority_and_inactive_tracks_measured(self):
    controller = CarController(DBC[self.CP.carFingerprint], self.CP)
    template = self._stock_template()

    active = structs.CarControl(enabled=True, latActive=True,
                                actuators=structs.CarControl.Actuators(steeringAngleDeg=30)).as_reader()
    actuators, sends = controller.update(active, self._car_state(template=template), 0)
    angle_msg = next(msg for msg in sends if msg[0] == 0xCB)
    assert angle_msg[2] == CanBus(self.CP).ECAN
    assert angle_msg[1][3] >> 4 == 2
    assert angle_msg[1][6] == 5
    assert abs(actuators.steeringAngleDeg) <= 1.0
    assert actuators.torqueOutputCan == 5

    inactive = structs.CarControl(enabled=False, latActive=False,
                                  actuators=structs.CarControl.Actuators(steeringAngleDeg=0)).as_reader()
    actuators, sends = controller.update(inactive, self._car_state(angle=8, template=template), 10_000_000)
    angle_msg = next(msg for msg in sends if msg[0] == 0xCB)
    assert angle_msg[1][3] >> 4 == 1
    assert angle_msg[1][6] == 0
    assert actuators.steeringAngleDeg == 8
    assert actuators.torqueOutputCan == 0

  def test_missing_template_or_driver_override_never_requests_angle(self):
    controller = CarController(DBC[self.CP.carFingerprint], self.CP)
    active = structs.CarControl(enabled=True, latActive=True,
                                actuators=structs.CarControl.Actuators(steeringAngleDeg=10)).as_reader()

    _, sends = controller.update(active, self._car_state(template=None), 0)
    assert all(msg[0] != 0xCB for msg in sends)

    _, sends = controller.update(active, self._car_state(steering_pressed=True, template=self._stock_template()), 10_000_000)
    angle_msg = next(msg for msg in sends if msg[0] == 0xCB)
    assert angle_msg[1][3] >> 4 == 1
    assert angle_msg[1][6] == 0


if __name__ == "__main__":
  unittest.main()
