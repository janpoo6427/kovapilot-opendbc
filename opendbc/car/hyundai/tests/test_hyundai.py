import unittest
import copy
from dataclasses import replace
from types import SimpleNamespace
from unittest.mock import patch

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.structs import CarParams
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.carcontroller import (hyundai_angle_lateral_authorized, ANGLE_DRIVER_TORQUE_LIMIT, ANGLE_DRIVER_TORQUE_RELEASE_FRAMES,
                                                ANGLE_ENGAGE_GRACE_FRAMES, CarController, apply_hyundai_angle_limits,
                                                get_hyundai_steering_output_can,
                                                update_hyundai_angle_engage, update_hyundai_angle_handover,
                                                update_hyundai_angle_torque_reduction_gain)
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.hyundaicanfd import (CanBus, create_acc_control_from_stock, create_angle_camera_mdps,
                                              create_angle_steering, hkg_can_fd_checksum)
from opendbc.car.hyundai.radar_interface import RADAR_START_ADDR
from opendbc.car.hyundai.values import (CAR, DATE_FW_ECUS, CANFD_FUZZY_WHITELIST, PLATFORM_CODE_ECUS,
                                        HYUNDAI_VERSION_REQUEST_LONG, HyundaiCanFDEcuDisableStrategy, HyundaiCanFDHarnessConnection,
                                        HyundaiCanFDSteeringApi, HyundaiCanFDStockSafetyStatus, HyundaiFlags,
                                        DBC, get_platform_codes, HyundaiSafetyFlags, match_fw_to_car_fuzzy)
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.car.vehicle_model import VehicleModel
from opendbc.testing import fuzzy_test

Ecu = CarParams.Ecu

# Some platforms have date codes in a different format we don't yet parse (or are missing).
# For now, assert list of expected missing date cars
NO_DATES_PLATFORMS = {
  # CAN FD
  CAR.KIA_SPORTAGE_5TH_GEN,
  CAR.HYUNDAI_SANTA_CRUZ_1ST_GEN,
  CAR.HYUNDAI_TUCSON_4TH_GEN,
  # CAN
  CAR.HYUNDAI_ELANTRA,
  CAR.HYUNDAI_ELANTRA_GT_I30,
  CAR.KIA_CEED,
  CAR.KIA_CEED_PHEV,
  CAR.KIA_FORTE,
  CAR.KIA_OPTIMA_G4,
  CAR.KIA_OPTIMA_G4_FL,
  CAR.KIA_SORENTO,
  CAR.HYUNDAI_KONA,
  CAR.HYUNDAI_KONA_EV,
  CAR.HYUNDAI_KONA_EV_2022,
  CAR.HYUNDAI_KONA_HEV,
  CAR.HYUNDAI_SONATA_LF,
  CAR.HYUNDAI_VELOSTER,
  CAR.HYUNDAI_KONA_2022,
}

CANFD_EXPECTED_ECUS = {Ecu.fwdCamera, Ecu.fwdRadar, Ecu.adas}

# CAN-only feature flags that should not appear on CAN FD platforms
CAN_FEATURE_FLAGS = (HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.TCU_GEARS | HyundaiFlags.CHECKSUM_CRC8 |
                     HyundaiFlags.CHECKSUM_6B | HyundaiFlags.LEGACY | HyundaiFlags.UNSUPPORTED_LONGITUDINAL |
                     HyundaiFlags.CAMERA_SCC)


def cars_with(flags):
  return {c for c in CAR if c.config.flags & flags}


class TestHyundaiCanFDAngleCommand(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker("hyundai_canfd_generated")
    self.CAN = CanBus(None, gen_empty_fingerprint())

  @staticmethod
  def assert_valid_crc(msg):
    address, dat, _ = msg
    assert int.from_bytes(dat[:2], "little") == hkg_can_fd_checksum(address, None, bytearray(dat))

  def test_active_command_matches_observed_lfa_alt_contract(self):
    template = {
      "ADAS_ActvACISta": 0xF,
      "ADAS_ActvACILvl2Sta": 0xF,
      "FCA_ESA_ActvSta": 0x3,
      "BYTE7_RESERVED": 0x3F,
      "FCA_ESA_TqBstGainVal": 1.0,
      **{f"BYTE{index}": 0xFF for index in range(9, 24)},
    }
    msg = create_angle_steering(self.packer, self.CAN, 7, True, 12.3, 0.6, template)
    address, dat, bus = msg

    self.assertEqual((address, bus, len(dat)), (0xCB, self.CAN.ECAN, 24))
    self.assertEqual(dat[2], 7)
    self.assertEqual(dat[3], 0x20)
    self.assertEqual(int.from_bytes(dat[4:6], "little") & 0x3FFF, 123)
    self.assertEqual(dat[6], 150)
    self.assertEqual(dat[7], 0xFF)
    self.assertEqual(dat[8], 250)
    self.assertEqual(dat[9:], bytes([0xFF]) * 15)
    self.assert_valid_crc(msg)

  def test_inactive_command_uses_stock_inactive_state(self):
    msg = create_angle_steering(self.packer, self.CAN, 0x105, False, -17.5, 0.8, {})
    _, dat, _ = msg

    self.assertEqual(dat[2], 5)
    self.assertEqual(dat[3], 0x10)
    raw_angle = int.from_bytes(dat[4:6], "little") & 0x3FFF
    self.assertEqual(raw_angle - (1 << 14), -175)
    self.assertEqual(dat[6:], bytes(18))
    self.assert_valid_crc(msg)

  def test_controller_active_command_is_bounded_inside_safety_boundary(self):
    apply_angle = 0.0
    for _ in range(100):
      apply_angle, active = apply_hyundai_angle_limits(120.0, apply_angle, 0.0, apply_angle, True, True)
      self.assertTrue(active)
      self.assertLessEqual(apply_angle, 59.9)

    apply_angle, active = apply_hyundai_angle_limits(120.0, apply_angle, 0.0, 60.0, True, True)
    self.assertFalse(active)
    self.assertEqual(apply_angle, 60.0)

    apply_angle, active = apply_hyundai_angle_limits(-120.0, apply_angle, 0.0, 0.0, True, False)
    self.assertFalse(active)
    self.assertEqual(apply_angle, 0.0)

  def test_controller_tracks_measurement_inside_safety_error(self):
    apply_angle, active = apply_hyundai_angle_limits(50.0, 40.0, 15.0, 20.0, True, True)
    self.assertTrue(active)
    self.assertEqual(apply_angle, 28.0)

  def test_vehicle_model_limits_high_speed_angle_and_tracks_measurement_when_inactive(self):
    CP = CarParams.new_message()
    CP.wheelbase = 3.13
    CP.centerToFront = 1.35
    CP.rotationalInertiaPerMass = 2.0
    CP.tireStiffnessFrontPerMass = 45.0
    CP.tireStiffnessRearPerMass = 55.0
    CP.steerRatio = 15.5
    VM = VehicleModel(CP)

    static_angle, active = apply_hyundai_angle_limits(50.0, 40.0, 35.0, 40.0, True, True)
    model_angle, model_active = apply_hyundai_angle_limits(50.0, 40.0, 35.0, 40.0, True, True, VM)
    self.assertTrue(active and model_active)
    self.assertLess(model_angle, static_angle)

    inactive_angle, inactive = apply_hyundai_angle_limits(-50.0, model_angle, 35.0, 17.0, True, False, VM)
    self.assertFalse(inactive)
    self.assertEqual(inactive_angle, 17.0)

  def test_driver_override_handover_requires_quiet_release_window(self):
    allowed, latched, frames = update_hyundai_angle_handover(True, False, ANGLE_DRIVER_TORQUE_LIMIT + 1, False, 0)
    self.assertFalse(allowed)
    self.assertTrue(latched)
    for _ in range(ANGLE_DRIVER_TORQUE_RELEASE_FRAMES - 1):
      allowed, latched, frames = update_hyundai_angle_handover(True, False, 0.0, latched, frames)
      self.assertFalse(allowed)
    allowed, latched, frames = update_hyundai_angle_handover(True, False, 0.0, latched, frames)
    self.assertTrue(allowed)
    self.assertFalse(latched)
    self.assertEqual(frames, 0)

  def test_angle_torque_reduction_gain_yields_faster_than_recovery(self):
    gain = 0.0
    gain = update_hyundai_angle_torque_reduction_gain(0.0, 12.0, True, gain)
    self.assertEqual(gain, 0.004)
    for _ in range(249):
      gain = update_hyundai_angle_torque_reduction_gain(0.0, 12.0, True, gain)
    self.assertLessEqual(gain, 0.85)

    reduced = update_hyundai_angle_torque_reduction_gain(700.0, 12.0, True, gain)
    self.assertLess(reduced, gain)
    self.assertGreaterEqual(gain - reduced, 0.012)
    self.assertEqual(update_hyundai_angle_torque_reduction_gain(0.0, 12.0, False, 0.004), 0.0)

  def test_angle_torque_output_reports_transmitted_authority(self):
    self.assertEqual(get_hyundai_steering_output_can(True, True, 0.6, 123), 150)
    self.assertEqual(get_hyundai_steering_output_can(True, False, 0.6, 123), 0)
    self.assertEqual(get_hyundai_steering_output_can(False, False, 0.6, 123), 123)

  def test_angle_lateral_authority_follows_panda_safety(self):
    # stock longitudinal: stock cruise, or always-on lateral; openpilot longitudinal: always
    for lat_active in (False, True):
      for op_long in (False, True):
        for cruise in (False, True):
          for always_on in (False, True):
            expected = lat_active and (op_long or cruise or always_on)
            self.assertEqual(expected, hyundai_angle_lateral_authorized(lat_active, op_long, cruise, always_on))

  def test_always_on_lateral_hud_and_authority(self):
    from opendbc.safety import ALTERNATIVE_EXPERIENCE
    CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, TestHyundaiFingerprint._ioniq9_fingerprint(), TestHyundaiFingerprint._ioniq9_fw(),
                                 False, False, False)
    self.assertFalse(CarController(DBC[CP.carFingerprint], CP).always_on_lateral)
    CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ALWAYS_ON_LATERAL
    controller = CarController(DBC[CP.carFingerprint], CP)
    self.assertTrue(controller.always_on_lateral)
    for enabled, lat_active, shown in ((False, False, False), (False, True, True), (True, False, True), (True, True, True)):
      self.assertEqual(shown, controller.lateral_hud_enabled(SimpleNamespace(enabled=enabled, latActive=lat_active)))
    controller.always_on_lateral = False
    self.assertFalse(controller.lateral_hud_enabled(SimpleNamespace(enabled=False, latActive=True)))

  def test_angle_engage_waits_for_stable_authority(self):
    frames = 0
    for _ in range(ANGLE_ENGAGE_GRACE_FRAMES - 1):
      ready, frames = update_hyundai_angle_engage(True, frames)
      self.assertFalse(ready)
    ready, frames = update_hyundai_angle_engage(True, frames)
    self.assertTrue(ready)
    ready, frames = update_hyundai_angle_engage(False, frames)
    self.assertFalse(ready)
    self.assertEqual(frames, 0)

  def test_camera_mdps_tracks_stock_lfa_request_and_preserves_faults(self):
    template = {
      "COUNTER": 7,
      "MDPS_LkaToiActvSta": 1,
      "MDPS_LkaToiUnblSta": 0,
      "MDPS_ADASAciActvSta_Lv2": 2,
      "MDPS_ADAS_AciFltSig_Lv2": 0,
    }
    address, dat, bus = create_angle_camera_mdps(self.packer, self.CAN, template, {"ADAS_ActvACILvl2Sta": 1})
    self.assertEqual((address, bus, len(dat)), (0xEA, self.CAN.CAM, 24))
    self.assertEqual(dat[2], 0)
    self.assertEqual(dat[6] & 0xF, 0x4)
    self.assertEqual(dat[18] & 0xF, 1)
    self.assertEqual((dat[18] >> 4) & 0x7, 4)
    self.assert_valid_crc((address, dat, bus))

    _, fault_dat, _ = create_angle_camera_mdps(self.packer, self.CAN,
                                               template | {"MDPS_ADAS_AciFltSig_Lv2": 1, "MDPS_ADASAciActvSta_Lv2": 3},
                                               {"ADAS_ActvACILvl2Sta": 2})
    self.assertEqual((fault_dat[18] >> 4) & 0x7, 1)
    self.assertEqual(fault_dat[18] & 0xF, 3)

    _, stock_active, _ = create_angle_camera_mdps(self.packer, self.CAN, template | {"MDPS_ADASAciActvSta_Lv2": 1},
                                                  {"ADAS_ActvACILvl2Sta": 2})
    self.assertEqual(stock_active[18] & 0xF, 2)

  def test_stock_scc_template_preserves_non_actuation_fields(self):
    class HudControl:
      leadDistanceBars = 3

    stock = {
      "COUNTER": 0x7E,
      "ACC_ObjDist": 42.3,
      "ACC_ObjRelSpd": -1.2,
      "ObjValid": 1,
      "SCC_ObjSta": 5,
      "OBJ_STATUS": 6,
      "NEW_SIGNAL_15": 12.7,
      "ZEROS": 0x123456789ABC,
      "NEW_SIGNAL_8": 9,
    }
    msg = create_acc_control_from_stock(self.packer, self.CAN, True, 0.0, 0.2, False, False, 87, HudControl(), stock)
    address, dat, bus = msg

    self.assertEqual((address, bus, len(dat)), (0x1A0, self.CAN.ECAN, 32))
    self.assertEqual(dat[2], 0x7F)
    self.assert_valid_crc(msg)

    # Repacking the stock template must preserve every DBC-defined field other
    # than checksum, counter, and the explicit longitudinal actuation allowlist.
    stock_msg = self.packer.make_can_msg("SCC_CONTROL", self.CAN.ECAN, stock)
    _, stock_dat, _ = stock_msg
    dbc_msg = self.packer.dbc.name_to_msg["SCC_CONTROL"]
    mutable_signals = {"CHECKSUM", "COUNTER", "ACCMode", "MainMode_ACC", "StopReq", "aReqValue", "aReqRaw", "VSetDis",
                       "JerkLowerLimit", "JerkUpperLimit", "DISTANCE_SETTING"}
    for name, signal in dbc_msg.sigs.items():
      if name not in mutable_signals:
        self.assertEqual(signal.get_raw_value(dat), signal.get_raw_value(stock_dat), name)


class TestHyundaiFingerprint(unittest.TestCase):
  @staticmethod
  def _ioniq9_fw():
    return [
      CarParams.CarFw(ecu=ecu, address=address, subAddress=0, fwVersion=versions[0])
      for (ecu, address, _), versions in FW_VERSIONS[CAR.HYUNDAI_IONIQ_9].items()
    ]

  @staticmethod
  def _ioniq9_fingerprint():
    fingerprint = gen_empty_fingerprint()
    CAN = CanBus(None, fingerprint)
    configs = (CAR.HYUNDAI_IONIQ_9.config.steering_config, CAR.HYUNDAI_IONIQ_9.config.longitudinal_config)
    for config in configs:
      for requirement in config.required_messages:
        fingerprint[getattr(CAN, requirement.role.value)][requirement.address] = requirement.dlc
    return fingerprint

  def test_feature_detection(self):
    # LKA steering
    for lka_steering in (True, False):
      fingerprint = gen_empty_fingerprint()
      if lka_steering:
        cam_can = CanBus(None, fingerprint).CAM
        fingerprint[cam_can] = [0x50, 0x110]  # LKA steering messages
      CP = CarInterface.get_params(CAR.KIA_EV6, fingerprint, [], False, False, False)
      assert bool(CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG) == lka_steering

    # radar available
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_angle_steering_capability_is_data_driven_and_lateral_only(self):
    fingerprint = self._ioniq9_fingerprint()
    CAN = CanBus(None, fingerprint)
    steering_config = CAR.HYUNDAI_IONIQ_9.config.steering_config
    stock_bus = getattr(CAN, steering_config.stock_bus.value)
    car_fw = self._ioniq9_fw()

    # Fingerprinting supplies a string in production. Without the user's alpha
    # longitudinal preference the car stays lateral-only (stock ACC owns speed).
    CP = CarInterface.get_params(str(CAR.HYUNDAI_IONIQ_9), fingerprint, car_fw, False, False, False)
    assert CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING
    assert not CP.dashcamOnly
    assert CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.hyundaiCanfd
    assert CP.safetyConfigs[0].safetyParam & HyundaiSafetyFlags.CANFD_ANGLE_STEERING
    assert not CP.safetyConfigs[0].safetyParam & HyundaiSafetyFlags.LONG
    # Longitudinal is offered (2026-11-27 unlock) but not active without opt-in.
    assert CP.alphaLongitudinalAvailable
    assert not CP.openpilotLongitudinalControl
    assert CP.pcmCruise

    # With the opt-in, the exact capability enables openpilot longitudinal
    # (2026-11-27 unlock; previously this asserted it stayed unavailable).
    CP = CarInterface.get_params(str(CAR.HYUNDAI_IONIQ_9), fingerprint, car_fw, True, False, False)
    assert steering_config.api == HyundaiCanFDSteeringApi.ANGLE_LFA_ALT
    assert CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING
    assert CP.flags & HyundaiFlags.CANFD_LONGITUDINAL_CANDIDATE
    assert CP.steerControlType == CarParams.SteerControlType.angle
    assert not CP.dashcamOnly
    assert CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.hyundaiCanfd
    assert CP.safetyConfigs[0].safetyParam & HyundaiSafetyFlags.CANFD_ANGLE_STEERING
    assert CP.safetyConfigs[0].safetyParam & HyundaiSafetyFlags.LONG
    assert CP.alphaLongitudinalAvailable
    assert CP.openpilotLongitudinalControl
    assert not CP.pcmCruise

    # Source gate and release builds remain fail-closed even with exact evidence.
    with patch("opendbc.car.hyundai.interface.ENABLE_CANFD_LATERAL_STAGING", False):
      gated_CP = CarInterface.get_params(str(CAR.HYUNDAI_IONIQ_9), fingerprint, car_fw, False, False, False)
    assert gated_CP.dashcamOnly
    assert gated_CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.noOutput

    release_CP = CarInterface.get_params(str(CAR.HYUNDAI_IONIQ_9), fingerprint, car_fw, False, True, False)
    assert release_CP.dashcamOnly
    assert release_CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.noOutput

    longitudinal_config = CAR.HYUNDAI_IONIQ_9.config.longitudinal_config
    assert longitudinal_config.harness_connection == HyundaiCanFDHarnessConnection.ADAS
    assert longitudinal_config.ecu_disable_strategy == HyundaiCanFDEcuDisableStrategy.NONE
    assert longitudinal_config.stock_aeb_fcw_status == HyundaiCanFDStockSafetyStatus.UNVERIFIED
    assert longitudinal_config.command_address == 0x1A0
    assert longitudinal_config.command_dlc == 32
    assert longitudinal_config.command_frequency_hz == 50

    # Seeing LFA_ALT is not sufficient to select angle control for a torque profile.
    torque_CP = CarInterface.get_params(CAR.KIA_EV6, fingerprint, car_fw, False, False, False)
    assert not torque_CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING
    assert torque_CP.steerControlType == CarParams.SteerControlType.torque

    # The configured address must also have the observed DLC before the capability is exposed.
    fingerprint[stock_bus][steering_config.command_address] = 16
    mismatched_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, car_fw, False, False, False)
    assert not mismatched_CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING
    assert mismatched_CP.dashcamOnly
    assert mismatched_CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.noOutput

    # The same address and DLC on a different logical bus is not sufficient.
    fingerprint = gen_empty_fingerprint()
    wrong_bus = CAN.ECAN if stock_bus != CAN.ECAN else CAN.CAM
    fingerprint[wrong_bus][steering_config.command_address] = steering_config.command_dlc
    wrong_bus_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, car_fw, False, False, False)
    assert not wrong_bus_CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING
    assert wrong_bus_CP.dashcamOnly
    assert wrong_bus_CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.noOutput

    # A complete passive topology without all exact firmware cannot authorize output.
    no_fw_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, self._ioniq9_fingerprint(), [], False, False, False)
    assert not no_fw_CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING
    assert no_fw_CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.noOutput

    wrong_fw = self._ioniq9_fw()
    wrong_fw[0].fwVersion = b'\xf1\x00unexpected'
    wrong_fw_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, self._ioniq9_fingerprint(), wrong_fw, False, False, False)
    assert not wrong_fw_CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING
    assert wrong_fw_CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.noOutput

    for requirement in steering_config.required_firmware:
      wrong_fw = self._ioniq9_fw()
      target = next(fw for fw in wrong_fw if fw.ecu.raw == requirement.ecu and fw.address == requirement.address)
      target.fwVersion = b'\xf1\x00unexpected'
      wrong_fw_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, self._ioniq9_fingerprint(), wrong_fw, True, False, False)
      assert not wrong_fw_CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING
      assert wrong_fw_CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.noOutput

    for requirement in steering_config.required_messages:
      requirement_bus = getattr(CAN, requirement.role.value)

      missing = copy.deepcopy(self._ioniq9_fingerprint())
      del missing[requirement_bus][requirement.address]
      missing_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, missing, car_fw, True, False, False)
      assert not missing_CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING
      assert missing_CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.noOutput

      wrong_dlc = copy.deepcopy(self._ioniq9_fingerprint())
      wrong_dlc[requirement_bus][requirement.address] = requirement.dlc - 1
      wrong_dlc_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, wrong_dlc, car_fw, True, False, False)
      assert not wrong_dlc_CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING
      assert wrong_dlc_CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.noOutput

      wrong_bus_fingerprint = copy.deepcopy(self._ioniq9_fingerprint())
      del wrong_bus_fingerprint[requirement_bus][requirement.address]
      other_bus = CAN.CAM if requirement_bus != CAN.CAM else CAN.ECAN
      wrong_bus_fingerprint[other_bus][requirement.address] = requirement.dlc
      wrong_bus_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, wrong_bus_fingerprint, car_fw, True, False, False)
      assert not wrong_bus_CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING
      assert wrong_bus_CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.noOutput

    # Missing observations still enumerate the platform, but can never authorize output.
    empty_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, gen_empty_fingerprint(), [], False, False, False)
    assert not empty_CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING
    assert not empty_CP.flags & HyundaiFlags.CANFD_LONGITUDINAL_CANDIDATE
    assert empty_CP.dashcamOnly
    assert empty_CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.noOutput

  def test_longitudinal_candidate_requires_adas_harness_topology_and_firmware(self):
    config = CAR.HYUNDAI_IONIQ_9.config.longitudinal_config
    fingerprint = self._ioniq9_fingerprint()
    car_fw = self._ioniq9_fw()
    CAN = CanBus(None, fingerprint)

    CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, car_fw, True, False, False)
    assert CP.flags & HyundaiFlags.CANFD_LONGITUDINAL_CANDIDATE
    assert CP.flags & HyundaiFlags.CANFD_CAMERA_SCC
    # 2026-11-27 unlock: the exact candidate is now available and opt-in active.
    assert CP.alphaLongitudinalAvailable
    assert CP.openpilotLongitudinalControl
    assert CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.LONG

    # 0x12A is specific to the observed ADAS-harness topology. Losing it must
    # block the longitudinal candidate without weakening the angle capability.
    no_adas_topology = copy.deepcopy(fingerprint)
    del no_adas_topology[CAN.CAM][0x12A]
    no_adas_topology_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, no_adas_topology, car_fw, True, False, False)
    assert no_adas_topology_CP.flags & HyundaiFlags.CANFD_ANGLE_STEERING
    assert not no_adas_topology_CP.flags & HyundaiFlags.CANFD_LONGITUDINAL_CANDIDATE
    assert not no_adas_topology_CP.openpilotLongitudinalControl

    for requirement in config.required_messages:
      missing = copy.deepcopy(fingerprint)
      del missing[getattr(CAN, requirement.role.value)][requirement.address]
      missing_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, missing, car_fw, True, False, False)
      assert not missing_CP.flags & HyundaiFlags.CANFD_LONGITUDINAL_CANDIDATE
      assert not missing_CP.openpilotLongitudinalControl
      assert not missing_CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.LONG

    for requirement in config.required_firmware:
      wrong_fw = self._ioniq9_fw()
      target = next(fw for fw in wrong_fw if fw.ecu.raw == requirement.ecu and fw.address == requirement.address)
      target.fwVersion = b'\xf1\x00unexpected'
      wrong_fw_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, wrong_fw, True, False, False)
      assert not wrong_fw_CP.flags & HyundaiFlags.CANFD_LONGITUDINAL_CANDIDATE
      assert not wrong_fw_CP.openpilotLongitudinalControl
      assert not wrong_fw_CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.LONG

  def _verified_longitudinal_profile(self):
    platform = CAR.HYUNDAI_IONIQ_9
    profile = replace(platform.config.longitudinal_config, stock_aeb_fcw_status=HyundaiCanFDStockSafetyStatus.PRESERVED)
    return patch.object(platform, "config", replace(platform.config, longitudinal_config=profile))

  def test_unverified_stock_aeb_profile_is_opt_in_only(self):
    # 2026-11-27: replaces test_unverified_longitudinal_profile_stays_unavailable.
    # Stock AEB/FCW preservation is still unverified and recorded as such, but it
    # no longer blocks the user's explicit alpha longitudinal choice.
    config = CAR.HYUNDAI_IONIQ_9.config.longitudinal_config
    assert config.stock_aeb_fcw_status == HyundaiCanFDStockSafetyStatus.UNVERIFIED
    fingerprint = self._ioniq9_fingerprint()
    car_fw = self._ioniq9_fw()

    off_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, car_fw, False, False, False)
    assert off_CP.flags & HyundaiFlags.CANFD_LONGITUDINAL_CANDIDATE
    assert off_CP.alphaLongitudinalAvailable
    assert not off_CP.openpilotLongitudinalControl
    assert off_CP.pcmCruise
    assert not off_CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.LONG

    CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, car_fw, True, False, False)
    assert CP.alphaLongitudinalAvailable
    assert CP.openpilotLongitudinalControl
    assert not CP.pcmCruise
    assert CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.LONG

    # The source gate still closes it again.
    with patch("opendbc.car.hyundai.interface.ENABLE_CANFD_LONGITUDINAL_STAGING", False):
      gated_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, car_fw, True, False, False)
    assert not gated_CP.alphaLongitudinalAvailable
    assert not gated_CP.openpilotLongitudinalControl
    assert gated_CP.pcmCruise
    assert not gated_CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.LONG

    # A profile that would disable a stock ECU is never offered by this path.
    platform = CAR.HYUNDAI_IONIQ_9
    disabling = replace(platform.config.longitudinal_config, ecu_disable_strategy=HyundaiCanFDEcuDisableStrategy.ADAS)
    with patch.object(platform, "config", replace(platform.config, longitudinal_config=disabling)):
      disabling_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, car_fw, True, False, False)
    assert not disabling_CP.alphaLongitudinalAvailable
    assert not disabling_CP.openpilotLongitudinalControl

  def test_unlocked_longitudinal_keeps_stock_ecus_and_sends_scc_control(self):
    # Default (unpatched) profile: opt-in longitudinal never disables the stock
    # ADAS/radar ECUs and only replaces SCC_CONTROL on ECAN.
    CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, self._ioniq9_fingerprint(), self._ioniq9_fw(), True, False, False)
    assert CP.openpilotLongitudinalControl
    assert CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CAMERA_SCC
    assert CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_ANGLE_STEERING
    with patch("opendbc.car.hyundai.interface.disable_ecu") as disable_ecu:
      CarInterface.init(CP, None, None)
      CarInterface.deinit(CP, None, None)
    disable_ecu.assert_not_called()

    controller = CarController(DBC[CP.carFingerprint], CP)
    CS = SimpleNamespace(
      out=SimpleNamespace(steeringPressed=False, steeringAngleDeg=0.0),
      lfa_alt={},
      mdps={"COUNTER": 0, "MDPS_ADASAciActvSta_Lv2": 1, "MDPS_ADAS_AciFltSig_Lv2": 0},
      cruise_info={"COUNTER": 0x20},
    )
    hud_control = SimpleNamespace(leadDistanceBars=2)
    CC = SimpleNamespace(enabled=True, latActive=True, cruiseControl=SimpleNamespace(override=False))
    controller.frame = 0
    msgs = controller.create_canfd_msgs(True, 0, 80, 0.1, False, hud_control, CS, CC)
    self.assertEqual([(address, bus, len(dat)) for address, dat, bus in msgs],
                     [(0xCB, controller.CAN.ECAN, 24), (0xEA, controller.CAN.CAM, 24),
                      (0x1A0, controller.CAN.ECAN, 32)])
    # No tester-present / ECU-disable keepalive is sent either.
    assert all(address not in (0x730, 0x7D0) for address, _, _ in msgs)

    # Release builds stay fail-closed.
    release_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, self._ioniq9_fingerprint(), self._ioniq9_fw(), True, True, False)
    assert not release_CP.alphaLongitudinalAvailable
    assert not release_CP.openpilotLongitudinalControl

  def test_longitudinal_staging_requires_source_gate_and_exact_capability(self):
    fingerprint = self._ioniq9_fingerprint()
    car_fw = self._ioniq9_fw()

    # 2026-11-27: the source gate now defaults to on, so the closed case is
    # exercised by patching it off instead of relying on the default.
    with self._verified_longitudinal_profile(), patch("opendbc.car.hyundai.interface.ENABLE_CANFD_LONGITUDINAL_STAGING", False):
      default_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, car_fw, True, False, False)
      assert not default_CP.alphaLongitudinalAvailable
      assert not default_CP.openpilotLongitudinalControl

    with self._verified_longitudinal_profile(), patch("opendbc.car.hyundai.interface.ENABLE_CANFD_LONGITUDINAL_STAGING", True):
      off_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, car_fw, False, False, False)
      assert off_CP.alphaLongitudinalAvailable
      assert not off_CP.openpilotLongitudinalControl
      assert off_CP.pcmCruise
      assert not off_CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.LONG

      CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, car_fw, True, False, False)
      assert CP.alphaLongitudinalAvailable
      assert CP.openpilotLongitudinalControl
      assert not CP.pcmCruise
      assert CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.LONG
      assert CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_ANGLE_STEERING

      release_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, car_fw, True, True, False)
      assert not release_CP.alphaLongitudinalAvailable
      assert not release_CP.openpilotLongitudinalControl
      assert release_CP.dashcamOnly

      # The source gate alone cannot authorize a missing passive capability.
      missing = copy.deepcopy(fingerprint)
      del missing[CanBus(None, fingerprint).CAM][0x12A]
      missing_CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, missing, car_fw, True, False, False)
      assert not missing_CP.alphaLongitudinalAvailable
      assert not missing_CP.openpilotLongitudinalControl
      assert missing_CP.pcmCruise
      assert not missing_CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.LONG

  def test_longitudinal_profile_preserves_stock_ecu_during_init_and_deinit(self):
    with self._verified_longitudinal_profile(), patch("opendbc.car.hyundai.interface.ENABLE_CANFD_LONGITUDINAL_STAGING", True):
      CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, self._ioniq9_fingerprint(), self._ioniq9_fw(), True, False, False)
      assert CP.openpilotLongitudinalControl
      CP.flags &= ~int(HyundaiFlags.CANFD_CAMERA_SCC | HyundaiFlags.CAMERA_SCC)
      with patch("opendbc.car.hyundai.interface.disable_ecu") as disable_ecu:
        CarInterface.init(CP, None, None)
        CarInterface.deinit(CP, None, None)
      disable_ecu.assert_not_called()

  def test_longitudinal_staging_output_topology(self):
    with self._verified_longitudinal_profile(), patch("opendbc.car.hyundai.interface.ENABLE_CANFD_LONGITUDINAL_STAGING", True):
      CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, self._ioniq9_fingerprint(), self._ioniq9_fw(), True, False, False)

    controller = CarController(DBC[CP.carFingerprint], CP)
    CS = SimpleNamespace(
      out=SimpleNamespace(steeringPressed=False, steeringAngleDeg=0.0),
      lfa_alt={},
      mdps={"COUNTER": 0, "MDPS_ADASAciActvSta_Lv2": 1, "MDPS_ADAS_AciFltSig_Lv2": 0},
      cruise_info={"COUNTER": 0x20},
    )
    hud_control = SimpleNamespace(leadDistanceBars=2)
    CC = SimpleNamespace(
      enabled=True,
      latActive=True,
      cruiseControl=SimpleNamespace(override=False),
    )

    controller.frame = 0
    controller.angle_active = True
    frame_zero = controller.create_canfd_msgs(True, 0, 80, 0.1, False, hud_control, CS, CC)
    self.assertEqual([(address, bus, len(dat)) for address, dat, bus in frame_zero],
                     [(0xCB, controller.CAN.ECAN, 24), (0xEA, controller.CAN.CAM, 24),
                      (0x1A0, controller.CAN.ECAN, 32)])
    self.assertEqual(frame_zero[2][1][2], 0x21)

    controller.frame = 1
    frame_one = controller.create_canfd_msgs(True, 0, 80, 0.1, False, hud_control, CS, CC)
    self.assertEqual([(address, bus, len(dat)) for address, dat, bus in frame_one],
                     [(0xCB, controller.CAN.ECAN, 24), (0xEA, controller.CAN.CAM, 24)])

    controller.frame = 2
    frame_two = controller.create_canfd_msgs(True, 0, 80, 0.1, False, hud_control, CS, CC)
    self.assertEqual([(address, bus, len(dat)) for address, dat, bus in frame_two],
                     [(0xCB, controller.CAN.ECAN, 24), (0xEA, controller.CAN.CAM, 24),
                      (0x1A0, controller.CAN.ECAN, 32)])
    self.assertEqual(frame_two[2][1][2], 0x22)

  def test_ioniq9_bursty_bsm_does_not_gate_can_validity(self):
    fingerprint = self._ioniq9_fingerprint()
    CAN = CanBus(None, fingerprint)
    fingerprint[CAN.ECAN][0x1BA] = 32
    CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_9, fingerprint, self._ioniq9_fw(), True, False, False)

    pt_parser = CarState(CP).get_can_parsers(CP)[Bus.pt]
    assert pt_parser.message_states[0x1BA].ignore_alive

  def test_angle_steering_config_is_complete_and_passive(self):
    for platform in CAR:
      steering_config = platform.config.steering_config if hasattr(platform.config, "steering_config") else None
      if steering_config is None or steering_config.api != HyundaiCanFDSteeringApi.ANGLE_LFA_ALT:
        continue

      assert steering_config.command_address is not None
      assert steering_config.command_dlc is not None
      assert steering_config.stock_bus is not None
      assert steering_config.command_bus is not None
      assert steering_config.measured_angle_signal is not None  # metadata-only until the parser path is implemented
      assert steering_config.required_messages
      assert steering_config.required_firmware

      CP = CarInterface.get_params(str(platform), gen_empty_fingerprint(), [], True, False, False)
      assert CP.dashcamOnly
      assert not CP.alphaLongitudinalAvailable
      assert not CP.openpilotLongitudinalControl
      assert len(CP.safetyConfigs) == 1
      assert CP.safetyConfigs[0].safetyModel == CarParams.SafetyModel.noOutput

  def test_alternate_limits(self):
    # Alternate lateral control limits, for high torque cars, verify Panda safety mode flag is set
    fingerprint = gen_empty_fingerprint()
    for car_model in CAR:
      CP = CarInterface.get_params(car_model, fingerprint, [], False, False, False)
      assert bool(CP.flags & HyundaiFlags.ALT_LIMITS) == bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.ALT_LIMITS)

  def test_can_features(self):
    for car_model in CAR:
      flags = car_model.config.flags
      # Test no EV/HEV with cluster/TCU gears (should all use ELECT_GEAR)
      if flags & (HyundaiFlags.HYBRID | HyundaiFlags.EV):
        assert not (flags & (HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.TCU_GEARS))

      # Test CAN FD car not in CAN feature lists
      if flags & HyundaiFlags.CANFD:
        assert not (flags & CAN_FEATURE_FLAGS), "CAN FD car unexpectedly has a CAN feature flag"

  def test_hybrid_ev_flags(self):
    for car_model in CAR:
      flags = car_model.config.flags
      assert not (flags & HyundaiFlags.HYBRID and flags & HyundaiFlags.EV), "Shared cars between hybrid and EV"
      assert not (flags & HyundaiFlags.CANFD and flags & HyundaiFlags.HYBRID), \
        "Hard coding CAN FD cars as hybrid is no longer supported"

  def test_canfd_ecu_whitelist(self):
    # Asserts only expected Ecus can exist in database for CAN-FD cars
    for car_model in CAR:
      if not (car_model.config.flags & HyundaiFlags.CANFD):
        continue
      ecus = {fw[0] for fw in FW_VERSIONS[car_model].keys()}
      ecus_not_in_whitelist = ecus - CANFD_EXPECTED_ECUS
      ecu_strings = ", ".join([f"Ecu.{ecu}" for ecu in ecus_not_in_whitelist])
      assert len(ecus_not_in_whitelist) == 0, \
                       f"{car_model}: Car model has unexpected ECUs: {ecu_strings}"

  def test_blacklisted_parts(self):
    # Asserts no ECUs known to be shared across platforms exist in the database.
    # Tucson having Santa Cruz camera and EPS for example
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        if car_model == CAR.HYUNDAI_SANTA_CRUZ_1ST_GEN:
          raise unittest.SkipTest("Skip checking Santa Cruz for its parts")

        for code, _ in get_platform_codes(ecus[(Ecu.fwdCamera, 0x7c4, None)]):
          if b"-" not in code:
            continue
          part = code.split(b"-")[1]
          assert not part.startswith(b'CW'), "Car has bad part number"

  def test_correct_ecu_response_database(self):
    """
    Assert standard responses for certain ECUs, since they can
    respond to multiple queries with different data
    """
    expected_fw_prefix = HYUNDAI_VERSION_REQUEST_LONG[1:]
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          assert all(fw.startswith(expected_fw_prefix) for fw in fws), \
                          f"FW from unexpected request in database: {(ecu, fws)}"

  @fuzzy_test(max_examples=100)
  def test_platform_codes_fuzzy_fw(self, fuzzy):
    """Ensure function doesn't raise an exception"""
    get_platform_codes(fuzzy.list(fuzzy.binary))

  def test_expected_platform_codes(self):
    # Ensures we don't accidentally add multiple platform codes for a car unless it is intentional
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          if ecu[0] not in PLATFORM_CODE_ECUS:
            continue

          # Third and fourth character are usually EV/hybrid identifiers
          codes = {code.split(b"-")[0][:2] for code, _ in get_platform_codes(fws)}
          if car_model == CAR.HYUNDAI_PALISADE:
            assert codes == {b"LX", b"ON"}, f"Car has unexpected platform codes: {car_model} {codes}"
          elif car_model == CAR.HYUNDAI_KONA_EV and ecu[0] == Ecu.fwdCamera:
            assert codes == {b"OE", b"OS"}, f"Car has unexpected platform codes: {car_model} {codes}"
          else:
            assert len(codes) == 1, f"Car has multiple platform codes: {car_model} {codes}"

  # Tests for platform codes, part numbers, and FW dates which Hyundai will use to fuzzy
  # fingerprint in the absence of full FW matches:
  def test_platform_code_ecus_available(self):
    # TODO: add queries for these non-CAN FD cars to get EPS
    no_eps_platforms = cars_with(HyundaiFlags.CANFD) | {CAR.KIA_SORENTO, CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL, CAR.KIA_OPTIMA_H, CAR.KIA_K7_2017,
                                    CAR.KIA_OPTIMA_H_G4_FL, CAR.HYUNDAI_SONATA_LF, CAR.HYUNDAI_TUCSON, CAR.GENESIS_G90, CAR.GENESIS_G80, CAR.HYUNDAI_ELANTRA}

    # Asserts ECU keys essential for fuzzy fingerprinting are available on all platforms
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for platform_code_ecu in PLATFORM_CODE_ECUS:
          if platform_code_ecu in (Ecu.fwdRadar, Ecu.eps) and car_model == CAR.HYUNDAI_GENESIS:
            continue
          if platform_code_ecu == Ecu.eps and car_model in no_eps_platforms:
            continue
          assert platform_code_ecu in [e[0] for e in ecus]

  def test_fw_format(self):
    # Asserts:
    # - every supported ECU FW version returns one platform code
    # - every supported ECU FW version has a part number
    # - expected parsing of ECU FW dates

    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          if ecu[0] not in PLATFORM_CODE_ECUS:
            continue

          codes = set()
          for fw in fws:
            result = get_platform_codes([fw])
            assert 1 == len(result), f"Unable to parse FW: {fw}"
            codes |= result

          if ecu[0] not in DATE_FW_ECUS or car_model in NO_DATES_PLATFORMS:
            assert all(date is None for _, date in codes)
          else:
            assert all(date is not None for _, date in codes)

          if car_model == CAR.HYUNDAI_GENESIS:
            raise unittest.SkipTest("No part numbers for car model")

          # Hyundai places the ECU part number in their FW versions, assert all parsable
          # Some examples of valid formats: b"56310-L0010", b"56310L0010", b"56310/M6300"
          assert all(b"-" in code for code, _ in codes), \
                          f"FW does not have part number: {fw}"

  def test_platform_codes_spot_check(self):
    # Asserts basic platform code parsing behavior for a few cases
    results = get_platform_codes([b"\xf1\x00DH LKAS 1.1 -150210"])
    assert results == {(b"DH", b"150210")}

    # Some cameras and all radars do not have dates
    results = get_platform_codes([b"\xf1\x00AEhe SCC H-CUP      1.01 1.01 96400-G2000         "])
    assert results == {(b"AEhe-G2000", None)}

    results = get_platform_codes([b"\xf1\x00CV1_ RDR -----      1.00 1.01 99110-CV000         "])
    assert results == {(b"CV1-CV000", None)}

    results = get_platform_codes([
      b"\xf1\x00DH LKAS 1.1 -150210",
      b"\xf1\x00AEhe SCC H-CUP      1.01 1.01 96400-G2000         ",
      b"\xf1\x00CV1_ RDR -----      1.00 1.01 99110-CV000         ",
    ])
    assert results == {(b"DH", b"150210"), (b"AEhe-G2000", None), (b"CV1-CV000", None)}

    results = get_platform_codes([
      b"\xf1\x00LX2 MFC  AT USA LHD 1.00 1.07 99211-S8100 220222",
      b"\xf1\x00LX2 MFC  AT USA LHD 1.00 1.08 99211-S8100 211103",
      b"\xf1\x00ON  MFC  AT USA LHD 1.00 1.01 99211-S9100 190405",
      b"\xf1\x00ON  MFC  AT USA LHD 1.00 1.03 99211-S9100 190720",
    ])
    assert results == {(b"LX2-S8100", b"220222"), (b"LX2-S8100", b"211103"),
                               (b"ON-S9100", b"190405"), (b"ON-S9100", b"190720")}

  def test_fuzzy_excluded_platforms(self):
    # Asserts a list of platforms that will not fuzzy fingerprint with platform codes due to them being shared.
    # This list can be shrunk as we combine platforms and detect features
    excluded_platforms = {
      CAR.GENESIS_G70,            # shared platform code, part number, and date
      CAR.GENESIS_G70_2020,
    }
    excluded_platforms |= cars_with(HyundaiFlags.CANFD) - cars_with(HyundaiFlags.EV) - CANFD_FUZZY_WHITELIST  # shared platform codes
    excluded_platforms |= NO_DATES_PLATFORMS  # date codes are required to match

    platforms_with_shared_codes = set()
    for platform, fw_by_addr in FW_VERSIONS.items():
      car_fw = []
      for ecu, fw_versions in fw_by_addr.items():
        ecu_name, addr, sub_addr = ecu
        for fw in fw_versions:
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

      CP = CarParams(carFw=car_fw)
      matches = match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), CP.carVin, FW_VERSIONS)
      if len(matches) == 1:
        assert list(matches)[0] == platform
      else:
        platforms_with_shared_codes.add(platform)

    assert platforms_with_shared_codes == excluded_platforms
