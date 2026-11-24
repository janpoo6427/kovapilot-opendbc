import re
from dataclasses import dataclass, field
from enum import Enum, IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16
from opendbc.car.lateral import AngleSteeringLimits

Ecu = CarParams.Ecu


class CarControllerParams:
  ACCEL_MIN = -3.5 # m/s^2
  ACCEL_MAX = 2.0 # m/s^2
  ANGLE_LIMITS = AngleSteeringLimits(
    175.0,
    ([0.0, 5.0, 25.0], [1.2, 0.8, 0.2]),
    ([0.0, 5.0, 25.0], [1.5, 1.0, 0.3]),
  )

  def __init__(self, CP):
    self.STEER_DELTA_UP = 3
    self.STEER_DELTA_DOWN = 7
    self.STEER_DRIVER_ALLOWANCE = 50
    self.STEER_DRIVER_MULTIPLIER = 2
    self.STEER_DRIVER_FACTOR = 1
    self.STEER_THRESHOLD = 150
    self.STEER_STEP = 1  # 100 Hz

    if CP.flags & HyundaiFlags.CANFD:
      self.STEER_MAX = 270
      self.STEER_DRIVER_ALLOWANCE = 250
      self.STEER_DRIVER_MULTIPLIER = 2
      self.STEER_THRESHOLD = 250
      self.STEER_DELTA_UP = 2
      self.STEER_DELTA_DOWN = 3

    # To determine the limit for your car, find the maximum value that the stock LKAS will request.
    # If the max stock LKAS request is <384, add your car to this list.
    elif CP.carFingerprint in (CAR.GENESIS_G80, CAR.HYUNDAI_ELANTRA, CAR.HYUNDAI_ELANTRA_GT_I30, CAR.HYUNDAI_IONIQ,
                               CAR.HYUNDAI_IONIQ_EV_LTD, CAR.HYUNDAI_SANTA_FE_PHEV_2022, CAR.HYUNDAI_SONATA_LF, CAR.KIA_FORTE, CAR.KIA_NIRO_PHEV,
                               CAR.KIA_OPTIMA_H, CAR.KIA_OPTIMA_H_G4_FL, CAR.KIA_SORENTO):
      self.STEER_MAX = 255

    # these cars have significantly more torque than most HKG; limit to 70% of max
    elif CP.flags & HyundaiFlags.ALT_LIMITS:
      self.STEER_MAX = 270
      self.STEER_DELTA_UP = 2
      self.STEER_DELTA_DOWN = 3

    elif CP.flags & HyundaiFlags.ALT_LIMITS_2:
      self.STEER_MAX = 170
      self.STEER_DELTA_UP = 2
      self.STEER_DELTA_DOWN = 3

    # Default for most HKG
    else:
      self.STEER_MAX = 384


class HyundaiSafetyFlags(IntFlag):
  EV_GAS = 1
  HYBRID_GAS = 2
  LONG = 4
  CAMERA_SCC = 8
  CANFD_LKA_STEER_MSG = 16
  CANFD_ALT_BUTTONS = 32
  ALT_LIMITS = 64
  CANFD_LKA_STEER_MSG_ALT = 128
  FCEV_GAS = 256
  ALT_LIMITS_2 = 512
  CANFD_ANGLE_STEERING = 1024


# Hyundai/Kia/Genesis SCC (Smart Cruise Control) and steering architecture:
#
# CAN cars:
#   - Default: Radar sends SCC messages on bus 0. For longitudinal control, disable radar
#     (0x7d0) and send our own SCC messages. AEB is lost since radar is disabled.
#   - CAMERA_SCC: Camera sends SCC and AEB/FCA messages (all on bus 2). Block camera SCC
#     and send our own. Stock FCA/AEB messages from camera are forwarded through.
#
# CAN-FD cars:
#   - CANFD_LKA_STEER_MSG (typically HDA2 with ADAS ECU): Camera sends LKA steering
#     message, ADAS DRV ECU forwards it as LFA to MDPS. For longitudinal, disable ADAS
#     ECU (0x730) and send ACC on ECAN. OP also suppresses LFA from ADAS ECU to prevent
#     fighting. Some cars lack the ADAS ECU (long not available).
#   - Non-LKA (HDA1): Camera sends LFA steering directly to MDPS (no ADAS ECU).
#     + CANFD_CAMERA_SCC: Camera also sends SCC (no CANFD_RADAR_SCC flag set).
#       Block camera SCC and send our own.
#     + CANFD_RADAR_SCC: Radar handles SCC instead.
#       Disable radar (0x7d0 on ECAN) for longitudinal.
#   - CANFD_NO_RADAR_DISABLE: Some CAN-FD cars refuse the communication control disable
#     request (0x7F2822 'conditions not correct') — longitudinal not available.

class HyundaiFlags(IntFlag):
  # Dynamic Flags

  # Default assumption: all cars use LFA (ADAS) steering from the camera (HDA1).
  # CANFD_LKA_STEER_MSG/CANFD_LKA_STEER_MSG_ALT cars typically have both LKA (camera) and LFA (ADAS) steering messages,
  # with LKA commands forwarded to the ADAS DRV ECU.
  # Most HDA2 trims are assumed to be equipped with the ADAS DRV ECU, though some variants may not be equipped with one.
  CANFD_LKA_STEER_MSG = 1
  CANFD_ALT_BUTTONS = 2
  CANFD_ALT_GEARS = 2 ** 2
  CANFD_CAMERA_SCC = 2 ** 3

  ALT_LIMITS = 2 ** 4
  CANFD_ENABLE_BLINKERS = 2 ** 5
  CANFD_ALT_GEARS_2 = 2 ** 6
  SEND_LFA = 2 ** 7
  USE_FCA = 2 ** 8
  CANFD_LKA_STEER_MSG_ALT = 2 ** 9

  # these cars use a different gas signal
  HYBRID = 2 ** 10
  EV = 2 ** 11

  # Static flags

  # If 0x500 is present on bus 1 it probably has a Mando radar outputting radar points.
  # If no points are outputted by default it might be possible to turn it on using  selfdrive/debug/hyundai_enable_radar_points.py
  MANDO_RADAR = 2 ** 12
  CANFD = 2 ** 13

  # The radar does SCC on these cars when HDA I, rather than the camera
  CANFD_RADAR_SCC = 2 ** 14
  # The camera does SCC on these cars, rather than the radar
  CAMERA_SCC = 2 ** 15
  CHECKSUM_CRC8 = 2 ** 16
  CHECKSUM_6B = 2 ** 17

  # these cars require a special panda safety mode due to missing counters and checksums in the messages
  LEGACY = 2 ** 18

  # these cars have not been verified to work with longitudinal yet - radar disable, sending correct messages, etc.
  UNSUPPORTED_LONGITUDINAL = 2 ** 19

  # These CAN FD cars do not accept communication control to disable the ADAS ECU,
  # responds with 0x7F2822 - 'conditions not correct'
  CANFD_NO_RADAR_DISABLE = 2 ** 20

  CLUSTER_GEARS = 2 ** 21
  TCU_GEARS = 2 ** 22

  MIN_STEER_32_MPH = 2 ** 23

  HAS_LDA_BUTTON = 2 ** 24

  FCEV = 2 ** 25

  ALT_LIMITS_2 = 2 ** 26

  HAS_BSM = 2 ** 27  # blind spot monitoring

  # The platform uses the CAN-FD LFA_ALT angle steering API. This flag is set
  # only after the platform capability and observed CAN topology agree.
  CANFD_ANGLE_STEERING = 2 ** 28
  # Passive evidence matches a typed CAN-FD longitudinal profile. This is a
  # candidate flag only and does not enable longitudinal control or safety.
  CANFD_LONGITUDINAL_CANDIDATE = 2 ** 29


@dataclass
class HyundaiCarDocs(CarDocs):
  package: str = "Smart Cruise Control (SCC)"


@dataclass
class HyundaiPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: "hyundai_can_generated"})

  def init(self):
    if self.flags & HyundaiFlags.MANDO_RADAR:
      self.dbc_dict = {Bus.pt: "hyundai_can_generated", Bus.radar: 'hyundai_kia_mando_front_radar_generated'}

    if self.flags & HyundaiFlags.MIN_STEER_32_MPH:
      self.specs = self.specs.override(minSteerSpeed=32 * CV.MPH_TO_MS)


class HyundaiCanFDSteeringApi(Enum):
  TORQUE_AUTO = "torque_auto"
  ANGLE_LFA_ALT = "angle_lfa_alt"


class HyundaiCanFDBusRole(Enum):
  CAM = "CAM"
  ECAN = "ECAN"


class HyundaiCanFDHarnessConnection(Enum):
  UNKNOWN = "unknown"
  ADAS = "adas"


class HyundaiCanFDEcuDisableStrategy(Enum):
  UNCONFIGURED = "unconfigured"
  NONE = "none"
  RADAR = "radar"
  ADAS = "adas"


class HyundaiCanFDStockSafetyStatus(Enum):
  UNVERIFIED = "unverified"
  PRESERVED = "preserved"


@dataclass(frozen=True)
class HyundaiCanFDMessageRequirement:
  role: HyundaiCanFDBusRole
  address: int
  dlc: int


@dataclass(frozen=True)
class HyundaiFirmwareRequirement:
  ecu: CarParams.Ecu
  address: int
  version: bytes


@dataclass(frozen=True)
class HyundaiCanFDSteeringConfig:
  api: HyundaiCanFDSteeringApi = HyundaiCanFDSteeringApi.TORQUE_AUTO
  command_address: int | None = None
  command_dlc: int | None = None
  stock_bus: HyundaiCanFDBusRole | None = None
  command_bus: HyundaiCanFDBusRole | None = None
  measured_angle_signal: str | None = None
  required_messages: tuple[HyundaiCanFDMessageRequirement, ...] = ()
  required_firmware: tuple[HyundaiFirmwareRequirement, ...] = ()


@dataclass(frozen=True)
class HyundaiCanFDLongitudinalConfig:
  harness_connection: HyundaiCanFDHarnessConnection = HyundaiCanFDHarnessConnection.UNKNOWN
  stock_scc_bus: HyundaiCanFDBusRole | None = None
  command_bus: HyundaiCanFDBusRole | None = None
  command_address: int | None = None
  command_dlc: int | None = None
  command_frequency_hz: int | None = None
  ecu_disable_strategy: HyundaiCanFDEcuDisableStrategy = HyundaiCanFDEcuDisableStrategy.UNCONFIGURED
  stock_aeb_fcw_status: HyundaiCanFDStockSafetyStatus = HyundaiCanFDStockSafetyStatus.UNVERIFIED
  required_messages: tuple[HyundaiCanFDMessageRequirement, ...] = ()
  required_firmware: tuple[HyundaiFirmwareRequirement, ...] = ()

  def __post_init__(self):
    command_fields = (self.stock_scc_bus, self.command_bus, self.command_address, self.command_dlc, self.command_frequency_hz)
    if any(field is not None for field in command_fields) != all(field is not None for field in command_fields):
      raise ValueError("CAN-FD longitudinal command metadata must be complete")
    if self.configured:
      if self.command_dlc is None or self.command_frequency_hz is None:
        raise ValueError("configured CAN-FD longitudinal profiles require command metadata")
      if self.harness_connection == HyundaiCanFDHarnessConnection.UNKNOWN:
        raise ValueError("configured CAN-FD longitudinal profiles require a harness connection")
      if self.ecu_disable_strategy == HyundaiCanFDEcuDisableStrategy.UNCONFIGURED:
        raise ValueError("configured CAN-FD longitudinal profiles require an ECU disable strategy")
      if self.command_dlc <= 0 or self.command_frequency_hz <= 0:
        raise ValueError("configured CAN-FD longitudinal profiles require positive DLC and frequency")
      if not self.required_messages or not self.required_firmware:
        raise ValueError("configured CAN-FD longitudinal profiles require passive evidence")

  @property
  def configured(self) -> bool:
    return self.command_address is not None


@dataclass
class HyundaiCanFDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: "hyundai_canfd_generated"})
  steering_config: HyundaiCanFDSteeringConfig = field(default_factory=HyundaiCanFDSteeringConfig)
  longitudinal_config: HyundaiCanFDLongitudinalConfig = field(default_factory=HyundaiCanFDLongitudinalConfig)

  def init(self):
    self.flags |= HyundaiFlags.CANFD


class CAR(Platforms):
  # Hyundai
  HYUNDAI_AZERA_6TH_GEN = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Azera 2022", "All", car_parts=CarParts.common([CarHarness.hyundai_k]))],
    CarSpecs(wheelbase=2.885, steerRatio=14.5),
  )
  HYUNDAI_AZERA_HEV_6TH_GEN = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Hyundai Azera Hybrid 2019", "All", car_parts=CarParts.common([CarHarness.hyundai_c])),
      HyundaiCarDocs("Hyundai Azera Hybrid 2020", "All", car_parts=CarParts.common([CarHarness.hyundai_k])),
    ],
    CarSpecs(wheelbase=2.885, steerRatio=14.5),
    flags=HyundaiFlags.HYBRID,
  )
  HYUNDAI_ELANTRA = HyundaiPlatformConfig(
    [
      # TODO: 2017-18 could be Hyundai G
      HyundaiCarDocs("Hyundai Elantra 2017-18", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_b])),
      HyundaiCarDocs("Hyundai Elantra 2019", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_g])),
    ],
    # steerRatio: 14 is Stock | Settled Params Learner values are steerRatio: 15.401566348670535, stiffnessFactor settled on 1.0081302973865127
    CarSpecs(wheelbase=2.7, steerRatio=15.4, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.LEGACY | HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.MIN_STEER_32_MPH,
  )
  HYUNDAI_ELANTRA_GT_I30 = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Hyundai Elantra GT 2017-20", car_parts=CarParts.common([CarHarness.hyundai_e])),
      HyundaiCarDocs("Hyundai i30 2017-19", car_parts=CarParts.common([CarHarness.hyundai_e])),
    ],
    HYUNDAI_ELANTRA.specs,
    flags=HyundaiFlags.LEGACY | HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.MIN_STEER_32_MPH,
  )
  HYUNDAI_ELANTRA_2021 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Elantra 2021-23", video="https://youtu.be/_EdYQtV52-c", car_parts=CarParts.common([CarHarness.hyundai_k]))],
    CarSpecs(wheelbase=2.72, steerRatio=12.9, tireStiffnessFactor=0.65),
    flags=HyundaiFlags.CHECKSUM_CRC8,
  )
  HYUNDAI_ELANTRA_HEV_2021 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Elantra Hybrid 2021-23", video="https://youtu.be/_EdYQtV52-c",
                    car_parts=CarParts.common([CarHarness.hyundai_k]))],
    CarSpecs(wheelbase=2.72, steerRatio=12.9, tireStiffnessFactor=0.65),
    flags=HyundaiFlags.CHECKSUM_CRC8 | HyundaiFlags.HYBRID,
  )
  HYUNDAI_GENESIS = HyundaiPlatformConfig(
    [
      # TODO: check 2015 packages
      HyundaiCarDocs("Hyundai Genesis 2015-16", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_j])),
      HyundaiCarDocs("Genesis G80 2017", "All", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_j])),
    ],
    CarSpecs(wheelbase=3.01, steerRatio=16.5, minSteerSpeed=60 * CV.KPH_TO_MS),
    flags=HyundaiFlags.CHECKSUM_6B | HyundaiFlags.LEGACY,
  )
  HYUNDAI_IONIQ = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Ioniq Hybrid 2017-19", car_parts=CarParts.common([CarHarness.hyundai_c]))],
    CarSpecs(wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.HYBRID | HyundaiFlags.MIN_STEER_32_MPH,
  )
  HYUNDAI_IONIQ_HEV_2022 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Ioniq Hybrid 2020-22", car_parts=CarParts.common([CarHarness.hyundai_h]))],
    CarSpecs(wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.HYBRID | HyundaiFlags.LEGACY,
  )
  HYUNDAI_IONIQ_EV_LTD = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Ioniq Electric 2019", car_parts=CarParts.common([CarHarness.hyundai_c]))],
    CarSpecs(wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.EV | HyundaiFlags.LEGACY | HyundaiFlags.MIN_STEER_32_MPH,
  )
  HYUNDAI_IONIQ_EV_2020 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Ioniq Electric 2020", "All", car_parts=CarParts.common([CarHarness.hyundai_h]))],
    CarSpecs(wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.EV,
  )
  HYUNDAI_IONIQ_PHEV_2019 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Ioniq Plug-in Hybrid 2019", car_parts=CarParts.common([CarHarness.hyundai_c]))],
    CarSpecs(wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.HYBRID | HyundaiFlags.MIN_STEER_32_MPH,
  )
  HYUNDAI_IONIQ_PHEV = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Ioniq Plug-in Hybrid 2020-22", "All", car_parts=CarParts.common([CarHarness.hyundai_h]))],
    CarSpecs(wheelbase=2.7, steerRatio=13.73, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.HYBRID,
  )
  HYUNDAI_KONA = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Kona 2020", min_enable_speed=6 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_b]))],
    CarSpecs(wheelbase=2.6, steerRatio=13.42, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.CLUSTER_GEARS | HyundaiFlags.ALT_LIMITS,
  )
  HYUNDAI_KONA_2022 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Kona 2022-23", car_parts=CarParts.common([CarHarness.hyundai_o]))],
    CarSpecs(wheelbase=2.6, steerRatio=13.42, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.CAMERA_SCC | HyundaiFlags.ALT_LIMITS_2,
  )
  HYUNDAI_KONA_EV = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Kona Electric 2018-21", car_parts=CarParts.common([CarHarness.hyundai_g]))],
    CarSpecs(wheelbase=2.6, steerRatio=13.42, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.EV | HyundaiFlags.ALT_LIMITS,
  )
  HYUNDAI_KONA_EV_2022 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Kona Electric 2022-23", car_parts=CarParts.common([CarHarness.hyundai_o]))],
    CarSpecs(wheelbase=2.6, steerRatio=13.42, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.CAMERA_SCC | HyundaiFlags.EV | HyundaiFlags.ALT_LIMITS,
  )
  HYUNDAI_KONA_EV_2ND_GEN = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Hyundai Kona Electric (with HDA II, Korea only) 2023", video="https://www.youtube.com/watch?v=U2fOCmcQ8hw",
                    car_parts=CarParts.common([CarHarness.hyundai_r]))],
    CarSpecs(wheelbase=2.66, steerRatio=13.6, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.EV | HyundaiFlags.CANFD_NO_RADAR_DISABLE,
  )
  HYUNDAI_KONA_HEV = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Kona Hybrid 2020", car_parts=CarParts.common([CarHarness.hyundai_i]))],  # TODO: check packages,
    CarSpecs(wheelbase=2.6, steerRatio=13.42, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.HYBRID | HyundaiFlags.ALT_LIMITS,
  )
  HYUNDAI_NEXO_1ST_GEN = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Nexo 2021", "All", car_parts=CarParts.common([CarHarness.hyundai_h]))],
    CarSpecs(wheelbase=2.79, steerRatio=14.19),  # https://www.hyundainews.com/assets/documents/original/42768-2021NEXOProductGuideSpecs.pdf
    flags=HyundaiFlags.FCEV,
  )
  HYUNDAI_SANTA_FE = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Santa Fe 2019-20", "All", video="https://youtu.be/bjDR0YjM__s",
                    car_parts=CarParts.common([CarHarness.hyundai_d]))],
    CarSpecs(wheelbase=2.766, steerRatio=16.55, tireStiffnessFactor=0.82),
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8,
  )
  HYUNDAI_SANTA_FE_2022 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Santa Fe 2021-23", "All", video="https://youtu.be/VnHzSTygTS4",
                    car_parts=CarParts.common([CarHarness.hyundai_l]))],
    HYUNDAI_SANTA_FE.specs,
    flags=HyundaiFlags.CHECKSUM_CRC8,
  )
  HYUNDAI_SANTA_FE_HEV_2022 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Santa Fe Hybrid 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_l]))],
    HYUNDAI_SANTA_FE.specs,
    flags=HyundaiFlags.CHECKSUM_CRC8 | HyundaiFlags.HYBRID,
  )
  HYUNDAI_SANTA_FE_PHEV_2022 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Santa Fe Plug-in Hybrid 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_l]))],
    HYUNDAI_SANTA_FE.specs,
    flags=HyundaiFlags.CHECKSUM_CRC8 | HyundaiFlags.HYBRID,
  )
  HYUNDAI_SONATA = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Sonata 2020-23", "All", video="https://www.youtube.com/watch?v=ix63r9kE3Fw",
                   car_parts=CarParts.common([CarHarness.hyundai_a]))],
    CarSpecs(wheelbase=2.84, steerRatio=13.27 * 1.15, tireStiffnessFactor=0.65),  # 15% higher at the center seems reasonable
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8,
  )
  HYUNDAI_SONATA_LF = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Sonata 2018-19", car_parts=CarParts.common([CarHarness.hyundai_e]))],
    CarSpecs(wheelbase=2.804, steerRatio=13.27 * 1.15),  # 15% higher at the center seems reasonable
    flags=HyundaiFlags.UNSUPPORTED_LONGITUDINAL | HyundaiFlags.TCU_GEARS,
  )
  HYUNDAI_STARIA_4TH_GEN = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Hyundai Staria 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_k]))],
    CarSpecs(wheelbase=3.273, steerRatio=11.94),  # https://www.hyundai.com/content/dam/hyundai/au/en/models/staria-load/premium-pip-update-2023/spec-sheet/STARIA_Load_Spec-Table_March_2023_v3.1.pdf
  )
  HYUNDAI_TUCSON = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Hyundai Tucson 2021", min_enable_speed=19 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_l])),
      HyundaiCarDocs("Hyundai Tucson Diesel 2019", car_parts=CarParts.common([CarHarness.hyundai_l])),
    ],
    CarSpecs(wheelbase=2.67, steerRatio=16.1, tireStiffnessFactor=0.385),
    flags=HyundaiFlags.TCU_GEARS,
  )
  HYUNDAI_PALISADE = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Hyundai Palisade 2020-22", "All", video="https://youtu.be/TAnDqjF4fDY?t=456", car_parts=CarParts.common([CarHarness.hyundai_h])),
      HyundaiCarDocs("Kia Telluride 2020-22", "All", car_parts=CarParts.common([CarHarness.hyundai_h])),
    ],
    CarSpecs(wheelbase=2.9, steerRatio=15.6 * 1.15, tireStiffnessFactor=0.63),
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8,
  )
  HYUNDAI_VELOSTER = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Veloster 2019-20", min_enable_speed=5. * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_e]))],
    CarSpecs(wheelbase=2.8, steerRatio=13.75 * 1.15, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.LEGACY | HyundaiFlags.TCU_GEARS,
  )
  HYUNDAI_SONATA_HYBRID = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Sonata Hybrid 2020-23", "All", car_parts=CarParts.common([CarHarness.hyundai_a]))],
    HYUNDAI_SONATA.specs,
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8 | HyundaiFlags.HYBRID,
  )
  HYUNDAI_IONIQ_5 = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Hyundai Ioniq 5 (Southeast Asia and Europe only) 2022-24", "All", car_parts=CarParts.common([CarHarness.hyundai_q])),
      HyundaiCarDocs("Hyundai Ioniq 5 (without HDA II) 2022-24", "Highway Driving Assist", car_parts=CarParts.common([CarHarness.hyundai_k])),
      HyundaiCarDocs("Hyundai Ioniq 5 (with HDA II) 2022-24", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_q])),
    ],
    CarSpecs(wheelbase=2.97, steerRatio=14.26, tireStiffnessFactor=0.65),
    flags=HyundaiFlags.EV,
  )
  HYUNDAI_IONIQ_6 = HyundaiCanFDPlatformConfig(
    [
        HyundaiCarDocs("Hyundai Ioniq 6 (without HDA II) 2023-24", "Highway Driving Assist", car_parts=CarParts.common([CarHarness.hyundai_l])),
        HyundaiCarDocs("Hyundai Ioniq 6 (with HDA II) 2023-24", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_p])),
    ],
    HYUNDAI_IONIQ_5.specs,
    flags=HyundaiFlags.EV | HyundaiFlags.CANFD_NO_RADAR_DISABLE,
  )
  HYUNDAI_IONIQ_9 = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Hyundai Ioniq 9 2025", "All", car_parts=CarParts.common([CarHarness.hyundai_m]))],
    CarSpecs(wheelbase=3.13, steerRatio=16.02),
    flags=HyundaiFlags.EV,
    steering_config=HyundaiCanFDSteeringConfig(
      api=HyundaiCanFDSteeringApi.ANGLE_LFA_ALT,
      command_address=0xCB,
      command_dlc=24,
      stock_bus=HyundaiCanFDBusRole.CAM,
      command_bus=HyundaiCanFDBusRole.ECAN,
      measured_angle_signal="MDPS_EstStrAnglVal",
      required_messages=(
        HyundaiCanFDMessageRequirement(HyundaiCanFDBusRole.CAM, 0xCB, 24),
        HyundaiCanFDMessageRequirement(HyundaiCanFDBusRole.CAM, 0x1A0, 32),
        HyundaiCanFDMessageRequirement(HyundaiCanFDBusRole.ECAN, 0x35, 32),
        HyundaiCanFDMessageRequirement(HyundaiCanFDBusRole.ECAN, 0xA0, 24),
        HyundaiCanFDMessageRequirement(HyundaiCanFDBusRole.ECAN, 0x125, 16),
        HyundaiCanFDMessageRequirement(HyundaiCanFDBusRole.ECAN, 0x175, 24),
        HyundaiCanFDMessageRequirement(HyundaiCanFDBusRole.ECAN, 0x1CF, 8),
        HyundaiCanFDMessageRequirement(HyundaiCanFDBusRole.ECAN, 0xEA, 24),
      ),
      required_firmware=(
        HyundaiFirmwareRequirement(Ecu.fwdRadar, 0x7D0, b'\xf1\x00MEev RDR -----      1.00 1.00 99110-GO000         '),
        HyundaiFirmwareRequirement(Ecu.fwdCamera, 0x7C4, b'\xf1\x00ME  MFC  AT KOR LHD 1.00 1.01 99211-GO000 250103'),
        HyundaiFirmwareRequirement(Ecu.adas, 0x730, b'\xf1\x00ME  FBL5 1.00 1.01 250122'),
      ),
    ),
    longitudinal_config=HyundaiCanFDLongitudinalConfig(
      # The physical harness is connected at the ADAS module, while passive
      # evidence shows stock SCC_CONTROL arriving on CAM and replacement
      # SCC_CONTROL being transmitted on ECAN. The two facts are independent.
      harness_connection=HyundaiCanFDHarnessConnection.ADAS,
      stock_scc_bus=HyundaiCanFDBusRole.CAM,
      command_bus=HyundaiCanFDBusRole.ECAN,
      command_address=0x1A0,
      command_dlc=32,
      command_frequency_hz=50,
      # The known-good route does not disable 0x730 or 0x7D0. Activation stays
      # prohibited until stock AEB/FCW preservation is independently proven.
      ecu_disable_strategy=HyundaiCanFDEcuDisableStrategy.NONE,
      stock_aeb_fcw_status=HyundaiCanFDStockSafetyStatus.UNVERIFIED,
      required_messages=(
        HyundaiCanFDMessageRequirement(HyundaiCanFDBusRole.CAM, 0x1A0, 32),
        HyundaiCanFDMessageRequirement(HyundaiCanFDBusRole.CAM, 0x12A, 16),
        HyundaiCanFDMessageRequirement(HyundaiCanFDBusRole.ECAN, 0x35, 32),
        HyundaiCanFDMessageRequirement(HyundaiCanFDBusRole.ECAN, 0xA0, 24),
        HyundaiCanFDMessageRequirement(HyundaiCanFDBusRole.ECAN, 0x175, 24),
        HyundaiCanFDMessageRequirement(HyundaiCanFDBusRole.ECAN, 0x1CF, 8),
      ),
      required_firmware=(
        HyundaiFirmwareRequirement(Ecu.fwdRadar, 0x7D0, b'\xf1\x00MEev RDR -----      1.00 1.00 99110-GO000         '),
        HyundaiFirmwareRequirement(Ecu.fwdCamera, 0x7C4, b'\xf1\x00ME  MFC  AT KOR LHD 1.00 1.01 99211-GO000 250103'),
        HyundaiFirmwareRequirement(Ecu.adas, 0x730, b'\xf1\x00ME  FBL5 1.00 1.01 250122'),
      ),
    ),
  )
  HYUNDAI_TUCSON_4TH_GEN = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Hyundai Tucson 2022", car_parts=CarParts.common([CarHarness.hyundai_n])),
      HyundaiCarDocs("Hyundai Tucson 2023-24", "All", car_parts=CarParts.common([CarHarness.hyundai_n])),
      HyundaiCarDocs("Hyundai Tucson Hybrid 2022-24", "All", car_parts=CarParts.common([CarHarness.hyundai_n])),
      HyundaiCarDocs("Hyundai Tucson Plug-in Hybrid 2024", "All", car_parts=CarParts.common([CarHarness.hyundai_n])),
    ],
    CarSpecs(wheelbase=2.756, steerRatio=13.7, tireStiffnessFactor=0.385),
  )
  HYUNDAI_SANTA_CRUZ_1ST_GEN = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Hyundai Santa Cruz 2022-24", car_parts=CarParts.common([CarHarness.hyundai_n]))],
    # weight from Limited trim - the only supported trim, steering ratio according to Hyundai News https://www.hyundainews.com/assets/documents/original/48035-2022SantaCruzProductGuideSpecsv2081521.pdf
    CarSpecs(wheelbase=3, steerRatio=14.2),
  )
  HYUNDAI_CUSTIN_1ST_GEN = HyundaiPlatformConfig(
    [HyundaiCarDocs("Hyundai Custin 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_k]))],
    CarSpecs(wheelbase=3.055, steerRatio=17),  # steerRatio: from learner
    flags=HyundaiFlags.CHECKSUM_CRC8,
  )

  # Kia
  KIA_FORTE = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Kia Forte 2019-21", min_enable_speed=6 * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_g])),
      HyundaiCarDocs("Kia Forte 2022-23", car_parts=CarParts.common([CarHarness.hyundai_e])),
    ],
    CarSpecs(wheelbase=2.8, steerRatio=13.75, tireStiffnessFactor=0.5)
  )
  KIA_K5_2021 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia K5 2021-24", car_parts=CarParts.common([CarHarness.hyundai_a]))],
    CarSpecs(wheelbase=2.85, steerRatio=13.27, tireStiffnessFactor=0.5),  # 2021 Kia K5 Steering Ratio (all trims)
    flags=HyundaiFlags.CHECKSUM_CRC8,
  )
  KIA_K5_HEV_2020 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia K5 Hybrid 2020-22", car_parts=CarParts.common([CarHarness.hyundai_a]))],
    KIA_K5_2021.specs,
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.CHECKSUM_CRC8 | HyundaiFlags.HYBRID,
  )
  KIA_K7_2017 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia K7 2017", car_parts=CarParts.common([CarHarness.hyundai_c]))],
    CarSpecs(wheelbase=2.86, steerRatio=16.8),
  )
  KIA_K8_HEV_1ST_GEN = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Kia K8 Hybrid (with HDA II) 2023", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_q]))],
    # steerRatio: guesstimate from K5 platform
    CarSpecs(wheelbase=2.895, steerRatio=13.27)
  )
  KIA_NIRO_EV = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Kia Niro EV 2019", "All", video="https://www.youtube.com/watch?v=lT7zcG6ZpGo", car_parts=CarParts.common([CarHarness.hyundai_h])),
      HyundaiCarDocs("Kia Niro EV 2020", "All", video="https://www.youtube.com/watch?v=lT7zcG6ZpGo", car_parts=CarParts.common([CarHarness.hyundai_f])),
      HyundaiCarDocs("Kia Niro EV 2021", "All", video="https://www.youtube.com/watch?v=lT7zcG6ZpGo", car_parts=CarParts.common([CarHarness.hyundai_c])),
      HyundaiCarDocs("Kia Niro EV 2022", "All", video="https://www.youtube.com/watch?v=lT7zcG6ZpGo", car_parts=CarParts.common([CarHarness.hyundai_h])),
    ],
    CarSpecs(wheelbase=2.7, steerRatio=13.6, tireStiffnessFactor=0.385),  # average of all the cars
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.EV,
  )
  KIA_NIRO_EV_2ND_GEN = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Kia Niro EV (without HDA II) 2023-25", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
      HyundaiCarDocs("Kia Niro EV (with HDA II) 2024-25", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_r])),
    ],
    KIA_NIRO_EV.specs,
    flags=HyundaiFlags.EV,
  )
  KIA_NIRO_PHEV = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Kia Niro Hybrid 2018", min_enable_speed=10. * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_c])),
      HyundaiCarDocs("Kia Niro Plug-in Hybrid 2018-19", "All", min_enable_speed=10. * CV.MPH_TO_MS, car_parts=CarParts.common([CarHarness.hyundai_c])),
      HyundaiCarDocs("Kia Niro Plug-in Hybrid 2020", car_parts=CarParts.common([CarHarness.hyundai_d])),
    ],
    KIA_NIRO_EV.specs,
    flags=HyundaiFlags.MANDO_RADAR | HyundaiFlags.HYBRID | HyundaiFlags.UNSUPPORTED_LONGITUDINAL | HyundaiFlags.MIN_STEER_32_MPH,
  )
  KIA_NIRO_PHEV_2022 = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Kia Niro Plug-in Hybrid 2021", car_parts=CarParts.common([CarHarness.hyundai_d])),
      HyundaiCarDocs("Kia Niro Plug-in Hybrid 2022", car_parts=CarParts.common([CarHarness.hyundai_f])),
    ],
    KIA_NIRO_EV.specs,
    flags=HyundaiFlags.HYBRID | HyundaiFlags.MANDO_RADAR,
  )
  KIA_NIRO_HEV_2021 = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Kia Niro Hybrid 2021", car_parts=CarParts.common([CarHarness.hyundai_d])),
      HyundaiCarDocs("Kia Niro Hybrid 2022", car_parts=CarParts.common([CarHarness.hyundai_f])),
    ],
    KIA_NIRO_EV.specs,
    flags=HyundaiFlags.HYBRID,
  )
  KIA_NIRO_HEV_2ND_GEN = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Kia Niro Hybrid 2023-24", car_parts=CarParts.common([CarHarness.hyundai_a]))],
    KIA_NIRO_EV.specs,
  )
  KIA_OPTIMA_G4 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia Optima 2017", "Advanced Smart Cruise Control",
                    car_parts=CarParts.common([CarHarness.hyundai_b]))],  # TODO: may support 2016, 2018
    CarSpecs(wheelbase=2.8, steerRatio=13.75, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.LEGACY | HyundaiFlags.TCU_GEARS | HyundaiFlags.MIN_STEER_32_MPH,
  )
  KIA_OPTIMA_G4_FL = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia Optima 2019-20", car_parts=CarParts.common([CarHarness.hyundai_g]))],
    CarSpecs(wheelbase=2.8, steerRatio=13.75, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.UNSUPPORTED_LONGITUDINAL | HyundaiFlags.TCU_GEARS,
  )
  # TODO: may support adjacent years. may have a non-zero minimum steering speed
  KIA_OPTIMA_H = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia Optima Hybrid 2017", "Advanced Smart Cruise Control", car_parts=CarParts.common([CarHarness.hyundai_c]))],
    CarSpecs(wheelbase=2.8, steerRatio=13.75, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.HYBRID | HyundaiFlags.LEGACY,
  )
  KIA_OPTIMA_H_G4_FL = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia Optima Hybrid 2019", car_parts=CarParts.common([CarHarness.hyundai_h]))],
    CarSpecs(wheelbase=2.8, steerRatio=13.75, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.HYBRID | HyundaiFlags.UNSUPPORTED_LONGITUDINAL,
  )
  KIA_SELTOS = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia Seltos 2021", car_parts=CarParts.common([CarHarness.hyundai_a]))],
    CarSpecs(wheelbase=2.63, steerRatio=14.56),
    flags=HyundaiFlags.CHECKSUM_CRC8,
  )
  KIA_SPORTAGE_5TH_GEN = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Kia Sportage 2023-24", car_parts=CarParts.common([CarHarness.hyundai_n])),
      HyundaiCarDocs("Kia Sportage Hybrid 2023", car_parts=CarParts.common([CarHarness.hyundai_n])),
      HyundaiCarDocs("Kia Sportage Plug-in Hybrid 2023", car_parts=CarParts.common([CarHarness.hyundai_n])),
    ],
    # weight from SX and above trims, average of FWD and AWD version, steering ratio according to Kia News https://www.kiamedia.com/us/en/models/sportage/2023/specifications
    CarSpecs(wheelbase=2.756, steerRatio=13.6),
  )
  KIA_SORENTO = HyundaiPlatformConfig(
    [
      HyundaiCarDocs("Kia Sorento 2018", "Advanced Smart Cruise Control & LKAS", video="https://www.youtube.com/watch?v=Fkh3s6WHJz8",
                     car_parts=CarParts.common([CarHarness.hyundai_e])),
      HyundaiCarDocs("Kia Sorento 2019", video="https://www.youtube.com/watch?v=Fkh3s6WHJz8", car_parts=CarParts.common([CarHarness.hyundai_e])),
    ],
    CarSpecs(wheelbase=2.78, steerRatio=14.4 * 1.1),  # 10% higher at the center seems reasonable
    flags=HyundaiFlags.CHECKSUM_6B | HyundaiFlags.UNSUPPORTED_LONGITUDINAL,
  )
  KIA_SORENTO_4TH_GEN = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Kia Sorento 2021-23", car_parts=CarParts.common([CarHarness.hyundai_k]))],
    CarSpecs(wheelbase=2.81, steerRatio=13.5),  # average of the platforms
    flags=HyundaiFlags.CANFD_RADAR_SCC,
  )
  KIA_SORENTO_HEV_4TH_GEN = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Kia Sorento Hybrid 2021-23", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
      HyundaiCarDocs("Kia Sorento Plug-in Hybrid 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
    ],
    CarSpecs(wheelbase=2.81, steerRatio=13.5),  # average of the platforms
    flags=HyundaiFlags.CANFD_RADAR_SCC,
  )
  KIA_STINGER = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia Stinger 2018-20", video="https://www.youtube.com/watch?v=MJ94qoofYw0",
                    car_parts=CarParts.common([CarHarness.hyundai_c]))],
    CarSpecs(wheelbase=2.78, steerRatio=14.4 * 1.15)  # 15% higher at the center seems reasonable
  )
  KIA_STINGER_2022 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia Stinger 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_k]))],
    KIA_STINGER.specs,
  )
  KIA_CEED = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia Ceed 2019-21", car_parts=CarParts.common([CarHarness.hyundai_e]))],
    CarSpecs(wheelbase=2.65, steerRatio=13.75, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.LEGACY,
  )
  KIA_CEED_PHEV = HyundaiPlatformConfig(
    [HyundaiCarDocs("Kia Ceed Plug-in Hybrid 2021", car_parts=CarParts.common([CarHarness.hyundai_b]))],
    CarSpecs(wheelbase=2.65, steerRatio=13.75, tireStiffnessFactor=0.5),
    flags=HyundaiFlags.LEGACY | HyundaiFlags.HYBRID,
  )
  KIA_EV6 = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Kia EV6 (Southeast Asia only) 2022-24", "All", car_parts=CarParts.common([CarHarness.hyundai_p])),
      HyundaiCarDocs("Kia EV6 (without HDA II) 2022-24", "Highway Driving Assist", car_parts=CarParts.common([CarHarness.hyundai_l])),
      HyundaiCarDocs("Kia EV6 (with HDA II) 2022-24", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_p]))
    ],
    CarSpecs(wheelbase=2.9, steerRatio=16, tireStiffnessFactor=0.65),
    flags=HyundaiFlags.EV,
  )
  KIA_CARNIVAL_4TH_GEN = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Kia Carnival 2022-24", car_parts=CarParts.common([CarHarness.hyundai_a])),
      HyundaiCarDocs("Kia Carnival (China only) 2023", car_parts=CarParts.common([CarHarness.hyundai_k]))
    ],
    CarSpecs(wheelbase=3.09, steerRatio=14.23),
    flags=HyundaiFlags.CANFD_RADAR_SCC,
  )

  # Genesis
  GENESIS_GV60_EV_1ST_GEN = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Genesis GV60 (Advanced Trim) 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_a])),
      HyundaiCarDocs("Genesis GV60 (Performance Trim) 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_k])),
    ],
    CarSpecs(wheelbase=2.9, steerRatio=17.6),
    flags=HyundaiFlags.EV,
  )
  GENESIS_G70 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Genesis G70 2018", "All", car_parts=CarParts.common([CarHarness.hyundai_f]))],
    CarSpecs(wheelbase=2.84, steerRatio=13.56),
    flags=HyundaiFlags.LEGACY,
  )
  GENESIS_G70_2020 = HyundaiPlatformConfig(
    [
      # TODO: 2021 MY harness is unknown
      HyundaiCarDocs("Genesis G70 2019-21", "All", car_parts=CarParts.common([CarHarness.hyundai_f])),
      # TODO: From 3.3T Sport Advanced 2022 & Prestige 2023 Trim, 2.0T is unknown
      HyundaiCarDocs("Genesis G70 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_l])),
    ],
    GENESIS_G70.specs,
    flags=HyundaiFlags.MANDO_RADAR,
  )
  GENESIS_GV70_1ST_GEN = HyundaiCanFDPlatformConfig(
    [
      # TODO: Hyundai P is likely the correct harness for HDA II for 2.5T (unsupported due to missing ADAS ECU, is that the radar?)
      HyundaiCarDocs("Genesis GV70 (2.5T Trim, without HDA II) 2022-24", "All", car_parts=CarParts.common([CarHarness.hyundai_l])),
      HyundaiCarDocs("Genesis GV70 (3.5T Trim, without HDA II) 2022-23", "All", car_parts=CarParts.common([CarHarness.hyundai_m])),
    ],
    CarSpecs(wheelbase=2.87, steerRatio=14.6),
    flags=HyundaiFlags.CANFD_RADAR_SCC,
  )
  GENESIS_GV70_ELECTRIFIED_1ST_GEN = HyundaiCanFDPlatformConfig(
    [
      HyundaiCarDocs("Genesis GV70 Electrified (Australia Only) 2022", "All", car_parts=CarParts.common([CarHarness.hyundai_q])),
      HyundaiCarDocs("Genesis GV70 Electrified (with HDA II) 2023-24", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_q])),
    ],
    CarSpecs(wheelbase=2.87, steerRatio=17.1),
    flags=HyundaiFlags.EV,
  )
  GENESIS_G80 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Genesis G80 2018-19", "All", car_parts=CarParts.common([CarHarness.hyundai_h]))],
    CarSpecs(wheelbase=3.01, steerRatio=16.5),
    flags=HyundaiFlags.LEGACY,
  )
  GENESIS_G80_2ND_GEN_FL = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Genesis G80 (2.5T Advanced Trim, with HDA II) 2024", "Highway Driving Assist II", car_parts=CarParts.common([CarHarness.hyundai_p]))],
    CarSpecs(wheelbase=3.00, steerRatio=14.0),
  )
  GENESIS_G90 = HyundaiPlatformConfig(
    [HyundaiCarDocs("Genesis G90 2017-20", "All", car_parts=CarParts.common([CarHarness.hyundai_c]))],
    CarSpecs(wheelbase=3.15, steerRatio=12.069),
  )
  GENESIS_GV80 = HyundaiCanFDPlatformConfig(
    [HyundaiCarDocs("Genesis GV80 2023", "All", car_parts=CarParts.common([CarHarness.hyundai_m]))],
    CarSpecs(wheelbase=2.95, steerRatio=14.14),
    flags=HyundaiFlags.CANFD_RADAR_SCC,
  )


class Buttons:
  NONE = 0
  RES_ACCEL = 1
  SET_DECEL = 2
  GAP_DIST = 3
  CANCEL = 4  # on newer models, this is a pause/resume button


def get_platform_codes(fw_versions: list[bytes]) -> set[tuple[bytes, bytes | None]]:
  # Returns unique, platform-specific identification codes for a set of versions
  codes = set()  # (code-Optional[part], date)
  for fw in fw_versions:
    code_match = PLATFORM_CODE_FW_PATTERN.search(fw)
    part_match = PART_NUMBER_FW_PATTERN.search(fw)
    date_match = DATE_FW_PATTERN.search(fw)
    if code_match is not None:
      code: bytes = code_match.group()
      part = part_match.group() if part_match else None
      date = date_match.group() if date_match else None
      if part is not None:
        # part number starts with generic ECU part type, add what is specific to platform
        code += b"-" + part[-5:]

      codes.add((code, date))
  return codes


def match_fw_to_car_fuzzy(live_fw_versions, vin, offline_fw_versions) -> set[str]:
  # Non-electric CAN FD platforms often do not have platform code specifiers needed
  # to distinguish between hybrid and ICE. All EVs so far are either exclusively
  # electric or specify electric in the platform code.
  canfd_cars = {c for c in CAR if c.config.flags & HyundaiFlags.CANFD}
  ev_cars = {c for c in CAR if c.config.flags & HyundaiFlags.EV}
  fuzzy_platform_blacklist = {str(c) for c in canfd_cars - ev_cars - CANFD_FUZZY_WHITELIST}
  candidates: set[str] = set()

  for candidate, fws in offline_fw_versions.items():
    # Keep track of ECUs which pass all checks (platform codes, within date range)
    valid_found_ecus = set()
    valid_expected_ecus = {ecu[1:] for ecu in fws if ecu[0] in PLATFORM_CODE_ECUS}
    for ecu, expected_versions in fws.items():
      addr = ecu[1:]
      # Only check ECUs expected to have platform codes
      if ecu[0] not in PLATFORM_CODE_ECUS:
        continue

      # Expected platform codes & dates
      codes = get_platform_codes(expected_versions)
      expected_platform_codes = {code for code, _ in codes}
      expected_dates = {date for _, date in codes if date is not None}

      # Found platform codes & dates
      codes = get_platform_codes(live_fw_versions.get(addr, set()))
      found_platform_codes = {code for code, _ in codes}
      found_dates = {date for _, date in codes if date is not None}

      # Check platform code + part number matches for any found versions
      if not any(found_platform_code in expected_platform_codes for found_platform_code in found_platform_codes):
        break

      if ecu[0] in DATE_FW_ECUS:
        # If ECU can have a FW date, require it to exist
        # (this excludes candidates in the database without dates)
        if not len(expected_dates) or not len(found_dates):
          break

        # Check any date within range in the database, format is %y%m%d
        if not any(min(expected_dates) <= found_date <= max(expected_dates) for found_date in found_dates):
          break

      valid_found_ecus.add(addr)

    # If all live ECUs pass all checks for candidate, add it as a match
    if valid_expected_ecus.issubset(valid_found_ecus):
      candidates.add(candidate)

  return candidates - fuzzy_platform_blacklist


HYUNDAI_VERSION_REQUEST_LONG = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf100)  # Long description

HYUNDAI_VERSION_REQUEST_ALT = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf110)  # Alt long description

HYUNDAI_ECU_MANUFACTURING_DATE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.ECU_MANUFACTURING_DATE)

HYUNDAI_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40])

HYUNDAI_FW_QUERY_ECUS = [Ecu.eps, Ecu.abs, Ecu.fwdRadar, Ecu.fwdCamera, Ecu.parkingAdas,
                         Ecu.transmission, Ecu.combinationMeter, Ecu.adas, Ecu.hvac, Ecu.cornerRadar]

# Regex patterns for parsing platform code, FW date, and part number from FW versions
PLATFORM_CODE_FW_PATTERN = re.compile(b'((?<=' + HYUNDAI_VERSION_REQUEST_LONG[1:] +
                                      b')[A-Z]{2}[A-Za-z0-9]{0,2})')
DATE_FW_PATTERN = re.compile(b'(?<=[ -])([0-9]{6}$)')
PART_NUMBER_FW_PATTERN = re.compile(b'(?<=[0-9][.,][0-9]{2} )([0-9]{5}[-/]?[A-Z][A-Z0-9]{3}[0-9])')

# We've seen both ICE and hybrid for these platforms, and they have hybrid descriptors (e.g. MQ4 vs MQ4H)
CANFD_FUZZY_WHITELIST = {CAR.KIA_SORENTO_4TH_GEN, CAR.KIA_SORENTO_HEV_4TH_GEN, CAR.KIA_K8_HEV_1ST_GEN,
                         # TODO: the hybrid variant is not out yet
                         CAR.KIA_CARNIVAL_4TH_GEN}

# List of ECUs expected to have platform codes, camera and radar should exist on all cars
# TODO: use abs, it has the platform code and part number on many platforms
PLATFORM_CODE_ECUS = [Ecu.fwdRadar, Ecu.fwdCamera, Ecu.eps]
# So far we've only seen dates in fwdCamera
# TODO: there are date codes in the ABS firmware versions in hex
DATE_FW_ECUS = [Ecu.fwdCamera]

# Note: an ECU on CAN FD cars may sometimes send 0x30080aaaaaaaaaaa (flow control continue) while we
# are attempting to query ECUs. This currently does not seem to affect fingerprinting from the camera
FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"\xf1\x00[\x00-\xff]{19,56}",
  requests=[
    # TODO: add back whitelists
    # CAN queries (OBD-II port)
    Request(
      [HYUNDAI_VERSION_REQUEST_LONG],
      [HYUNDAI_VERSION_RESPONSE],
      whitelist_ecus=HYUNDAI_FW_QUERY_ECUS,
    ),

    # CAN & CAN-FD queries (from camera)
    Request(
      [HYUNDAI_VERSION_REQUEST_LONG],
      [HYUNDAI_VERSION_RESPONSE],
      bus=0,
    ),
    Request(
      [HYUNDAI_VERSION_REQUEST_LONG],
      [HYUNDAI_VERSION_RESPONSE],
      bus=1,
      obd_multiplexing=False,
    ),
    # Direct harness buses for single-panda CAN-FD installations. Limit these
    # read-only DID queries to the ECUs used by the typed steering capability.
    # No diagnostic-session change, communication-control, or write is sent.
    Request(
      [HYUNDAI_VERSION_REQUEST_LONG],
      [HYUNDAI_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.fwdRadar],
      bus=0,
      obd_multiplexing=False,
    ),
    Request(
      [HYUNDAI_VERSION_REQUEST_LONG],
      [HYUNDAI_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.fwdCamera, Ecu.adas],
      bus=2,
      obd_multiplexing=False,
    ),

    # CAN & CAN FD query to understand the three digit date code
    # LKA steering cars usually use 6 digit date codes, so skip bus 1
    Request(
      [HYUNDAI_ECU_MANUFACTURING_DATE],
      [HYUNDAI_VERSION_RESPONSE],
      bus=0,
      logging=True,
    ),

    # CAN-FD alt request logging queries for hvac and parkingAdas
    Request(
      [HYUNDAI_VERSION_REQUEST_ALT],
      [HYUNDAI_VERSION_RESPONSE],
      bus=0,
      logging=True,
    ),
    Request(
      [HYUNDAI_VERSION_REQUEST_ALT],
      [HYUNDAI_VERSION_RESPONSE],
      bus=1,
      logging=True,
      obd_multiplexing=False,
    ),
  ],
  # We lose these ECUs without the comma power on these cars.
  # Note that we still attempt to match with them when they are present
  non_essential_ecus={
    Ecu.abs: [CAR.HYUNDAI_PALISADE, CAR.HYUNDAI_SONATA, CAR.HYUNDAI_SANTA_FE_2022, CAR.KIA_K5_2021, CAR.HYUNDAI_ELANTRA_2021,
              CAR.HYUNDAI_SANTA_FE, CAR.HYUNDAI_KONA_EV_2022, CAR.HYUNDAI_KONA_EV, CAR.HYUNDAI_CUSTIN_1ST_GEN, CAR.KIA_SORENTO,
              CAR.KIA_CEED, CAR.KIA_SELTOS],
  },
  extra_ecus=[
    (Ecu.parkingAdas, 0x7b1, None),       # ADAS Parking ECU (may exist on all platforms)
    (Ecu.hvac, 0x7b3, None),              # HVAC Control Assembly
    (Ecu.cornerRadar, 0x7b7, None),
    (Ecu.combinationMeter, 0x7c6, None),  # CAN FD Instrument cluster
  ],
  # Custom fuzzy fingerprinting function using platform codes, part numbers + FW dates:
  match_fw_to_car_fuzzy=match_fw_to_car_fuzzy,
)

DBC = CAR.create_dbc_map()
