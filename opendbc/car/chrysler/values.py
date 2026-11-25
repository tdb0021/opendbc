from enum import IntFlag
from dataclasses import dataclass, field

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarHarness, CarDocs, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, p16

Ecu = CarParams.Ecu


class ChryslerSafetyFlags(IntFlag):
  RAM_DT = 1
  RAM_HD = 2
  # openpilot longitudinal. Off unless the alpha-long toggle is on, and the
  # panda only honors it in a non-release build (ALLOW_DEBUG).
  LONG = 4


class ChryslerFlags(IntFlag):
  # Detected flags
  HIGHER_MIN_STEERING_SPEED = 1
  HAS_BSM = 2  # blind spot monitoring


@dataclass
class ChryslerCarDocs(CarDocs):
  package: str = "Adaptive Cruise Control (ACC)"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.fca]))


@dataclass
class ChryslerPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {
    Bus.pt: 'chrysler_pacifica_2017_hybrid_generated',
    Bus.radar: 'chrysler_pacifica_2017_hybrid_private_fusion',
  })


@dataclass(frozen=True)
class ChryslerCarSpecs(CarSpecs):
  minSteerSpeed: float = 3.8  # m/s


class CAR(Platforms):
  # Chrysler
  CHRYSLER_PACIFICA_2018_HYBRID = ChryslerPlatformConfig(
    [ChryslerCarDocs("Chrysler Pacifica Hybrid 2017-18")],
    ChryslerCarSpecs(wheelbase=3.089, steerRatio=16.2),
  )
  CHRYSLER_PACIFICA_2019_HYBRID = ChryslerPlatformConfig(
    [ChryslerCarDocs("Chrysler Pacifica Hybrid 2019-25")],
    CHRYSLER_PACIFICA_2018_HYBRID.specs,
  )
  CHRYSLER_PACIFICA_2018 = ChryslerPlatformConfig(
    [ChryslerCarDocs("Chrysler Pacifica 2017-18")],
    CHRYSLER_PACIFICA_2018_HYBRID.specs,
  )
  CHRYSLER_PACIFICA_2020 = ChryslerPlatformConfig(
    [
      ChryslerCarDocs("Chrysler Pacifica 2019-20"),
      ChryslerCarDocs("Chrysler Pacifica 2021-23", package="All"),
    ],
    CHRYSLER_PACIFICA_2018_HYBRID.specs,
  )

  # Dodge
  DODGE_DURANGO = ChryslerPlatformConfig(
    [ChryslerCarDocs("Dodge Durango 2020-21")],
    CHRYSLER_PACIFICA_2018_HYBRID.specs,
  )

  # Jeep
  JEEP_CHEROKEE_5TH_GEN = ChryslerPlatformConfig(
    [ChryslerCarDocs("Jeep Cherokee 2019-23")],
    ChryslerCarSpecs(wheelbase=2.70, steerRatio=17.0, minSteerSpeed=18.5),
    {Bus.pt: 'chrysler_cusw'},
  )
  JEEP_GRAND_CHEROKEE = ChryslerPlatformConfig(  # includes 2017 Trailhawk
    [ChryslerCarDocs("Jeep Grand Cherokee 2016-18", video="https://www.youtube.com/watch?v=eLR9o2JkuRk")],
    ChryslerCarSpecs(wheelbase=2.71, steerRatio=16.7),
  )

  JEEP_GRAND_CHEROKEE_2019 = ChryslerPlatformConfig(  # includes 2020 Trailhawk
    [ChryslerCarDocs("Jeep Grand Cherokee 2019-21")],
    JEEP_GRAND_CHEROKEE.specs,
  )

  # Ram
  RAM_1500_5TH_GEN = ChryslerPlatformConfig(
    [ChryslerCarDocs("Ram 1500 2019-24", car_parts=CarParts.common([CarHarness.ram]))],
    ChryslerCarSpecs(wheelbase=3.88, steerRatio=16.3, minSteerSpeed=14.5),
    {Bus.pt: 'chrysler_ram_dt_generated'},
  )
  RAM_HD_5TH_GEN = ChryslerPlatformConfig(
    [
      ChryslerCarDocs("Ram 2500 2020-24", car_parts=CarParts.common([CarHarness.ram])),
      ChryslerCarDocs("Ram 3500 2019-24", car_parts=CarParts.common([CarHarness.ram])),
    ],
    ChryslerCarSpecs(wheelbase=3.785, steerRatio=15.61, minSteerSpeed=16.),
    {Bus.pt: 'chrysler_ram_hd_generated'},
  )


class CarControllerParams:
  # Safety ceilings. Must match opendbc/safety/modes/chrysler.h.
  # Friction-brake decel still uses the openpilot standard of -3.5 m/s^2. The only
  # stock friction-brake samples on the 2020 HD were about -0.3 m/s^2 and under a second.
  ACCEL_MIN = -3.5           # m/s^2
  ACCEL_MAX = 2.0            # m/s^2. The logged pull peaked near 2.1; keep the usual cap.
  MAX_TORQUE = 500.0         # Nm, safety max. The 532.5 Nm sample was a cancel blip.
  MIN_TORQUE = -100.0        # Nm, safety floor. Stock coast reached -91 Nm.
  # Stock "no brake" is raw 4093, which decodes to 3.9943 m/s^2. 4.0 packs to raw 4094.
  INACTIVE_DECEL = 3.9943

  # Friction brake only below this. Stock used engine torque for ordinary slowing and
  # ACC_DECEL for two short extra dips around -0.3 m/s^2. -0.25 is that neighborhood;
  # longer braking was not in the log.
  BRAKE_ENTER_ACCEL = -0.25  # m/s^2

  # Open-loop map from route 00000a3e--51571a2b07 (2020 Ram 2500 Cummins, stock ACC,
  # no driver gas, no ACC_DECEL_REQ, n=4320, r=0.56):
  #   ENGINE_TORQUE_REQUEST ≈ 122 Nm per m/s^2 * accel + 145 Nm
  # 55 mph hold was ~152 Nm. The 5→55 mph pull asked for 392–422 Nm.
  # No speed term: almost all of the steady data is near 55 mph, so speed and the
  # 145 Nm rolling load are not separable. The -91 Nm engine-brake dips sit below
  # this line; the safety floor allows them, this mean fit does not reproduce them.
  TORQUE_FROM_ACCEL = 122.0  # Nm / (m/s^2)
  TORQUE_OFFSET = 145.0      # Nm at 0 m/s^2
  MIN_TORQUE_CMD = -100.0    # Nm, same as the safety floor
  MAX_TORQUE_CMD = 450.0     # Nm, above the 422 Nm pull and under the 500 Nm ceiling
  # DAS_5 wheel-torque raw 0. The packer turns a physical 0 Nm into 0x1E57, which
  # this truck never sends. -7767 Nm is raw 0, bytes 4-5 = 00 00.
  WHEEL_TORQUE_INACTIVE = -7767.0

  def __init__(self, CP):
    self.STEER_STEP = 2  # 50 Hz
    self.STEER_ERROR_MAX = 80
    if CP.carFingerprint in RAM_HD:
      self.STEER_DELTA_UP = 14
      self.STEER_DELTA_DOWN = 14
      self.STEER_MAX = 361  # higher than this faults the EPS
    elif CP.carFingerprint in RAM_DT:
      self.STEER_DELTA_UP = 6
      self.STEER_DELTA_DOWN = 6
      self.STEER_MAX = 350  # EPS allows more, up to 350?
    elif CP.carFingerprint in CUSW_CARS:
      self.STEER_STEP = 1  # 100 Hz
      self.STEER_DELTA_UP = 4
      self.STEER_DELTA_DOWN = 4
      self.STEER_MAX = 250  # TODO: Some CUSW will go to 261, some not quite, exact boundaries not yet determined
    else:
      self.STEER_DELTA_UP = 3
      self.STEER_DELTA_DOWN = 3
      self.STEER_MAX = 261  # higher than this faults the EPS


STEER_THRESHOLD = 120

RAM_DT = {CAR.RAM_1500_5TH_GEN, }
RAM_HD = {CAR.RAM_HD_5TH_GEN, }
RAM_CARS = RAM_DT | RAM_HD
CUSW_CARS = {CAR.JEEP_CHEROKEE_5TH_GEN, }


CHRYSLER_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf132)
CHRYSLER_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(0xf132)

CHRYSLER_SOFTWARE_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.SYSTEM_SUPPLIER_ECU_SOFTWARE_NUMBER)
CHRYSLER_SOFTWARE_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.SYSTEM_SUPPLIER_ECU_SOFTWARE_NUMBER)

CHRYSLER_RX_OFFSET = -0x280

FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"[A-Z0-9_]{10} ?",
  requests=[
    Request(
      [CHRYSLER_VERSION_REQUEST],
      [CHRYSLER_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.eps, Ecu.srs, Ecu.fwdRadar, Ecu.combinationMeter],
      rx_offset=CHRYSLER_RX_OFFSET,
      bus=0,
    ),
    Request(
      [CHRYSLER_VERSION_REQUEST],
      [CHRYSLER_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.hybrid, Ecu.engine, Ecu.transmission],
      bus=0,
    ),
    Request(
      [CHRYSLER_SOFTWARE_VERSION_REQUEST],
      [CHRYSLER_SOFTWARE_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.engine, Ecu.transmission],
      bus=0,
    ),
  ],
  extra_ecus=[
    (Ecu.abs, 0x7e4, None),  # alt address for abs on hybrids, NOTE: not on all hybrid platforms
  ],
)

DBC = CAR.create_dbc_map()
