#!/usr/bin/env python3
# ruff: noqa: E501
from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarHarness, CarParts
from opendbc.car.fw_query_definitions import FwQueryConfig, Request
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu


@dataclass
class BydCarDocs(CarDocs):
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.custom]))


@dataclass
class BydPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: "byd_generic_pt"})


class BydFlags(IntFlag):
  CANFD = 1
  ANGLE_CONTROL = 2
  ALT_INDICATOR = 4
  SETSPEED_X10 = 8
  BCM_SEAL = 16
  ALT_ACC_EPS = 32
  ALT_ACC_EPS_SEAL = 64
  ALT_ACC_CURISE_MODE = 128
  ALT_ACC_ACCON1 = 256
  ALT_PCM_BTN = 512
  ACC_STOP_FIX = 1024
  ANGLE_DRIVER_OVERRIDE_RELEASE = 2048


# panda safety flags. These must stay in sync with byd_init()/byd_tx_hook() in
# opendbc/safety/safety/safety_byd.h.
class BydSafetyFlags(IntFlag):
  HAN_TANG_DMEV = 0x1        # default torque protocol: ACC_MPC_STATE / ACC_EPS_STATE
  TANG_DMI = 0x2             # reserved, currently same wire protocol as HAN_TANG
  SONG_PLUS_DMI = 0x4        # reserved, currently same wire protocol as HAN_TANG
  QIN_PLUS_DMI = 0x8         # reserved, currently same wire protocol as HAN_TANG
  YUAN_PLUS_DMI_ATTO3 = 0x10 # reserved, currently same wire protocol as HAN_TANG
  SEAL = 0x20                # angle protocol: ACC_MPC_STATE_SEAL / ACC_EPS_STATE_SEAL


class LKASConfig:
  DISABLE = 0
  ALARM = 1
  LKA = 2
  ALARM_AND_LKA = 3


class CanBus:
  ESC = 0
  MRR = 1
  MPC = 2
  LOOPBACK = 128
  DROPPED = 192


class CarControllerParams:
  STEER_MAX = 300
  # Rate limits satisfy the ISO 11270 jerk guard in test_lateral_limits.
  # The dp module used 17/17 with a 50 Hz send rate, which violates it; the
  # previous CarrotPilot BYD module used 6/9. Retune on car if needed.
  STEER_DELTA_UP = 7
  STEER_DELTA_DOWN = 9
  STEER_DRIVER_ALLOWANCE = 68
  STEER_DRIVER_MULTIPLIER = 3
  STEER_DRIVER_FACTOR = 1
  STEER_ERROR_MAX = 80
  STEER_STEP = 2
  ACC_STEP = 2
  MAX_STEER_ANGLE = 640
  ANGLE_MAX_TORQUE = 200  # 0.01 Nm/count on CID_TCDUBH
  ACCEL_MAX = 3.0
  ACCEL_MIN = -4.5
  K_DASHSPEED = 0.0719088
  K_accel_jerk_upper = 0.1
  K_accel_jerk_lower = 0.5
  K_jerk_xp = [4, 10, 20, 40, 80]
  K_jerk_base_lower_fp = [-2.0, -1.8, -1.4, -1.0, -0.4]
  K_jerk_base_upper_fp = [0.8, 0.7, 0.6, 0.3, 0.2]

  def __init__(self, CP):
    pass


class CAR(Platforms):
  BYD_HAN_DM_20 = BydPlatformConfig([BydCarDocs("Byd Han DM 20", "All")], CarSpecs(mass=2080.0, wheelbase=2.92, steerRatio=15.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 1024)
  BYD_HAN_EV_20 = BydPlatformConfig([BydCarDocs("Byd Han EV 20", "All")], CarSpecs(mass=2100.0, wheelbase=2.959, steerRatio=15.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 1024)
  BYD_HAN_DMI_22 = BydPlatformConfig([BydCarDocs("Byd Han DMI 22", "All")], CarSpecs(mass=2080.0, wheelbase=2.92, steerRatio=15.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 573)
  BYD_HAN_DMI_25 = BydPlatformConfig([BydCarDocs("Byd Han DMI 25", "All")], CarSpecs(mass=2080.0, wheelbase=2.92, steerRatio=15.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 565)
  BYD_HAN_DMI_22R = BydPlatformConfig([BydCarDocs("Byd Han DMI 22R", "All")], CarSpecs(mass=2080.0, wheelbase=2.92, steerRatio=15.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 701)
  BYD_HAN_DMI_22J = BydPlatformConfig([BydCarDocs("Byd Han DMI 22J", "All")], CarSpecs(mass=2080.0, wheelbase=2.92, steerRatio=15.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 607)
  BYD_TANG_DM = BydPlatformConfig([BydCarDocs("Byd Tang DM", "All")], CarSpecs(mass=2250.0, wheelbase=2.82, steerRatio=15.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 0)
  BYD_TANG_DMI_21 = BydPlatformConfig([BydCarDocs("Byd Tang DMI 21", "All")], CarSpecs(mass=2250.0, wheelbase=2.82, steerRatio=15.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 1077)
  BYD_TANG_DMP_22 = BydPlatformConfig([BydCarDocs("Byd Tang DMP 22", "All")], CarSpecs(mass=2445.0, wheelbase=2.82, steerRatio=15.0, centerToFrontRatio=0.6), {Bus.pt: "byd_generic_pt"}, 701)
  BYD_TANG_DMP_23 = BydPlatformConfig([BydCarDocs("Byd Tang DMP 23", "All")], CarSpecs(mass=2445.0, wheelbase=2.82, steerRatio=15.0, centerToFrontRatio=0.6), {Bus.pt: "byd_generic_pt"}, 565)
  BYD_TANG_DMI_24 = BydPlatformConfig([BydCarDocs("Byd Tang DMI 24", "All")], CarSpecs(mass=2250.0, wheelbase=2.82, steerRatio=15.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 565)
  BYD_SONG_PLUS_DMI_21 = BydPlatformConfig([BydCarDocs("Byd Song Plus DMI 21", "All")], CarSpecs(mass=1785.0, wheelbase=2.765, steerRatio=15.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 4)
  BYD_SONG_PLUS_DMI_22 = BydPlatformConfig([BydCarDocs("Byd Song Plus DMI 22", "All")], CarSpecs(mass=1785.0, wheelbase=2.765, steerRatio=15.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 564)
  BYD_SONG_PRO_DMI_22 = BydPlatformConfig([BydCarDocs("Byd Song Pro DMI 22", "All")], CarSpecs(mass=1785.0, wheelbase=2.712, steerRatio=15.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 0)
  BYD_SONG_L_DMI_24 = BydPlatformConfig([BydCarDocs("Byd Song L DMI 24", "All")], CarSpecs(mass=2000.0, wheelbase=2.782, steerRatio=15.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 3799)
  BYD_QIN_PLUS_DMI_23 = BydPlatformConfig([BydCarDocs("Byd Qin Plus DMI 23", "All")], CarSpecs(mass=1580.0, wheelbase=2.718, steerRatio=15.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 0)
  BYD_QIN_L_DMI = BydPlatformConfig([BydCarDocs("Byd Qin L DMI", "All")], CarSpecs(mass=2036.0, wheelbase=2.9, steerRatio=19.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 3671)
  BYD_SEAL_06_DMI = BydPlatformConfig([BydCarDocs("Byd Seal 06 DMI", "All")], CarSpecs(mass=2000.0, wheelbase=2.782, steerRatio=15.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 3799)
  BYD_QIN_PRO = BydPlatformConfig([BydCarDocs("Byd Qin Pro", "All")], CarSpecs(mass=2250.0, wheelbase=2.82, steerRatio=15.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 0)
  BYD_YUAN_PLUS_DMI_22 = BydPlatformConfig([BydCarDocs("Byd Yuan Plus DMI 22", "All")], CarSpecs(mass=1625.0, wheelbase=2.72, steerRatio=15.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 565)
  BYD_SEAL_23 = BydPlatformConfig([BydCarDocs("Byd Seal 23", "All")], CarSpecs(mass=2150.0, wheelbase=2.92, steerRatio=15.0, centerToFrontRatio=0.6), {Bus.pt: "byd_generic_pt"}, 855)
  BYD_SEAL_EV_22 = BydPlatformConfig([BydCarDocs("Byd Seal EV 22", "All")], CarSpecs(mass=2695.0, wheelbase=3.11, steerRatio=17.0, centerToFrontRatio=0.6), {Bus.pt: "byd_generic_pt"}, 1623)
  BYD_SEAL_07_DMI = BydPlatformConfig([BydCarDocs("Byd Seal 07 DMI", "All")], CarSpecs(mass=2695.0, wheelbase=3.11, steerRatio=17.0, centerToFrontRatio=0.6), {Bus.pt: "byd_generic_pt"}, 1623)
  BYD_FRIGATE_07_DMI = BydPlatformConfig([BydCarDocs("Byd Frigate 07 DMI", "All")], CarSpecs(mass=2266.0, wheelbase=2.82, steerRatio=19.0, centerToFrontRatio=0.44), {Bus.pt: "byd_generic_pt"}, 3671)
  BYD_TENGSHI_D9_22R = BydPlatformConfig([BydCarDocs("Byd Tengshi D9 22R", "All")], CarSpecs(mass=2695.0, wheelbase=3.11, steerRatio=17.0, centerToFrontRatio=0.6), {Bus.pt: "byd_generic_pt"}, 1751)
  BYD_TENGSHI_D9_22 = BydPlatformConfig([BydCarDocs("Byd Tengshi D9 22", "All")], CarSpecs(mass=2695.0, wheelbase=3.11, steerRatio=17.0, centerToFrontRatio=0.6), {Bus.pt: "byd_generic_pt"}, 1623)
  BYD_TENGSHI_D9_24 = BydPlatformConfig([BydCarDocs("Byd Tengshi D9 24", "All")], CarSpecs(mass=2695.0, wheelbase=3.11, steerRatio=17.0, centerToFrontRatio=0.6), {Bus.pt: "byd_generic_pt"}, 1623)


DBC = CAR.create_dbc_map()

FW_QUERY_CONFIG = FwQueryConfig(requests=[
  Request([b'>\x00', b'"\xf1\x81'], [b'~\x00', b'b\xf1\x81'], whitelist_ecus=[Ecu.eps], rx_offset=0x8, bus=0),
  Request([b'>\x00', b'"\xf1\x81'], [b'~\x00', b'b\xf1\x81'], whitelist_ecus=[Ecu.adas, Ecu.fwdCamera], rx_offset=0x8, bus=2),
])

PLATFORM_HAN_DMEV = {CAR.BYD_HAN_DM_20, CAR.BYD_HAN_EV_20}
PLATFORM_HAN_DMI = {CAR.BYD_HAN_DMI_22, CAR.BYD_HAN_DMI_22R, CAR.BYD_HAN_DMI_25, CAR.BYD_TANG_DMP_22, CAR.BYD_TANG_DMP_23, CAR.BYD_YUAN_PLUS_DMI_22}
PLATFORM_TANG_DM = {CAR.BYD_QIN_PRO, CAR.BYD_TANG_DM}
PLATFORM_TANG_DMI = {CAR.BYD_SONG_PLUS_DMI_22, CAR.BYD_TANG_DMI_21, CAR.BYD_TANG_DMI_24}
PLATFORM_SONG_PRO = {CAR.BYD_SONG_PRO_DMI_22}
PLATFORM_SONG_PLUS_DMI = {CAR.BYD_SONG_PLUS_DMI_21}
PLATFORM_QIN_PLUS_DMI = {CAR.BYD_QIN_PLUS_DMI_23}
PLATFORM_QIN_SEAL06 = {CAR.BYD_QIN_L_DMI}
PLATFORM_YUAN_PLUS_DMI_ATTO3 = set()
PLATFORM_SEAL = {CAR.BYD_SEAL_23}
PLATFORM_TENGSHI = {CAR.BYD_FRIGATE_07_DMI, CAR.BYD_HAN_DMI_22J, CAR.BYD_SEAL_06_DMI, CAR.BYD_SEAL_07_DMI, CAR.BYD_SEAL_EV_22, CAR.BYD_SONG_L_DMI_24, CAR.BYD_TENGSHI_D9_22, CAR.BYD_TENGSHI_D9_22R, CAR.BYD_TENGSHI_D9_24}

PT_RADAR_100 = {CAR.BYD_HAN_DMI_22, CAR.BYD_HAN_DMI_22R, CAR.BYD_HAN_DMI_25, CAR.BYD_TANG_DMP_22, CAR.BYD_TANG_DMP_23, CAR.BYD_TENGSHI_D9_22, CAR.BYD_TENGSHI_D9_22R, CAR.BYD_YUAN_PLUS_DMI_22}
PT_RADAR_80 = {CAR.BYD_SEAL_07_DMI, CAR.BYD_SEAL_EV_22, CAR.BYD_TENGSHI_D9_24}
PT_RADAR_CAR = {CAR.BYD_HAN_DMI_22, CAR.BYD_HAN_DMI_22R, CAR.BYD_HAN_DMI_25, CAR.BYD_HAN_DM_20, CAR.BYD_HAN_EV_20, CAR.BYD_TANG_DMP_22, CAR.BYD_TANG_DMP_23, CAR.BYD_TENGSHI_D9_22, CAR.BYD_TENGSHI_D9_22R, CAR.BYD_YUAN_PLUS_DMI_22}
RADAR_CAR = {CAR.BYD_QIN_PRO, CAR.BYD_TANG_DM}

# torque control platforms (torque_from_lateral_accel / torque panda safety)
TORQUE_LAT_CAR = {CAR.BYD_QIN_PLUS_DMI_23, CAR.BYD_HAN_DMI_22, CAR.BYD_HAN_DMI_22R, CAR.BYD_HAN_DMI_25, CAR.BYD_HAN_DM_20, CAR.BYD_HAN_EV_20, CAR.BYD_QIN_PRO, CAR.BYD_SONG_PLUS_DMI_21, CAR.BYD_SONG_PLUS_DMI_22, CAR.BYD_SONG_PRO_DMI_22, CAR.BYD_TANG_DM, CAR.BYD_TANG_DMI_21, CAR.BYD_TANG_DMI_24, CAR.BYD_TANG_DMP_22, CAR.BYD_TANG_DMP_23, CAR.BYD_YUAN_PLUS_DMI_22}
EXP_LONG_CAR = {CAR.BYD_FRIGATE_07_DMI, CAR.BYD_HAN_DMI_22, CAR.BYD_HAN_DMI_22J, CAR.BYD_HAN_DMI_22R, CAR.BYD_HAN_DMI_25, CAR.BYD_HAN_DM_20, CAR.BYD_HAN_EV_20, CAR.BYD_QIN_L_DMI, CAR.BYD_QIN_PRO, CAR.BYD_SEAL_06_DMI, CAR.BYD_SEAL_07_DMI, CAR.BYD_SEAL_23, CAR.BYD_SEAL_EV_22, CAR.BYD_SONG_L_DMI_24, CAR.BYD_SONG_PLUS_DMI_21, CAR.BYD_SONG_PLUS_DMI_22, CAR.BYD_SONG_PRO_DMI_22, CAR.BYD_TANG_DM, CAR.BYD_TANG_DMI_21, CAR.BYD_TANG_DMI_24, CAR.BYD_TANG_DMP_22, CAR.BYD_TANG_DMP_23, CAR.BYD_TENGSHI_D9_22, CAR.BYD_TENGSHI_D9_22R, CAR.BYD_TENGSHI_D9_24, CAR.BYD_YUAN_PLUS_DMI_22}

# panda safety coverage. The torque protocol below is what safety_byd.h
# implements and checks (LKAS_Output at bit 16, AccState at bit 19 of 0x32D).
# Angle control platforms are exposed read-only (dashcamOnly): the angle frame
# layout in byd_generic_pt.dbc (CID_TCDUBH / 0x1E2) does not match the SEAL
# angle checks currently in safety_byd.h, so enabling them would be unverified.
SAFETY_TORQUE_CAR = TORQUE_LAT_CAR
SAFETY_SEAL_CAR = set()


if __name__ == "__main__":
  for platform in CAR:
    print(platform, DBC[platform])
