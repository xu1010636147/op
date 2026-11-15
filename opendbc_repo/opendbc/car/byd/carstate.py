
#!/usr/bin/env python3
import copy

import numpy as np
from opendbc.car import Bus, create_button_events, structs
from opendbc.can import CANDefine, CANParser
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.bydparams import get_bool, get_int
from opendbc.car.byd.tuning import Tuning
from opendbc.car.byd.values import (
  BydFlags, CanBus, CarControllerParams, DBC, LKASConfig, PLATFORM_HAN_DMI,
  PLATFORM_QIN_SEAL06, PLATFORM_SEAL, PLATFORM_TENGSHI, PT_RADAR_CAR,
)

ButtonType = structs.CarState.ButtonEvent.Type
DASHSPEED_BP = [0, 30, 60, 90, 120]


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.CP = CP
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.frame = 0
    self.shifter_values = can_define.dv["CID_PFTVEH"]["sig_wymsqb"]
    self.speed_kph = 0
    self.mpc_lkas_config = LKASConfig.DISABLE
    self.acc_hud_adas_counter = 0
    self.acc_mpc_state_counter = 0
    self.acc_cmd_counter = 0
    self.eps_warning = False
    self.eps_warn_angle_range = False
    self.eps_warn_angle_speed = False
    self.eps_warn_angle_range_cnt = 0
    self.eps_warn_angle_speed_cnt = 0
    self.eps_state_warning_cnt = 0
    self.acc_active_last = 0
    self.low_speed_alert = False
    self.lkas_allowed_speed = False
    self.eps_state = 0
    self.acc_state = 0
    self.cruisemodedisp = 0
    self.adas_set_dist = 4
    self.adas_set_dist_changed = False
    self.adas_set_dist_changed_notified = False
    self.mpc_laks_output = 0
    self.mpc_laks_active = False
    self.mpc_laks_reqprepare = False
    self.fake_driver_torque = 0
    self.cam_lkas = {}
    # CID_TCDUBH is only present on ALT_ACC_EPS_SEAL platforms. Seed it with
    # zeros so the angle controller can always build a message before the first
    # frame arrives.
    self.cam_lkas_seal = {sig: 0 for sig in (
      "sig_hfquge", "sig_tqntde", "sig_hfhxif", "sig_ausukb", "sig_uipsvp",
      "sig_kqgoqc", "sig_yjvxpi", "sig_hcyhoo", "sig_fghgbq", "sig_xbjyma",
    )}
    self.cam_acc = {}
    self.cam_adas = {}
    self.esc_eps = {}
    self.esc_pcm = {}
    self.setTimeDelay = 0
    self.mrr_leading_dist = 255
    self.btn_acc_cancel = 0
    self.btn_acc_set_reset = 0
    self.btn_acc_dist_inc = 0
    self.btn_acc_dist_dec = 0
    self.modified_stock_long = False
    self.use_byd_tsr = False
    self.bsd_on_camerabus = get_bool("BydBsdType2")
    self.use_byd_low_speed_long = False
    self.lowCruiseActive = False
    self.lowCruiseSetSpeed = 30
    self.lowCruiseDeactivePending = False
    self.lowCruiseSetSpeedDisplay = 0
    self.speed_correct_30 = get_int("SpeedCorrect30") * 0.1
    self.speed_correct_60 = get_int("SpeedCorrect60") * 0.1
    self.speed_correct_90 = get_int("SpeedCorrect90") * 0.1
    self.speed_correct_120 = get_int("SpeedCorrect120") * 0.1
    self.dashspeed_fp = [0.0, self.speed_correct_30, self.speed_correct_60, self.speed_correct_90, self.speed_correct_120]

  def update_eps_state_warning(self, eps_state):
    if eps_state in (3, 5, 6, 7, 10):
      self.eps_state_warning_cnt += 1
    else:
      self.eps_state_warning_cnt = 0
    return self.eps_state_warning_cnt > Tuning.EPS_STATE_WARNING_CNT

  def update(self, can_parsers):
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    ret = structs.CarState()

    if self.frame > 500:
      self.modified_stock_long = get_bool("BydModifiedStockLong")
      self.use_byd_tsr = get_bool("BydMpcTsr")
      self.use_byd_low_speed_long = get_bool("BydLowSpdLong")
      self.frame = 0
    else:
      self.frame += 1

    self.mpc_lkas_config = int(cp_cam.vl["CID_HGZKCQ"]["sig_mfykom"])
    lkas_enabled = self.mpc_lkas_config != LKASConfig.DISABLE
    self.acc_state = int(cp_cam.vl["CID_XQYNBW"]["sig_hprcqm"])
    self.cruisemodedisp = int(cp_cam.vl["CID_XQYNBW"]["sig_ohsrtt"])
    old_gap = self.adas_set_dist
    self.adas_set_dist = int(cp_cam.vl["CID_XQYNBW"]["sig_ijrdxw"])
    if old_gap != self.adas_set_dist:
      self.adas_set_dist_changed = True
      self.adas_set_dist_changed_notified = False
    ret.pcmCruiseGap = self.adas_set_dist

    prev_btn_acc_cancel = self.btn_acc_cancel
    prev_btn_acc_set_reset = self.btn_acc_set_reset
    self.btn_acc_cancel = cp.vl["CID_YHMGPU"]["sig_ssvpvb"]
    self.btn_acc_set_reset = cp.vl["CID_YHMGPU"]["sig_ymtlod"]

    dash_speed = int(cp.vl["CID_QXPDTS"]["sig_mniqdb"])
    cluster_kph = dash_speed * CarControllerParams.K_DASHSPEED
    correction = np.interp(cluster_kph, DASHSPEED_BP, self.dashspeed_fp)
    self.speed_kph = float(cluster_kph + correction)
    ret.vEgoRaw = self.speed_kph * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.vEgoCluster = ret.vEgoRaw
    ret.standstill = dash_speed == 0

    if self.CP.minSteerSpeed > 0:
      if self.speed_kph > 0.5:
        self.lkas_allowed_speed = True
      elif self.speed_kph < 0.1:
        self.lkas_allowed_speed = False
    else:
      self.lkas_allowed_speed = True

    gear_raw = int(cp.vl["CID_PFTVEH"]["sig_wymsqb"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(gear_raw, None))
    ret.genericToggle = bool(cp.vl["CID_QMUUVA"]["sig_pjswju"])

    if self.CP.enableBsm:
      bsm_cp = cp_cam if self.bsd_on_camerabus else cp
      ret.leftBlindspot = bool(bsm_cp.vl["CID_WMSVRZ"]["sig_ebwnrw"])
      ret.rightBlindspot = bool(bsm_cp.vl["CID_WMSVRZ"]["sig_rxlfvj"])

    ret.steeringAngleOffsetDeg = Tuning.STEERING_ANGLE_OFFSET
    ret.steeringAngleDeg = cp.vl["CID_VSUYVS"]["sig_uefwof"] + Tuning.STEERING_ANGLE_OFFSET
    ret.steeringRateDeg = cp.vl["CID_VSUYVS"]["sig_ttcoap"]

    if self.CP.flags & BydFlags.ALT_INDICATOR:
      ret.leftBlinker = cp.vl["CID_QMUUVA"]["sig_cvzplw"] == 3
      ret.rightBlinker = cp.vl["CID_QMUUVA"]["sig_cvzplw"] == 5
    else:
      ret.leftBlinker = bool(cp.vl["CID_QMUUVA"]["sig_ecghww"])
      ret.rightBlinker = bool(cp.vl["CID_QMUUVA"]["sig_qofkml"])

    if self.CP.flags & BydFlags.SETSPEED_X10:
      set_speed_kph = cp_cam.vl["CID_XQYNBW"]["sig_gqnxpm"] * 10
    else:
      set_speed_kph = cp_cam.vl["CID_XQYNBW"]["sig_gqnxpm"]

    if self.CP.flags & BydFlags.BCM_SEAL:
      door_msg = cp.vl["CID_QSAUBE"]
      ret.doorOpen = any([
        door_msg["sig_lzlffi"] != 1, door_msg["sig_kftxmm"] != 1,
        door_msg["sig_ktlxyu"] != 1, door_msg["sig_jhvaxq"] != 1,
      ])
      ret.seatbeltUnlatched = door_msg["sig_kftyqu"] != 2
    else:
      door_msg = cp.vl["CID_SROETQ"]
      ret.doorOpen = any([door_msg["sig_lzlffi"], door_msg["sig_kftxmm"], door_msg["sig_ktlxyu"], door_msg["sig_jhvaxq"]])
      ret.seatbeltUnlatched = door_msg["sig_kftyqu"] != 1

    if self.CP.flags & BydFlags.ALT_PCM_BTN:
      button_events = [
        *create_button_events(self.btn_acc_cancel, prev_btn_acc_cancel, {1: ButtonType.setCruise}),
        *create_button_events(
          self.btn_acc_set_reset,
          prev_btn_acc_set_reset,
          {1: ButtonType.decelCruise, 3: ButtonType.accelCruise},
        ),
      ]
    else:
      button_events = [
        *create_button_events(self.btn_acc_cancel, prev_btn_acc_cancel, {1: ButtonType.cancel}),
        *create_button_events(
          self.btn_acc_set_reset,
          prev_btn_acc_set_reset,
          {1: ButtonType.decelCruise, 3: ButtonType.accelCruise},
        ),
      ]

    if self.CP.flags & BydFlags.ALT_ACC_EPS_SEAL:
      eps_msg = cp.vl["CID_KERMHK"]
      self.eps_state = int(eps_msg["sig_rcgbok"])
      self.eps_warning = self.update_eps_state_warning(self.eps_state)
      ret.steeringTorque = eps_msg["sig_bhnetn"]
      ret.steeringTorqueEps = eps_msg["sig_itfrci"]
      self.eps_state_counter = int(eps_msg["sig_fghgbq"])
      self.esc_eps = copy.copy(eps_msg)
    else:
      eps_msg = cp.vl["CID_IQMESB"] if "CID_IQMESB" in cp.vl else {}
      self.eps_state = int(eps_msg.get("sig_rcgbok", 0))
      self.eps_warning = self.update_eps_state_warning(self.eps_state)
      ret.steeringTorque = eps_msg.get("sig_bhnetn", 0)
      ret.steeringTorqueEps = eps_msg.get("sig_itfrci", 0)
      self.eps_state_counter = int(eps_msg.get("sig_fghgbq", 0))
      self.esc_eps = copy.copy(eps_msg)

    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > Tuning.STEER_PRESSED_THRESHOLD, 5)
    ret.brake = int(cp.vl["CID_GDWRBP"]["sig_wrfhru"])
    ret.brakePressed = ret.brake != 0
    ret.gas = int(cp.vl["CID_GDWRBP"]["sig_oefitp"])
    ret.gasPressed = ret.gas != 0

    ret.cruiseState.available = lkas_enabled and not ret.steerFaultPermanent
    ret.cruiseState.enabled = ret.cruiseState.available and self.acc_state in (3, 5)
    ret.cruiseState.speed = set_speed_kph * CV.KPH_TO_MS
    ret.cruiseState.speedCluster = ret.cruiseState.speed
    ret.stockFcw = bool(cp_cam.vl["CID_XQYNBW"]["sig_znmccg"])
    ret.stockAeb = bool(cp_cam.vl["CID_XQYNBW"]["sig_rqvixu"])

    if self.CP.carFingerprint in PT_RADAR_CAR:
      track_id = int(cp_cam.vl["CID_NCBHOL"]["sig_qfnwgn"]) if "CID_NCBHOL" in cp_cam.vl else 0
      if track_id == 2 and bool(cp_cam.vl["CID_NCBHOL"]["sig_ijbmoi"]):
        self.mrr_leading_dist = int(cp_cam.vl["CID_NCBHOL"]["sig_voxvab"])
      elif track_id == 2:
        self.mrr_leading_dist = 255
      else:
        self.mrr_leading_dist = 0

    self.acc_hud_adas_counter = cp_cam.vl["CID_XQYNBW"]["sig_fghgbq"]
    self.acc_mpc_state_counter = cp_cam.vl["CID_HGZKCQ"]["sig_fghgbq"]
    self.acc_cmd_counter = cp_cam.vl["CID_RJDCMR"]["sig_fghgbq"]
    self.cam_lkas = copy.copy(cp_cam.vl["CID_HGZKCQ"])
    if "CID_TCDUBH" in cp.vl:
      self.cam_lkas_seal = copy.copy(cp.vl["CID_TCDUBH"])
    self.cam_adas = copy.copy(cp_cam.vl["CID_XQYNBW"])
    self.cam_acc = copy.copy(cp_cam.vl["CID_RJDCMR"])
    self.esc_pcm = copy.copy(cp.vl["CID_YHMGPU"])

    if self.adas_set_dist_changed:
      button_events.append(structs.CarState.ButtonEvent(pressed=not self.adas_set_dist_changed_notified,
                                                        type=ButtonType.gapAdjustCruise))
      self.adas_set_dist_changed_notified = not self.adas_set_dist_changed_notified
      if not self.adas_set_dist_changed_notified:
        self.adas_set_dist_changed = False

    ret.buttonEvents = button_events
    return ret

  @staticmethod
  def get_can_parsers(CP):
    pt_messages = [
      ("CID_VSUYVS", 100), ("CID_QXPDTS", 50), ("CID_GDWRBP", 50),
      ("CID_PFTVEH", 50), ("CID_QMUUVA", 1), ("CID_YHMGPU", 20), ("CID_XMKCQO", 2),
    ]
    cam_messages = [("CID_XQYNBW", 50), ("CID_RJDCMR", 50), ("CID_HGZKCQ", 50)]
    if CP.enableBsm:
      if get_bool("BydBsdType2"):
        cam_messages.append(("CID_WMSVRZ", 20))
      else:
        pt_messages.append(("CID_WMSVRZ", 20))

    if CP.carFingerprint in (PLATFORM_SEAL | PLATFORM_TENGSHI | PLATFORM_QIN_SEAL06):
      pt_messages.append(("CID_KERMHK", 50))
      pt_messages.append(("CID_QSAUBE", 20))
      cam_messages.append(("CID_TCDUBH", 50))
    elif CP.carFingerprint in PLATFORM_HAN_DMI:
      pt_messages.append(("CID_IQMESB", 50))
      pt_messages.append(("CID_QSAUBE", 20))
    else:
      pt_messages.append(("CID_IQMESB", 50))
      pt_messages.append(("CID_SROETQ", 1))

    if CP.carFingerprint in PT_RADAR_CAR:
      cam_messages.append(("CID_NCBHOL", 60))

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, CanBus.ESC),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], cam_messages, CanBus.MPC),
    }
