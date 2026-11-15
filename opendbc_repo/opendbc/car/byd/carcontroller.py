
#!/usr/bin/env python3
import numpy as np
from opendbc.can.packer import CANPacker
from opendbc.car import Bus, apply_driver_steer_torque_limits, apply_std_steer_angle_limits, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd import bydcan
from opendbc.car.byd.values import (
  BydFlags, CarControllerParams, PLATFORM_SONG_PLUS_DMI, TORQUE_LAT_CAR,
)
from opendbc.car.byd.bydparams import get_bool, get_int

LongCtrlState = structs.CarControl.Actuators.LongControlState

ACC_STEP = 2
PARAM_STEP = 500
PCM_STEP = 5
STEER_STEP = 2
BYD_ACC_BUTTON_RELEASE = 0
BYD_ACC_BUTTON_DECEL = 1
BYD_ACC_BUTTON_ACCEL = 3
AUTO_STOCK_ACC_MIN_KPH = 30.0
AUTO_STOCK_ACC_MAX_KPH = 160.0
AUTO_STOCK_ACC_BUTTON_STEP_KPH = 5.0
AUTO_STOCK_ACC_MARGIN_KPH = 1.0
AUTO_STOCK_ACC_MAX_PULSES = 30
AUTO_GAS_SYNC_MARGIN_KPH = 1.0
AUTO_GAS_SYNC_MAX_PULSES = 3
AUTO_GAS_SYNC_HOLD_FRAMES = 40


class BydJerk:
  def __init__(self):
    self.jerk = 0.0
    self.jerk_u = 0.0
    self.jerk_l = 0.0
    self.cb_upper = 0.0
    self.cb_lower = 0.0

  def make_jerk(self, CS, accel, actuators):
    jerk_u = 6.0
    cb_upper = 0.6
    cb_lower = -0.6
    jerk_l = -5.0
    if actuators.longControlState == LongCtrlState.off:
      self.jerk_u = jerk_u
      self.jerk_l = jerk_l
      self.cb_upper = 0.0
      self.cb_lower = 0.0
    else:
      self.jerk_u = np.clip(accel * jerk_u / 2.5, cb_upper, jerk_u)
      self.jerk_l = np.clip(accel * jerk_l / -3.5, jerk_l, cb_lower)
      self.cb_upper = np.clip(0.09 + accel * 0.03, 0, 0.15)
      self.cb_lower = np.clip(0.08 + accel * 0.02, 0, 0.15)


# Mirrors the Seal angle limits in opendbc/safety/safety/safety_byd.h
class AngleSteeringLimits:
  STEER_ANGLE_MAX = 640
  ANGLE_RATE_LIMIT_UP = ([0., 5., 15.], [5., 0.8, 0.15])
  ANGLE_RATE_LIMIT_DOWN = ([0., 5., 15.], [5., 3.5, 0.4])


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.packer_obj = CANPacker("gm_global_a_object")
    self.params = CarControllerParams(CP)
    self.CP = CP
    self.modified_stock_long = get_bool("BydModifiedStockLong")
    self.auto_gas_sync_speed = get_bool("AutoGasSyncSpeed")
    self.speed_from_pcm = get_int("SpeedFromPCM")
    self.use_ext_radar = get_bool("EnableExtRadar")
    self.last_param_frame = 0
    self.last_acc_frame = 0
    self.last_steer_frame = 0
    self.last_pcm_frame = 0
    self.apply_torque_last = 0
    self.apply_angle_last = 0.0
    self.mpc_lkas_counter = 0
    self.mpc_acc_counter = 0
    self.eps_fake318_counter = 0
    self.mpc_pcm_counter = 0
    self.lkas_temp_override_seal = False
    self.lkas_req_prepare = False
    self.lkas_active = False
    self.lkas_max_torque = 0
    self.lat_safeoff = False
    self.steer_softstart_limit = 0
    self.first_start = True
    self.rfss = 0
    self.sss = 0
    self.byd_jerk = BydJerk()
    self.auto_stock_acc_target_kph = 0.0
    self.auto_stock_acc_restore_kph = 0.0
    self.auto_stock_acc_pulses = 0
    self.auto_stock_acc_release_frames = 0
    self.auto_stock_acc_direction = 0
    self.auto_gas_sync_gas_frames = 0
    self.auto_gas_sync_target_kph = 0.0
    self.auto_gas_sync_pulses = 0
    self.auto_gas_sync_release_frames = 0
    self.apply_accel_last = 0
    self.use_torque_lat = self.CP.carFingerprint in TORQUE_LAT_CAR
    self.send_fake_1e2 = self.CP.carFingerprint in PLATFORM_SONG_PLUS_DMI
    self.keep_lkas_passive = False
    self.angle_driver_override_release = bool(self.CP.flags & BydFlags.ANGLE_DRIVER_OVERRIDE_RELEASE)

  @staticmethod
  def _round_stock_acc_speed(speed_kph, direction):
    speed_kph = np.clip(speed_kph, AUTO_STOCK_ACC_MIN_KPH, AUTO_STOCK_ACC_MAX_KPH)
    if direction < 0:
      return np.floor(speed_kph / AUTO_STOCK_ACC_BUTTON_STEP_KPH) * AUTO_STOCK_ACC_BUTTON_STEP_KPH
    return np.ceil(speed_kph / AUTO_STOCK_ACC_BUTTON_STEP_KPH) * AUTO_STOCK_ACC_BUTTON_STEP_KPH

  def _reset_auto_stock_acc_target(self, keep_restore=False):
    self.auto_stock_acc_target_kph = 0.0
    self.auto_stock_acc_pulses = 0
    self.auto_stock_acc_release_frames = 0
    self.auto_stock_acc_direction = 0
    if not keep_restore:
      self.auto_stock_acc_restore_kph = 0.0

  def _auto_stock_acc_available(self, CC, CS, require_modified_stock_long=False):
    if require_modified_stock_long and not self.modified_stock_long:
      return False

    return (
      self.CP.pcmCruise
      and self.speed_from_pcm == 1
      and self.auto_gas_sync_speed
      and CC.enabled
      and CS.out.cruiseState.enabled
    )

  def _update_auto_stock_acc_target(self, CC, CS, hud_control):
    if not self._auto_stock_acc_available(CC, CS, require_modified_stock_long=True):
      self._reset_auto_stock_acc_target()
      return

    if not CS.lowCruiseActive and (CS.out.brakePressed or CS.out.standstill):
      self._reset_auto_stock_acc_target()
      return

    if CS.out.gasPressed:
      self._reset_auto_stock_acc_target()
      return

    current_kph = CS.out.cruiseState.speed * CV.MS_TO_KPH
    set_kph = hud_control.setSpeed * CV.MS_TO_KPH
    if current_kph <= 0 or set_kph <= 0:
      self._reset_auto_stock_acc_target()
      return

    down_target = self._round_stock_acc_speed(set_kph, -1)
    if down_target < current_kph - AUTO_STOCK_ACC_MARGIN_KPH:
      restore_target = self._round_stock_acc_speed(current_kph, 1)
      if self.auto_stock_acc_restore_kph <= 0 or restore_target > self.auto_stock_acc_restore_kph:
        self.auto_stock_acc_restore_kph = restore_target

      target_changed = abs(self.auto_stock_acc_target_kph - down_target) > AUTO_STOCK_ACC_MARGIN_KPH
      if self.auto_stock_acc_direction != -1 or target_changed:
        self.auto_stock_acc_target_kph = down_target
        self.auto_stock_acc_pulses = 0
        self.auto_stock_acc_release_frames = 0
        self.auto_stock_acc_direction = -1
      return

    if self.auto_stock_acc_direction < 0:
      if current_kph <= self.auto_stock_acc_target_kph + AUTO_STOCK_ACC_MARGIN_KPH:
        self._reset_auto_stock_acc_target(keep_restore=True)

      if self.auto_stock_acc_restore_kph > 0:
        if current_kph >= self.auto_stock_acc_restore_kph - AUTO_STOCK_ACC_MARGIN_KPH:
          self._reset_auto_stock_acc_target()
          return

        if set_kph > current_kph + AUTO_STOCK_ACC_MARGIN_KPH:
          up_target = min(self.auto_stock_acc_restore_kph, self._round_stock_acc_speed(set_kph, 1))
          if up_target <= current_kph + AUTO_STOCK_ACC_MARGIN_KPH:
            return

          target_changed = abs(self.auto_stock_acc_target_kph - up_target) > AUTO_STOCK_ACC_MARGIN_KPH
          if self.auto_stock_acc_direction != 1 or target_changed:
            self.auto_stock_acc_target_kph = up_target
            self.auto_stock_acc_pulses = 0
            self.auto_stock_acc_release_frames = 0
            self.auto_stock_acc_direction = 1
      return

  def _get_auto_stock_acc_button(self, CS):
    if self.auto_stock_acc_target_kph <= 0 or self.auto_stock_acc_direction == 0:
      return BYD_ACC_BUTTON_RELEASE
    if CS.esc_pcm.get("sig_ymtlod", 0) != 0 or CS.esc_pcm.get("sig_ssvpvb", 0) != 0:
      self._reset_auto_stock_acc_target()
      return BYD_ACC_BUTTON_RELEASE

    current_kph = CS.out.cruiseState.speed * CV.MS_TO_KPH
    if self.auto_stock_acc_direction < 0:
      if current_kph <= self.auto_stock_acc_target_kph + AUTO_STOCK_ACC_MARGIN_KPH:
        self._reset_auto_stock_acc_target(keep_restore=True)
        return BYD_ACC_BUTTON_RELEASE
      button = BYD_ACC_BUTTON_DECEL
    else:
      if current_kph >= self.auto_stock_acc_target_kph - AUTO_STOCK_ACC_MARGIN_KPH:
        keep_restore = self.auto_stock_acc_restore_kph > self.auto_stock_acc_target_kph + AUTO_STOCK_ACC_MARGIN_KPH
        self._reset_auto_stock_acc_target(keep_restore=keep_restore)
        return BYD_ACC_BUTTON_RELEASE
      button = BYD_ACC_BUTTON_ACCEL

    if self.auto_stock_acc_pulses >= AUTO_STOCK_ACC_MAX_PULSES:
      self._reset_auto_stock_acc_target()
      return BYD_ACC_BUTTON_RELEASE
    if self.auto_stock_acc_release_frames > 0:
      self.auto_stock_acc_release_frames -= 1
      return BYD_ACC_BUTTON_RELEASE

    self.auto_stock_acc_pulses += 1
    self.auto_stock_acc_release_frames = 1
    return button

  def _reset_auto_gas_sync(self):
    self.auto_gas_sync_gas_frames = 0
    self.auto_gas_sync_target_kph = 0.0
    self.auto_gas_sync_pulses = 0
    self.auto_gas_sync_release_frames = 0

  def _update_auto_gas_sync_target(self, CC, CS):
    if not self._auto_stock_acc_available(CC, CS):
      self._reset_auto_gas_sync()
      return

    if not CS.lowCruiseActive and (CS.out.brakePressed or CS.out.standstill):
      self._reset_auto_gas_sync()
      return

    if not CS.out.gasPressed:
      self._reset_auto_gas_sync()
      return

    self.auto_gas_sync_gas_frames += 1
    if self.auto_gas_sync_gas_frames <= AUTO_GAS_SYNC_HOLD_FRAMES:
      return

    current_kph = CS.out.cruiseState.speed * CV.MS_TO_KPH
    ego_kph = max(CS.out.vEgoCluster, CS.out.vEgoRaw, CS.out.vEgo) * CV.MS_TO_KPH
    if current_kph <= 0 or ego_kph <= current_kph + AUTO_GAS_SYNC_MARGIN_KPH:
      return

    target_kph = min(ego_kph, AUTO_STOCK_ACC_MAX_KPH)
    if target_kph > self.auto_gas_sync_target_kph + AUTO_GAS_SYNC_MARGIN_KPH:
      self.auto_gas_sync_target_kph = target_kph
      self.auto_gas_sync_pulses = 0
      self.auto_gas_sync_release_frames = 0

  def _get_auto_gas_sync_button(self, CS):
    if self.auto_gas_sync_target_kph <= 0:
      return BYD_ACC_BUTTON_RELEASE

    if not CS.out.gasPressed:
      self._reset_auto_gas_sync()
      return BYD_ACC_BUTTON_RELEASE

    if CS.esc_pcm.get("sig_ymtlod", 0) != 0 or CS.esc_pcm.get("sig_ssvpvb", 0) != 0:
      self._reset_auto_gas_sync()
      return BYD_ACC_BUTTON_RELEASE

    current_kph = CS.out.cruiseState.speed * CV.MS_TO_KPH
    if current_kph >= self.auto_gas_sync_target_kph - AUTO_GAS_SYNC_MARGIN_KPH:
      self._reset_auto_gas_sync()
      return BYD_ACC_BUTTON_RELEASE

    if self.auto_gas_sync_pulses >= AUTO_GAS_SYNC_MAX_PULSES:
      self._reset_auto_gas_sync()
      return BYD_ACC_BUTTON_RELEASE

    if self.auto_gas_sync_release_frames > 0:
      self.auto_gas_sync_release_frames -= 1
      return BYD_ACC_BUTTON_RELEASE

    self.auto_gas_sync_pulses += 1
    self.auto_gas_sync_release_frames = 1
    return BYD_ACC_BUTTON_DECEL

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    can_sends = []

    if self.frame - self.last_param_frame >= PARAM_STEP:
      self.modified_stock_long = get_bool("BydModifiedStockLong")
      self.auto_gas_sync_speed = get_bool("AutoGasSyncSpeed")
      self.speed_from_pcm = get_int("SpeedFromPCM")
      self.use_ext_radar = get_bool("EnableExtRadar")
      self.last_param_frame = self.frame

    lat_active = CC.latActive
    long_active = CC.longActive

    new_actuators = actuators.as_builder()
    apply_accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    self.byd_jerk.make_jerk(CS, apply_accel, actuators)
    self._update_auto_gas_sync_target(CC, CS)
    self._update_auto_stock_acc_target(CC, CS, hud_control)

    if self.use_torque_lat:
      apply_torque = int(round(actuators.torque * self.params.STEER_MAX))
      apply_torque = apply_driver_steer_torque_limits(apply_torque, self.apply_torque_last, CS.out.steeringTorque, self.params)
      if not lat_active:
        apply_torque = 0
      self.apply_torque_last = apply_torque
      new_actuators.torque = apply_torque / self.params.STEER_MAX
    else:
      requested_angle = actuators.steeringAngleDeg
      requested_angle = apply_std_steer_angle_limits(requested_angle, self.apply_angle_last, CS.out.vEgoRaw,
                                                       CS.out.steeringAngleDeg, lat_active, AngleSteeringLimits)
      if not lat_active:
        requested_angle = CS.out.steeringAngleDeg
      self.apply_angle_last = requested_angle
      new_actuators.steeringAngleDeg = requested_angle
      apply_torque = 0

    if self.frame % STEER_STEP == 0:
      if self.use_torque_lat:
        can_sends.append(bydcan.create_steering_control(
          self.packer, self.CP, CS.cam_lkas, apply_torque, self.lkas_req_prepare,
          lat_active, hud_control, self.keep_lkas_passive, self.mpc_lkas_counter,
        ))
      else:
        max_torque = CarControllerParams.ANGLE_MAX_TORQUE
        can_sends.append(bydcan.create_angle_control(
          self.packer, self.CP, CS.cam_lkas_seal, self.apply_angle_last,
          max_torque, lat_active, self.lkas_req_prepare, self.mpc_lkas_counter,
        ))
      self.mpc_lkas_counter = (self.mpc_lkas_counter + 1) % 16

    if self.frame % ACC_STEP == 0:
      if self.modified_stock_long:
        can_sends.append(bydcan.acc_cmd_modified_stock_long(
          self.packer, self.CP, CS.cam_acc, CS.adas_set_dist, CS.mrr_leading_dist,
          long_active, self.mpc_acc_counter,
        ))
      else:
        can_sends.append(bydcan.acc_cmd(
          self.packer, self.CP, CS.cam_acc, self.byd_jerk, apply_accel,
          self.rfss, self.sss, long_active, 3 if long_active else 0, self.mpc_acc_counter,
        ))
      self.mpc_acc_counter = (self.mpc_acc_counter + 1) % 16

    if self.frame % PCM_STEP == 0 and CS.esc_pcm:
      updown_cmd = self._get_auto_stock_acc_button(CS)
      if updown_cmd == BYD_ACC_BUTTON_RELEASE:
        updown_cmd = self._get_auto_gas_sync_button(CS)

      can_sends.append(bydcan.create_mpc_pcm_button(
        self.packer, CS.esc_pcm, CS.lowCruiseActive and CS.out.cruiseState.enabled,
        self.mpc_pcm_counter, updown_cmd,
      ))
      self.mpc_pcm_counter = (self.mpc_pcm_counter + 1) % 16

    if self.frame % 2 == 0:
      can_sends.append(bydcan.create_adas_hud(
        self.packer, CS.cam_adas, CS.lowCruiseSetSpeedDisplay or 0, bool(CS.lowCruiseActive),
        CS.acc_hud_adas_counter,
      ))

    new_actuators.accel = apply_accel
    self.apply_accel_last = apply_accel
    self.frame += 1
    return new_actuators, can_sends
