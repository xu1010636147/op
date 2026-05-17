#!/usr/bin/env python3
import math
from typing import SupportsFloat
from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog
import numpy as np
from collections import deque
from opendbc.car.car_helpers import interfaces
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature, get_lag_adjusted_curvature
from openpilot.selfdrive.controls.lib.latcontrol import LatControl, MIN_LATERAL_CONTROL_SPEED
from openpilot.selfdrive.controls.lib.latcontrol_pid import LatControlPID
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle, STEER_ANGLE_SATURATION_THRESHOLD
from openpilot.selfdrive.controls.lib.latcontrol_torque import LatControlTorque
from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from selfdrive.modeld.modeld import LAT_SMOOTH_SECONDS
State = log.SelfdriveState.OpenpilotState
LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
ACTUATOR_FIELDS = tuple(car.CarControl.Actuators.schema.fields.keys())
class Controls:
  def __init__(self) -> None:
    self.params = Params()
    cloudlog.info("controlsd is waiting for CarParams")
    self.CP = messaging.log_from_bytes(self.params.get("CarParams", block=True), car.CarParams)
    cloudlog.info("controlsd got CarParams")
    self.CI = interfaces[self.CP.carFingerprint](self.CP)
    self.disable_dm = False
    self.sm = messaging.SubMaster(['liveParameters', 'liveTorqueParameters', 'modelV2', 'selfdriveState',
                                   'liveCalibration', 'liveLocationKalman', 'longitudinalPlan', 'carState', 'carOutput',
                                   'liveDelay', 'carrotMan', 'lateralPlan', 'radarState',
                                   'driverMonitoringState', 'onroadEvents', 'driverAssistance', 'accelerometer'], poll='selfdriveState')
    self.pm = messaging.PubMaster(['carControl', 'controlsState'])
    self.steer_limited_by_controls = False
    self.curvature = 0.0
    self.desired_curvature = 0.0
    self.yStd = 0.0
    self.side_state = {
        "left":  {"main": {"dRel": None, "lat": None}, "sub": {"dRel": None, "lat": None}},
        "right": {"main": {"dRel": None, "lat": None}, "sub": {"dRel": None, "lat": None}},
    }
    self.LoC = LongControl(self.CP)
    self.VM = VehicleModel(self.CP)
    self.LaC: LatControl
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      self.LaC = LatControlAngle(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'pid':
      self.LaC = LatControlPID(self.CP, self.CI)
    elif self.CP.lateralTuning.which() == 'torque':
      self.LaC = LatControlTorque(self.CP, self.CI)
    self._set_speed_prev = 0.0
    self._use_stock_long_prev = None
    self._use_stock_long = False
    self.static_lead_counter = 0
    self.cut_in_lead = None
    self.last_accel = 0.0
    self.traffic_jam_stop = False
    self.speed_stable_counter = 0
    self.switch_hysteresis_counter = 0
    self.SWITCH_HYSTERESIS_FRAMES = 50
    self.CURVE_TRIGGER_CURVATURE = 0.0015
    self.CURVE_PRE_SWITCH_DISTANCE = 40.0
    # 新增：文档2静态识别相关参数
    self.CURVATURE_HYSTERESIS_ON = 0.002    # 高速弯道触发阈值（文档2）
    self.CURVATURE_HYSTERESIS_OFF = 0.0008  # 退出弯道阈值（文档2）
    self.MIN_SPEED_STATIC_DETECT = 10.0 * CV.KPH_TO_MS  # 静态识别最小车速（10km/h，文档2）
    self.STATIC_CONFIRM_FRAMES_FCW = 30     # FCW触发时确认帧数（0.3s，文档2）
    self.STATIC_CONFIRM_FRAMES_NO_FCW = 60  # 无FCW时确认帧数（0.6s，文档2）

  def update(self):
    self.sm.update(15)

  # 替换：文档2的增强版静止目标判定（雷达+视觉+FCW三重确认）
  def _is_static_lead(self, leadOne, CS):
    """判定前方目标物完全静止且有碰撞风险（整合文档2逻辑）"""
    # 条件0：前车存在
    if not leadOne.status:
      self.static_lead_counter = 0
      return False
    
    # 条件1：本车车速≥10km/h（避免低速误报）
    if CS.vEgo < self.MIN_SPEED_STATIC_DETECT:
      self.static_lead_counter = 0
      return False
    
    # 条件2：距离≤100m
    if leadOne.dRel > 100.0:
      self.static_lead_counter = 0
      return False
    
    # 条件3：前车绝对速度<1km/h
    lead_abs_speed = (CS.vEgo + leadOne.vRel) * CV.MS_TO_KPH
    if lead_abs_speed >= 1.0:
      self.static_lead_counter = 0
      return False
    
    # 条件4：雷达加速度接近0（|aLeadK| < 0.2 m/s²）
    low_acceleration = abs(leadOne.aLeadK) < 0.2 if hasattr(leadOne, 'aLeadK') else True
    
    # 条件5：视觉模型确认（文档2核心逻辑）
    model_v2 = self.sm['modelV2']
    model_leads = model_v2.leadsV3 if hasattr(model_v2, 'leadsV3') else []
    vision_confirmed = False
    if len(model_leads) > 0:
      model_lead = model_leads[0]
      vision_high_prob = model_lead.prob > 0.8 if hasattr(model_lead, 'prob') else True
      # 视觉与雷达距离匹配（误差<5m，1.52m为雷达-相机偏移）
      vision_distance_match = abs(model_lead.x[0] - (leadOne.dRel + 1.52)) < 5.0
      vision_confirmed = vision_high_prob and vision_distance_match
    else:
      vision_confirmed = True  # 无视觉数据时不依赖
    
    # 条件6：雷达目标有效性
    is_radar_target = leadOne.radar if hasattr(leadOne, 'radar') else True
    
    # 条件7：FCW碰撞风险确认（文档2核心逻辑）
    fcw_warning = leadOne.fcw if hasattr(leadOne, 'fcw') else False
    model_hard_brake = model_v2.meta.hardBrakePredicted if hasattr(model_v2.meta, 'hardBrakePredicted') else False
    fcw_confirmed = fcw_warning or model_hard_brake
    
    # 计数逻辑（文档2滞回优化）
    if low_acceleration and vision_confirmed and is_radar_target:
      if fcw_confirmed:
        self.static_lead_counter += 1
        return self.static_lead_counter >= self.STATIC_CONFIRM_FRAMES_FCW
      else:
        self.static_lead_counter += 1
        return self.static_lead_counter >= self.STATIC_CONFIRM_FRAMES_NO_FCW
    else:
      self.static_lead_counter = max(0, self.static_lead_counter - 2)  # 缓慢递减
      return False

  # 保留文档1原有加塞识别（20m内）
  def _is_cut_in(self, radarState, CS):
    cut_in_detected = False
    for side in ["left", "right"]:
        side_leads = radarState.leadsLeft2 if side == "left" else radarState.leadsRight2
        if side_leads:
            nearest_lead = side_leads[0]
            if nearest_lead.dRel < 20.0 and nearest_lead.vRel > 0.1:
                self.cut_in_lead = nearest_lead
                cut_in_detected = True
                break
    return cut_in_detected

  # 保留文档1堵车停止判定
  def _is_traffic_jam_stop(self, CS, leadOne):
    if CS.standstill and leadOne.status:
        self.traffic_jam_stop = True
        return True
    return False

  def state_control(self):
    CS = self.sm['carState']
    # Update VehicleModel
    lp = self.sm['liveParameters']
    x = max(lp.stiffnessFactor, 0.1)
    sr = max(lp.steerRatio, 0.1) * self.params.get_float("SteerRatioRate") / 100.0
    custom_sr = self.params.get_float("CustomSR") / 10.0
    sr = max(custom_sr if custom_sr > 1.0 else sr, 0.1)
    self.VM.update_params(x, sr)
    steer_angle_without_offset = math.radians(CS.steeringAngleDeg - lp.angleOffsetDeg)
    self.curvature = -self.VM.calc_curvature(steer_angle_without_offset, CS.vEgo, lp.roll)
    
    # Update Torque Params
    if self.CP.lateralTuning.which() == 'torque':
      torque_params = self.sm['liveTorqueParameters']
      if self.sm.all_checks(['liveTorqueParameters']) and torque_params.useParams:
        self.LaC.update_live_torque_params(torque_params.latAccelFactorFiltered, torque_params.latAccelOffsetFiltered,
                                           torque_params.frictionCoefficientFiltered)
    
    long_plan = self.sm['longitudinalPlan']
    model_v2 = self.sm['modelV2']
    radarState = self.sm['radarState']
    leadOne = radarState.leadOne
    lat_plan = self.sm['lateralPlan']
    CC = car.CarControl.new_message()
    CC.enabled = self.sm['selfdriveState'].enabled
    
    # carrot
    gear = car.CarState.GearShifter
    driving_gear = CS.gearShifter not in (gear.neutral, gear.park, gear.reverse, gear.unknown)
    lateral_enabled = driving_gear
    standstill = abs(CS.vEgo) <= max(self.CP.minSteerSpeed, MIN_LATERAL_CONTROL_SPEED) or CS.standstill
    CC.latActive = ((self.sm['selfdriveState'].active or lateral_enabled) and CS.latEnabled and
                    not CS.steerFaultTemporary and not CS.steerFaultPermanent and not standstill)
    
    # 核心修改：整合文档2触发条件（保留文档1场景+新增高速弯道）
    ###########################################################################
    # 1. 场景条件（文档1+文档2融合）
    static_lead = self._is_static_lead(leadOne, CS) if leadOne.status else False  # 替换为文档2逻辑
    cut_in = self._is_cut_in(radarState, CS)  # 保留文档1加塞
    traffic_jam_stop = self._is_traffic_jam_stop(CS, leadOne)  # 保留文档1堵车
    low_speed_stop = CS.vEgo < 0.1  # 文档2本车静止条件
    
    # 弯道条件（融合双逻辑）：文档1预切换 + 文档2高速弯道
    high_curvature = abs(self.curvature) >= self.CURVE_TRIGGER_CURVATURE
    high_speed_curve = (abs(self.curvature) >= self.CURVATURE_HYSTERESIS_ON) and (CS.vEgo * CV.MS_TO_KPH > 35.0)  # 文档2高速弯道
    no_lead_near = not leadOne.status or leadOne.dRel >= 40.0
    
    # 弯道预切换（文档1）
    curve_approaching = False
    if lat_plan.useLaneLines and len(lat_plan.curvatures) > 0 and len(lat_plan.distances) > 0:
      curve_start_distance = lat_plan.distances[0]
      curve_curvature = lat_plan.curvatures[0]
      curve_approaching = (curve_start_distance <= self.CURVE_PRE_SWITCH_DISTANCE and 
                          abs(curve_curvature) >= self.CURVE_TRIGGER_CURVATURE)
    
    # 2. OP纵向触发：文档1所有场景 + 文档2高速弯道/本车静止
    trigger_op_long = static_lead or cut_in or traffic_jam_stop or low_speed_stop or \
                      (high_curvature and no_lead_near) or curve_approaching or high_speed_curve
    
    # 3. 切回原车ACC条件（融合文档2逻辑）
    switch_back_stock = False
    if not self._use_stock_long_prev:
      target_lead = self.cut_in_lead if self.cut_in_lead else leadOne
      speed_match = False
      if target_lead and target_lead.status:
          target_speed = CS.vEgo - target_lead.vRel
          speed_diff = abs(CS.vEgo - target_speed)
          speed_match = speed_diff <= 1 * CV.KPH_TO_MS
        
      jam_speed_up = self.traffic_jam_stop and CS.vEgo >= 5 * CV.KPH_TO_MS
      low_curvature = abs(self.curvature) < self.CURVATURE_HYSTERESIS_OFF  # 文档2退出弯道阈值
      speed_stable = abs(self.last_accel) <= 0.1
      if speed_stable:
          self.speed_stable_counter += 1
      else:
          self.speed_stable_counter = 0
      speed_stable = self.speed_stable_counter >= 30
      has_lead_near = leadOne.status and leadOne.dRel < 40.0
      
      # 文档2新增：前车移动判定
      lead_moving = False
      if leadOne.status:
        lead_abs_speed = (CS.vEgo + leadOne.vRel) * CV.MS_TO_KPH
        lead_accel = leadOne.aLeadK if hasattr(leadOne, 'aLeadK') else 0
        lead_moving = (lead_abs_speed > 1.0) or (lead_accel > 0.1)
      
      # 文档2新增：FCW清除判定
      fcw_cleared = True
      if hasattr(leadOne, 'fcw'):
        fcw_cleared = not leadOne.fcw
      if hasattr(model_v2.meta, 'hardBrakePredicted'):
        fcw_cleared = fcw_cleared and not model_v2.meta.hardBrakePredicted
      
      # 切回条件：文档1原有 + 文档2前车移动/FCW清除
      switch_back_stock = speed_match or jam_speed_up or low_curvature or speed_stable or has_lead_near or (lead_moving and fcw_cleared)
      
      if jam_speed_up:
          self.traffic_jam_stop = False
          self.cut_in_lead = None
    
    # 4. 保留文档1滞回逻辑（避免频繁切换）
    if trigger_op_long and not self._use_stock_long_prev:
        self.switch_hysteresis_counter = 0
    elif switch_back_stock and self._use_stock_long_prev:
        self.switch_hysteresis_counter = 0
    elif trigger_op_long:
        self.switch_hysteresis_counter += 1
        if self.switch_hysteresis_counter >= self.SWITCH_HYSTERESIS_FRAMES:
            self._use_stock_long = False
            self.switch_hysteresis_counter = 0
    elif switch_back_stock:
        self.switch_hysteresis_counter += 1
        if self.switch_hysteresis_counter >= self.SWITCH_HYSTERESIS_FRAMES:
            self._use_stock_long = True
            self.static_lead_counter = 0
            self.cut_in_lead = None
            self.speed_stable_counter = 0
            self.switch_hysteresis_counter = 0
    else:
        self._use_stock_long = self._use_stock_long_prev
        self.switch_hysteresis_counter = 0
    
    if self._use_stock_long_prev is None:
        self._use_stock_long = True
        self._use_stock_long_prev = self._use_stock_long
    self._use_stock_long_prev = self._use_stock_long
    ###########################################################################
    
    CC.longActive = (CC.enabled
                     and not any(e.overrideLongitudinal for e in self.sm['onroadEvents'])
                     and self.CP.openpilotLongitudinalControl
                     and not self._use_stock_long)
    
    actuators = CC.actuators
    actuators.longControlState = self.LoC.long_control_state
    
    # Enable blinkers while lane changing
    if model_v2.meta.laneChangeState != LaneChangeState.off:
      CC.leftBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.left
      CC.rightBlinker = model_v2.meta.laneChangeDirection == LaneChangeDirection.right
    
    if not CC.latActive:
      self.LaC.reset()
    if not CC.longActive:
      self.LoC.reset()
    
    # accel PID loop
    pid_accel_limits = self.CI.get_pid_accel_limits(self.CP, CS.vEgo, CS.vCruise * CV.KPH_TO_MS)
    t_since_plan = (self.sm.frame - self.sm.recv_frame['longitudinalPlan']) * DT_CTRL
    accel, aTarget, jerk = self.LoC.update(CC.longActive, CS, long_plan, pid_accel_limits, t_since_plan, self.sm['radarState'])
    
    # 平顺性优化（保留文档1）
    if CC.longActive:
        max_accel_delta = 0.2
        accel_delta = np.clip(accel - self.last_accel, -max_accel_delta, max_accel_delta)
        accel = self.last_accel + accel_delta
        self.last_accel = accel
    else:
        self.last_accel = 0.0
    
    actuators.accel = float(accel)
    actuators.aTarget = float(aTarget)
    actuators.jerk = float(jerk)
    
    if not CC.longActive:
      actuators.accel = 0.0
      actuators.aTarget = 0.0
      actuators.jerk = 0.0
    
    # Steering PID loop and lateral MPC
    curve_speed_abs = abs(self.sm['carrotMan'].vTurnSpeed)
    self.lanefull_mode_enabled = (lat_plan.useLaneLines and curve_speed_abs > self.params.get_int("UseLaneLineCurveSpeed"))
    lat_smooth_seconds = LAT_SMOOTH_SECONDS
    steer_actuator_delay = self.params.get_float("SteerActuatorDelay") * 0.01
    if steer_actuator_delay == 0.0:
      steer_actuator_delay = self.sm['liveDelay'].lateralDelay 
    if len(model_v2.position.yStd) > 0:
      yStd = np.interp(steer_actuator_delay + lat_smooth_seconds, ModelConstants.T_IDXS, model_v2.position.yStd)
      self.yStd = yStd * 0.02 + self.yStd * 0.98
    else:
      self.yStd = 0.0
    
    if not CC.latActive:
      new_desired_curvature = self.curvature
    elif self.lanefull_mode_enabled:
      if len(lat_plan.curvatures) == 0:
        new_desired_curvature = self.curvature
      else:
        def smooth_value(val, prev_val, tau):
          alpha = 1 - np.exp(-DT_CTRL / tau) if tau > 0 else 1
          return alpha * val + (1 - alpha) * prev_val
        curvature = get_lag_adjusted_curvature(self.CP, CS.vEgo, lat_plan.psis, lat_plan.curvatures, steer_actuator_delay + lat_smooth_seconds, lat_plan.distances)
        new_desired_curvature = smooth_value(curvature, self.desired_curvature, lat_smooth_seconds)
    else:
      new_desired_curvature = model_v2.action.desiredCurvature
    
    self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, new_desired_curvature, lp.roll)
    actuators.curvature = float(self.desired_curvature)
    steer, steeringAngleDeg, lac_log = self.LaC.update(CC.latActive, CS, self.VM, lp,
                                                       self.steer_limited_by_controls, self.desired_curvature,
                                                       self.sm['liveLocationKalman'], curvature_limited,
                                                       model_data=self.sm['modelV2'])
    actuators.torque = float(steer)
    actuators.steeringAngleDeg = float(steeringAngleDeg)
    actuators.yStd = float(self.yStd)
    
    # Ensure no NaNs/Infs
    for p in ACTUATOR_FIELDS:
      attr = getattr(actuators, p)
      if not isinstance(attr, SupportsFloat):
        continue
      if not math.isfinite(attr):
        cloudlog.error(f"actuators.{p} not finite {actuators.to_dict()}")
        setattr(actuators, p, 0.0)
    
    return CC, lac_log

  # 以下保留文档1原有方法（_update_side、publish、run、main）
  def _update_side(self, side: str, leads2, road_edge, bsd_state, hudControl):
      def ema(prev, curr, a=0.02):
        return curr if prev is None else prev * (1 - a) + curr * a
      def set_hud(side_cap, name, val):
        setattr(hudControl, f"lead{side_cap}{name}", float(val if val is not None else 0.0))
        
      st = self.side_state[side]
      if road_edge <= 2.0 or not leads2:
        st["main"] = {"dRel": None, "lat": None}
        st["sub"]  = {"dRel": None, "lat": None}
        if not bsd_state:
          return
      lead_main = leads2[0] if len(leads2) > 0 else None
      side_cap = side.capitalize()
      if bsd_state:
        set_hud(side_cap, "Dist2", 1)
        set_hud(side_cap, "Lat2",  3.2)
      elif len(leads2) > 1 and lead_main.dRel < 10:
        st["sub"]["dRel"] = ema(st["sub"]["dRel"], lead_main.dRel)
        st["sub"]["lat"]  = ema(st["sub"]["lat"],  abs(lead_main.dPath))
        set_hud(side_cap, "Dist2", st["sub"]["dRel"])
        set_hud(side_cap, "Lat2",  st["sub"]["lat"])
        lead_main = leads2[1]
      if len(leads2) > 0:
        st["main"]["dRel"] = ema(st["main"]["dRel"], lead_main.dRel)
        st["main"]["lat"]  = ema(st["main"]["lat"],  abs(lead_main.dPath))
        set_hud(side_cap, "Dist", st["main"]["dRel"])
        set_hud(side_cap, "Lat",  st["main"]["lat"])

  def publish(self, CC, lac_log):
    CS = self.sm['carState']
    orientation_value = list(self.sm['liveLocationKalman'].calibratedOrientationNED.value)
    if len(orientation_value) > 2:
      CC.orientationNED = orientation_value
    angular_rate_value = list(self.sm['liveLocationKalman'].angularVelocityCalibrated.value)
    if len(angular_rate_value) > 2:
      CC.angularVelocity = angular_rate_value
    acceleration_value = list(self.sm['liveLocationKalman'].accelerationCalibrated.value)
    if len(acceleration_value) > 2:
      if abs(acceleration_value[0]) > 16.0:
        print("Collision detected. disable openpilot, restart")
        self.params.put_bool("OpenpilotEnabledToggle", False)
        self.params.put_int("SoftRestartTriggered", 1)
    CC.cruiseControl.override = CC.enabled and not CC.longActive and self.CP.openpilotLongitudinalControl
    CC.cruiseControl.cancel = CS.cruiseState.enabled and (not CC.enabled or not self.CP.pcmCruise)
    desired_kph = min(CS.vCruiseCluster, self.sm['carrotMan'].desiredSpeed)
    setSpeed = float(desired_kph * CV.KPH_TO_MS)
    speeds = self.sm['longitudinalPlan'].speeds
    if len(speeds):
      CC.cruiseControl.resume = CC.enabled and CS.cruiseState.standstill and speeds[-1] > 0.1
      vCluRatio = CS.vCluRatio if CS.vCluRatio > 0.5 else 1.0
      plan_speed = float(speeds[-1] / vCluRatio)
      use_stock_long = self._use_stock_long
      if use_stock_long:
        lp = self.sm['longitudinalPlan']
        try:
          stop_cap = bool(lp.shouldStop)
        except Exception:
          stop_cap = False
        try:
          v_target_now = float(lp.vTargetNow)
        except Exception:
          v_target_now = plan_speed
        v_target_cap = 0.0 if stop_cap else float(v_target_now / vCluRatio)
        plan_speed = min(plan_speed, v_target_cap)
        if plan_speed > self._set_speed_prev:
          setSpeed = min(plan_speed, self._set_speed_prev + 0.3)
        else:
          setSpeed = plan_speed
      else:
        if self._use_stock_long_prev:
            setSpeed = np.clip(setSpeed, self._set_speed_prev - 0.3, self._set_speed_prev + 0.3)
        setSpeed = plan_speed
    hudControl = CC.hudControl
    hudControl.activeCarrot = self.sm['carrotMan'].activeCarrot
    hudControl.atcDistance = self.sm['carrotMan'].xDistToTurn
    lp = self.sm['longitudinalPlan']
    if self.CP.pcmCruise:
      speed_from_pcm = self.params.get_int("SpeedFromPCM")
      if speed_from_pcm == 1:
        hudControl.setSpeed = float(CS.vCruiseCluster * CV.KPH_TO_MS)
      elif speed_from_pcm == 2:
        hudControl.setSpeed = float(max(30/3.6, desired_kph * CV.KPH_TO_MS))
      elif speed_from_pcm == 3:
        hudControl.setSpeed = setSpeed if lp.xState == 3 else float(desired_kph * CV.KPH_TO_MS)
      else:
        hudControl.setSpeed = float(max(30/3.6, setSpeed))
    else:
      hudControl.setSpeed = setSpeed if lp.xState == 3 else float(desired_kph * CV.KPH_TO_MS)
    self._set_speed_prev = float(hudControl.setSpeed)
    hudControl.speedVisible = CC.enabled
    hudControl.lanesVisible = CC.enabled
    hudControl.leadVisible = self.sm['longitudinalPlan'].hasLead
    hudControl.leadDistanceBars = self.sm['selfdriveState'].personality.raw + 1
    hudControl.visualAlert = self.sm['selfdriveState'].alertHudVisual
    radarState = self.sm['radarState']
    leadOne = radarState.leadOne
    hudControl.leadDistance = leadOne.dRel if leadOne.status else 0
    hudControl.leadRelSpeed = leadOne.vRel if leadOne.status else 0
    hudControl.leadRadar = 1 if leadOne.radar else 0
    hudControl.leadDPath = leadOne.dPath
    meta = self.sm['modelV2'].meta
    hudControl.modelDesire = 1 if meta.desire == log.Desire.turnLeft else 2 if meta.desire == log.Desire.turnRight else 0
    self._update_side("left",  radarState.leadsLeft2,  meta.distanceToRoadEdgeLeft,  CS.leftBlindspot, hudControl)
    self._update_side("right", radarState.leadsRight2, meta.distanceToRoadEdgeRight, CS.rightBlindspot, hudControl)
    hudControl.rightLaneVisible = True
    hudControl.leftLaneVisible = True
    if self.sm.valid['driverAssistance']:
      hudControl.leftLaneDepart = self.sm['driverAssistance'].leftLaneDeparture
      hudControl.rightLaneDepart = self.sm['driverAssistance'].rightLaneDeparture
    if self.sm['selfdriveState'].active:
      CO = self.sm['carOutput']
      if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
        self.steer_limited_by_controls = abs(CC.actuators.steeringAngleDeg - CO.actuatorsOutput.steeringAngleDeg) > \
                                              STEER_ANGLE_SATURATION_THRESHOLD
      else:
        self.steer_limited_by_controls = abs(CC.actuators.torque - CO.actuatorsOutput.torque) > 1e-2
    dat = messaging.new_message('controlsState')
    dat.valid = CS.canValid
    cs = dat.controlsState
    cs.curvature = self.curvature
    cs.longitudinalPlanMonoTime = self.sm.logMonoTime['longitudinalPlan']
    cs.lateralPlanMonoTime = self.sm.logMonoTime['modelV2']
    cs.desiredCurvature = self.desired_curvature
    cs.longControlState = self.LoC.long_control_state
    cs.upAccelCmd = float(self.LoC.pid.p)
    cs.uiAccelCmd = float(self.LoC.pid.i)
    cs.ufAccelCmd = float(self.LoC.pid.f)
    cs.forceDecel = bool((self.sm['driverMonitoringState'].awarenessStatus < 0. and self.params.get_int("DisableDM") == 0) or
                         (self.sm['selfdriveState'].state == State.softDisabling))
    lat_tuning = self.CP.lateralTuning.which()
    if self.CP.steerControlType == car.CarParams.SteerControlType.angle:
      cs.lateralControlState.angleState = lac_log
    elif lat_tuning == 'pid':
      cs.lateralControlState.pidState = lac_log
    elif lat_tuning == 'torque':
      cs.lateralControlState.torqueState = lac_log
    cs.activeLaneLine = self.lanefull_mode_enabled
    self.pm.send('controlsState', dat)
    cc_send = messaging.new_message('carControl')
    cc_send.valid = CS.canValid
    cc_send.carControl = CC
    self.pm.send('carControl', cc_send)

  def run(self):
    rk = Ratekeeper(100, print_delay_threshold=None)
    while True:
      self.update()
      CC, lac_log = self.state_control()
      self.publish(CC, lac_log)
      rk.monitor_time()

def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  controls = Controls()
  controls.run()

if __name__ == "__main__":
  main()

