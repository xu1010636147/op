#!/usr/bin/env python3
"""
现代汽车自动超车控制器
集成到OpenPilot中的自动超车控制器
访问地址: http://op_ip:8088
"""

import os
import sys
import json
import time
import threading
import socket
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

# 添加到OpenPilot路径
sys.path.append('/data/openpilot')
try:
    import cereal.messaging as messaging
    from common.realtime import Ratekeeper
    from common.params import Params
    OP_AVAILABLE = True
    print("✅ OpenPilot环境检测成功")
except ImportError:
    print("❌ 错误：未找到OpenPilot环境")
    sys.exit(1)

class AutoOvertakeController:
    def __init__(self):
        self.vehicle_data = self._init_vehicle_data()
        self.control_state = self._init_control_state()
        self.config = self._init_config()
        # OP赋值+1表示变道成功
        self.control_state.setdefault('overtakeSuccessCount', 0)
        # OP赋值1时表示OP正在控制转向，取消一切超车行为
        self.vehicle_data.setdefault('system_auto_control', 0)
           
        # 消息发布/订阅
        self.pm = messaging.PubMaster(['autoOvertake'])
        self.sm = messaging.SubMaster([
            'carState', 'carControl', 'radarState',
            'modelV2', 'selfdriveState', 'liveLocationKalman'
        ])
        self.params = Params()

        # UDP客户端用于发送指令
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.remote_ip = "127.0.0.1"
        self.remote_port = 4211

        # 指令索引
        self.cmd_index = 0
        self.last_command_time = 0

        # 线程控制
        self.running = True
        self.data_thread = None
        self.web_server = None

        # 加载持久化配置
        self.load_persistent_config()

        print("✅ 控制器初始化完成")

    def _init_vehicle_data(self):
        return {
            'v_cruise_kph': 0, 'v_ego_kph': 0, 'IsOnroad': False,
            'desire_speed': 0, 'active': False, 'lead_speed': 0,
            'lead_distance': 0, 'lead_relative_speed': 0, 'lane_count': 3,
            'l_lane_width': 3.2, 'r_lane_width': 3.2, 'l_edge_dist': 1.5,
            'r_edge_dist': 1.5, 'road_curvature': 0.0, 'steering_angle': 0.0,
            'lat_a': 0.0, 'max_curve': 0.0, 'atc_type': 'none',
            'left_blindspot': False, 'right_blindspot': False,
            'left_lead_speed': 0, 'left_lead_distance': 0, 'left_lead_relative_speed': 0,
            'right_lead_speed': 0, 'right_lead_distance': 0, 'right_lead_relative_speed': 0,
            'blinker': 'none', 'gas_press': False, 'break_press': False,
            'engaged': False, 'l_front_blind': False, 'r_front_blind': False,
            'system_auto_control': 0  # 新增：系统自动控制状态，由OP自动赋值
        }

    def _init_control_state(self):
        return {
            'current_status': '就绪', 'last_command': '', 'blinker_state': 'none',
            'cruise_active': False, 'isOvertaking': False, 'overtakeState': '等待超车条件',
            'overtakeReason': '分析道路情况中...', 'overtakingCompleted': False,
            'overtakeSuccessCount': 0, 'lastOvertakeDirection': '',
            'lastOvertakeTime': 0, 'lastLaneChangeCommandTime': 0,
            'lane_change_in_progress': False,
            'original_lane': None,
            'return_to_original_lane_pending': False,
            'follow_start_time': None,
            'is_following_slow_vehicle': False,
            'max_follow_time_reached': False,
            'last_overtake_result': 'none',
            'dynamic_cooldown': 8000,
            'consecutive_failures': 0
        }

    def _init_config(self):
        return {
            'road_type': 'highway', 'lane_count': 3, 'current_lane_number': 2,
            'autoOvertakeEnabled': False, 'shouldReturnToLane': True, 
            'autoLaneCountEnabled': True, 'HIGHWAY_MIN_SPEED': 75.0, 
            'NORMAL_ROAD_MIN_SPEED': 40.0, 'CRUISE_SPEED_RATIO_THRESHOLD': 0.8, 
            'FOLLOW_TIME_GAP_THRESHOLD': 3.0,  # 时间距离阈值（秒）
            'MAX_FOLLOW_TIME': 120000,  # 最大跟车时间2分钟
            'OVERTAKE_COOLDOWN_BASE': 8000, 
            'OVERTAKE_COOLDOWN_FAILED': 3000, 
            'OVERTAKE_COOLDOWN_SUCCESS': 15000, 
            'OVERTAKE_COOLDOWN_CONDITION': 5000,
            'MIN_LANE_WIDTH': 2.5, 
            'SAFE_LANE_WIDTH': 3.0, 
            'SIDE_LEAD_DISTANCE_MIN': 15.0,
            'SIDE_RELATIVE_SPEED_THRESHOLD': 20, 
            'CURVATURE_THRESHOLD': 0.02,
            'STEERING_THRESHOLD': 20.0, 
            'LEAD_RELATIVE_SPEED_THRESHOLD': -5.0,
            'PENALTY_WEIGHTS': {
                'lead_relative_speed': 2.0, 
                'side_lead_distance': 1.5,
                'side_relative_speed': 1.8, 
                'lane_width': 1.2,
                'blindspot': 3.0, 
                'curvature': 1.5,
                'min_speed_advantage': 5.0
            },
            'PENALTY_THRESHOLD': 50.0, 
            'MIN_SPEED_ADVANTAGE': 5.0,
            'HIGHWAY_STRATEGY': {
                'prefer_left_overtake': True, 
                'avoid_rightmost_lane': True,
                'emergency_lane_penalty': 100, 
                'fast_lane_bonus': 15,
                'min_advantage_threshold': 3
            }
        }

    def load_persistent_config(self):
        """从持久化存储加载配置"""
        try:
            config_json = self.params.get("AutoOvertakeConfig")
            if config_json is not None:
                saved_config = json.loads(config_json)
                print(f"📥 加载保存的配置")
                for key, value in saved_config.items():
                    if key in self.config:
                        self.config[key] = value
            else:
                print("📥 使用默认配置")
        except Exception as e:
            print(f"⚠️ 加载配置失败: {e}")

    def save_persistent_config(self):
        """保存配置到持久化存储"""
        try:
            self.params.put("AutoOvertakeConfig", json.dumps(self.config))
            print("✅ 配置已保存")
        except Exception as e:
            print(f"⚠️ 保存配置失败: {e}")

    def calculate_time_gap(self):
        """计算跟车时间距离（秒）"""
        vd = self.vehicle_data
        
        if vd['lead_distance'] <= 0 or vd['v_ego_kph'] <= 0:
            return 0
        
        # 将本车速度从km/h转换为m/s
        v_ego_ms = vd['v_ego_kph'] / 3.6
        
        # 计算时间距离（秒）= 距离（米）/ 速度（米/秒）
        time_gap = vd['lead_distance'] / v_ego_ms if v_ego_ms > 0 else 0
        
        return time_gap

    def update_vehicle_data(self):
        """更新车辆数据 - 使用真实OpenPilot数据"""
        try:
            # 基础在线状态
            isOnroad = self.params.get_bool("IsOnroad")
            self.vehicle_data['IsOnroad'] = isOnroad

            if isOnroad:
              self.sm.update(100)  # 100ms超时
            else:
              self.sm.update(0)  # 不阻塞，不触发等待

            if isOnroad:
                # 车辆状态数据
                if self.sm.alive['carState']:
                    carState = self.sm['carState']

                    # 速度相关
                    v_ego_kph = int(carState.vEgo * 3.6 + 0.5) if carState.vEgo else 0
                    v_cruise_kph = carState.vCruise

                    self.vehicle_data.update({
                        'v_ego_kph': v_ego_kph,
                        'v_cruise_kph': v_cruise_kph,
                        'cruise_speed': v_cruise_kph,
                        'steering_angle': round(carState.steeringAngleDeg, 1) if carState.steeringAngleDeg else 0.0,
                        'blinker': self._get_blinker_state(carState.leftBlinker, carState.rightBlinker),
                        'gas_press': carState.gasPressed,
                        'break_press': carState.brakePressed,
                        'engaged': carState.cruiseState.enabled,
                        'left_blindspot': bool(carState.leftBlindspot),
                        'right_blindspot': bool(carState.rightBlindspot)
                    })

                    # 加速度
                    if carState.aEgo:
                        self.vehicle_data['lat_a'] = round(carState.aEgo, 1)

                # 雷达数据 - 前车
                if self.sm.alive['radarState']:
                    radarState = self.sm['radarState']

                    # 主前车
                    if radarState.leadOne.status:
                        leadOne = radarState.leadOne
                        self.vehicle_data.update({
                            'lead_distance': int(leadOne.dRel),
                            'lead_speed': int(leadOne.vLead * 3.6),
                            'lead_relative_speed': int(leadOne.vRel * 3.6)
                        })
                    else:
                        # 前车消失时重置数据
                        self.vehicle_data.update({
                            'lead_distance': 0,
                            'lead_speed': 0,
                            'lead_relative_speed': 0
                        })

                    # 左侧前车
                    if radarState.leadLeft.status:
                        leadLeft = radarState.leadLeft
                        self.vehicle_data.update({
                            'left_lead_distance': int(leadLeft.dRel),
                            'left_lead_speed': int(leadLeft.vLead * 3.6),
                            'left_lead_relative_speed': int(leadLeft.vRel * 3.6)
                        })
                    else:
                        # 左侧前车消失时重置数据
                        self.vehicle_data.update({
                            'left_lead_distance': 0,
                            'left_lead_speed': 0,
                            'left_lead_relative_speed': 0
                        })

                    # 右侧前车
                    if radarState.leadRight.status:
                        leadRight = radarState.leadRight
                        self.vehicle_data.update({
                            'right_lead_distance': int(leadRight.dRel),
                            'right_lead_speed': int(leadRight.vLead * 3.6),
                            'right_lead_relative_speed': int(leadRight.vRel * 3.6)
                        })
                    else:
                        # 右侧前车消失时重置数据
                        self.vehicle_data.update({
                            'right_lead_distance': 0,
                            'right_lead_speed': 0,
                            'right_lead_relative_speed': 0
                        })

                # 期望速度
                self.vehicle_data['desire_speed'] = 90

            # 模型数据 - 车道信息和盲区
            if self.sm.alive['modelV2']:
                modelV2 = self.sm['modelV2']
                meta = modelV2.meta

                self.vehicle_data.update({
                    'blinker': meta.blinker,
                    'l_front_blind': meta.leftFrontBlind,
                    'r_front_blind': meta.rightFrontBlind,
                    'l_lane_width': round(meta.laneWidthLeft, 1),
                    'r_lane_width': round(meta.laneWidthRight, 1),
                    'l_edge_dist': round(meta.distanceToRoadEdgeLeft, 1),
                    'r_edge_dist': round(meta.distanceToRoadEdgeRight, 1)
                })

            # 自驾状态
            if self.sm.alive['selfdriveState']:
                selfdriveState = self.sm['selfdriveState']
                self.vehicle_data['active'] = "on" if selfdriveState.active else "off"

        except Exception as e:
            print(f"更新车辆数据错误: {e}")

    def _get_blinker_state(self, left_blinker, right_blinker):
        if left_blinker and right_blinker:
            return "hazard"
        elif left_blinker:
            return "left"
        elif right_blinker:
            return "right"
        else:
            return "none"

    def update_following_status(self):
        """更新跟车状态 - 考虑三个触发条件"""
        vd = self.vehicle_data
        cs = self.control_state
        cfg = self.config
        now = time.time() * 1000
        
        # 计算时间距离和速度比例
        time_gap = self.calculate_time_gap()
        speed_ratio = vd['v_ego_kph'] / vd['v_cruise_kph'] if vd['v_cruise_kph'] > 0 else 1.0
        
        # 检查是否在跟车状态（满足任意一个触发条件）
        is_following = (
            vd['lead_distance'] > 0 and  # 前方有车
            (
                # 三个触发条件：或关系
                vd['lead_relative_speed'] < cfg['LEAD_RELATIVE_SPEED_THRESHOLD'] or
                (0 < time_gap <= cfg['FOLLOW_TIME_GAP_THRESHOLD']) or
                speed_ratio < cfg['CRUISE_SPEED_RATIO_THRESHOLD']
            )
        )
        
        if is_following:
            if cs['follow_start_time'] is None:
                # 开始跟车，记录时间
                cs['follow_start_time'] = now
                cs['is_following_slow_vehicle'] = True
                
                # 记录触发原因
                trigger_reasons = []
                if vd['lead_relative_speed'] < cfg['LEAD_RELATIVE_SPEED_THRESHOLD']:
                    trigger_reasons.append(f"相对速度{vd['lead_relative_speed']}km/h")
                if 0 < time_gap <= cfg['FOLLOW_TIME_GAP_THRESHOLD']:
                    trigger_reasons.append(f"时间距离{time_gap:.1f}秒")
                if speed_ratio < cfg['CRUISE_SPEED_RATIO_THRESHOLD']:
                    trigger_reasons.append(f"速度比例{speed_ratio*100:.0f}%")
                
                print(f"🚗 开始跟车计时 | 触发原因: {' | '.join(trigger_reasons)}")
            
            # 检查是否达到最大跟车时间
            follow_duration = now - cs['follow_start_time']
            if follow_duration >= cfg['MAX_FOLLOW_TIME'] and not cs['max_follow_time_reached']:
                cs['max_follow_time_reached'] = True
                minutes = cfg['MAX_FOLLOW_TIME'] // 60000
                cs['overtakeReason'] = f"跟车时间超过{minutes}分钟，强制超车"
                print(f"⏰ 达到最大跟车时间: {follow_duration/60000:.1f}分钟，触发强制超车")
        else:
            # 不在跟车状态，重置计时器
            if cs['follow_start_time'] is not None:
                print(f"🔄 重置跟车计时器 | 前车状态变化")
            cs['follow_start_time'] = None
            cs['is_following_slow_vehicle'] = False
            cs['max_follow_time_reached'] = False

    def calculate_dynamic_cooldown(self):
        """计算动态冷却时间"""
        cs = self.control_state
        cfg = self.config
        
        base_cooldown = cfg['OVERTAKE_COOLDOWN_BASE']
        
        # 根据上次超车结果调整冷却时间
        if cs['last_overtake_result'] == 'success':
            # 成功超车后冷却时间较长，避免频繁超车
            cooldown = cfg['OVERTAKE_COOLDOWN_SUCCESS']
            cs['consecutive_failures'] = 0  # 重置连续失败计数
        elif cs['last_overtake_result'] == 'failed':
            # 超车失败后较短冷却，尽快重试
            cooldown = cfg['OVERTAKE_COOLDOWN_FAILED']
            cs['consecutive_failures'] += 1
        elif cs['last_overtake_result'] == 'condition':
            # 条件不满足，中等冷却时间
            cooldown = cfg['OVERTAKE_COOLDOWN_CONDITION']
            cs['consecutive_failures'] += 1
        else:
            # 首次或无结果，使用基础冷却
            cooldown = base_cooldown
        
        # 连续失败惩罚机制
        if cs['consecutive_failures'] > 3:
            # 连续失败3次以上，增加冷却时间避免频繁尝试
            penalty = min(10000, cs['consecutive_failures'] * 2000)  # 最大10秒惩罚
            cooldown += penalty
            print(f"⚠️ 连续失败{cs['consecutive_failures']}次，增加冷却时间{penalty/1000}秒")
        
        # 根据道路类型调整
        if self.config['road_type'] == 'highway':
            # 高速公路冷却时间稍短
            cooldown = max(5000, cooldown * 0.8)
        else:
            # 普通道路冷却时间稍长
            cooldown = cooldown * 1.2
        
        cs['dynamic_cooldown'] = cooldown
        return cooldown

    def get_trigger_conditions(self):
        """获取当前触发超车的条件状态 - 三个独立条件：或关系"""
        vd = self.vehicle_data
        cfg = self.config
        cs = self.control_state
        
        conditions = []
        
        # 最大跟车时间触发（独立条件）
        if cs['max_follow_time_reached']:
            conditions.append("⏰ 最大跟车时间触发")
            return conditions  # 最大跟车时间触发时，直接返回
        
        # 三个独立触发条件：或关系，满足任意一个就触发
        speed_ratio = vd['v_ego_kph'] / vd['v_cruise_kph'] if vd['v_cruise_kph'] > 0 else 1.0
        
        # 1. 前车相对速度触发
        if vd['lead_relative_speed'] < cfg['LEAD_RELATIVE_SPEED_THRESHOLD']:
            conditions.append(f"🚗 前车较慢: {vd['lead_relative_speed']}km/h")
            return conditions
        
        # 2. 跟车时间距离触发
        time_gap = self.calculate_time_gap()
        if 0 < time_gap <= cfg['FOLLOW_TIME_GAP_THRESHOLD']:
            conditions.append(f"⏱️ 跟车时间: {time_gap:.1f}秒")
            return conditions
        
        # 3. 低于巡航速度百分比触发
        if speed_ratio < cfg['CRUISE_SPEED_RATIO_THRESHOLD']:
            conditions.append(f"🚀 速度比例: {speed_ratio*100:.0f}%")
            return conditions
        
        # 如果没有触发条件，返回空列表
        return conditions

    def check_overtake_conditions(self):
        """检查超车条件 - 三个独立条件：或关系"""
        vd = self.vehicle_data
        cs = self.control_state
        cfg = self.config
        now = time.time() * 1000

        # 检查系统自动控制状态
        if vd['system_auto_control'] == 1:
            cs['overtakeReason'] = "OP自动控制中，暂停超车"
            cs['last_overtake_result'] = 'condition'
            return False

        if not vd['IsOnroad']:
            cs['overtakeReason'] = "车辆不在道路上"
            cs['last_overtake_result'] = 'condition'
            return False

        if not vd['engaged']:
            cs['overtakeReason'] = "巡航未激活"
            cs['last_overtake_result'] = 'condition'
            return False

        # 最大跟车时间强制超车（最高优先级）
        if cs['max_follow_time_reached']:
            cs['overtakeReason'] = f"跟车时间超过{cfg['MAX_FOLLOW_TIME']//60000}分钟，强制超车"
            return True

        # 常规超车条件检查 - 前方必须有车辆
        if vd['lead_distance'] <= 0:
            cs['overtakeReason'] = "前方无车辆"
            cs['last_overtake_result'] = 'condition'
            return False

        # 计算速度比例
        speed_ratio = vd['v_ego_kph'] / vd['v_cruise_kph'] if vd['v_cruise_kph'] > 0 else 1.0
        
        # 三个独立触发条件：或关系
        has_trigger = False
        trigger_reason = ""
        
        # 1. 前车相对速度触发
        if vd['lead_relative_speed'] < cfg['LEAD_RELATIVE_SPEED_THRESHOLD']:
            has_trigger = True
            trigger_reason = f"前车相对速度{vd['lead_relative_speed']}km/h"
        
        # 2. 跟车时间距离触发
        time_gap = self.calculate_time_gap()
        if not has_trigger and 0 < time_gap <= cfg['FOLLOW_TIME_GAP_THRESHOLD']:
            has_trigger = True
            trigger_reason = f"跟车时间距离{time_gap:.1f}秒"
        
        # 3. 低于巡航速度百分比触发
        if not has_trigger and speed_ratio < cfg['CRUISE_SPEED_RATIO_THRESHOLD']:
            has_trigger = True
            trigger_reason = f"速度比例{speed_ratio*100:.0f}%"
        
        # 如果没有触发条件，返回
        if not has_trigger:
            cs['overtakeReason'] = "未满足任何超车触发条件"
            cs['last_overtake_result'] = 'condition'
            return False

        # 最低速度检查
        if cfg['road_type'] == 'highway' and vd['v_ego_kph'] < cfg['HIGHWAY_MIN_SPEED']:
            cs['overtakeReason'] = f"高速公路车速{vd['v_ego_kph']}km/h低于最低超车速度"
            cs['last_overtake_result'] = 'condition'
            return False

        if cfg['road_type'] == 'normal' and vd['v_ego_kph'] < cfg['NORMAL_ROAD_MIN_SPEED']:
            cs['overtakeReason'] = f"普通公路车速{vd['v_ego_kph']}km/h低于最低超车速度"
            cs['last_overtake_result'] = 'condition'
            return False

        # 智能冷却时间检查
        current_cooldown = self.calculate_dynamic_cooldown()
        if now - cs['lastOvertakeTime'] < current_cooldown:
            remaining = (current_cooldown - (now - cs['lastOvertakeTime'])) / 1000
            reason_suffix = ""
            if cs['last_overtake_result'] == 'success':
                reason_suffix = "（成功超车后冷却）"
            elif cs['last_overtake_result'] == 'failed':
                reason_suffix = "（超车失败后快速重试）"
            elif cs['last_overtake_result'] == 'condition':
                reason_suffix = "（条件不满足冷却）"
            
            cs['overtakeReason'] = f"超车冷却中，请等待{remaining:.1f}秒{reason_suffix}"
            return False

        # 所有条件满足，记录触发原因
        cs['overtakeReason'] = f"触发超车: {trigger_reason}"
        return True

    def evaluate_overtake_effectiveness(self, direction):
        """评估超车有效性 - 避免无效超车"""
        vd = self.vehicle_data
        cfg = self.config
        
        if direction == "LEFT":
            side_lead_speed = vd['left_lead_speed']
            side_lead_distance = vd['left_lead_distance']
            side_relative_speed = vd['left_lead_relative_speed']
        else:  # RIGHT
            side_lead_speed = vd['right_lead_speed']
            side_lead_distance = vd['right_lead_distance']
            side_relative_speed = vd['right_lead_relative_speed']
        
        current_speed = vd['v_ego_kph']
        current_lead_speed = vd['lead_speed']
        
        # 有效性分析
        effectiveness = 100  # 初始有效性评分
        reasons = []
        
        # 1. 检查侧车道前车速度是否明显更慢
        if side_lead_speed > 0 and side_lead_speed < current_lead_speed - 5:
            effectiveness -= 30
            reasons.append(f"侧前车速度{side_lead_speed}km/h比当前前车{current_lead_speed}km/h更慢")
        
        # 2. 检查侧车道前车速度是否比本车慢很多
        if side_lead_speed > 0 and side_lead_speed < current_speed - 10:
            effectiveness -= 40
            reasons.append(f"侧前车速度{side_lead_speed}km/h比本车{current_speed}km/h慢太多")
        
        # 3. 检查侧车道前车距离是否过近
        if side_lead_distance > 0 and side_lead_distance < 20:
            effectiveness -= 20
            reasons.append(f"侧前车距离{side_lead_distance}m过近")
        
        # 4. 检查侧车道前车相对速度（如果是负值表示比本车慢）
        if side_relative_speed < -15:
            effectiveness -= 25
            reasons.append(f"侧前车相对速度{side_relative_speed}km/h，明显更慢")
        
        # 5. 如果是向右变道，考虑右侧车道通常较慢
        if direction == "RIGHT" and cfg['road_type'] == 'highway':
            effectiveness -= 10
            reasons.append("右侧车道通常较慢")
        
        # 确保有效性在合理范围内
        effectiveness = max(0, effectiveness)
        
        return effectiveness, reasons

    def is_overtake_effective(self, direction):
        """判断超车是否有效"""
        effectiveness, reasons = self.evaluate_overtake_effectiveness(direction)
        
        # 有效性阈值
        min_effectiveness = 60  # 最低有效性要求
        
        is_effective = effectiveness >= min_effectiveness
        return is_effective, effectiveness, reasons

    def check_lane_safety(self, side):
        """检查车道安全性"""
        vd = self.vehicle_data
        cfg = self.config

        if side == "left":
            # 检查车道宽度
            if vd['l_lane_width'] < cfg['MIN_LANE_WIDTH']:
                return False, "车道过窄⚠️禁止变道"

            if vd['left_blindspot'] or vd['l_front_blind']:
                return False, "盲区有车"
            if vd['left_lead_distance'] > 0 and vd['left_lead_distance'] < cfg['SIDE_LEAD_DISTANCE_MIN']:
                return False, "侧前车距离过近"
            if abs(vd['left_lead_relative_speed']) > cfg['SIDE_RELATIVE_SPEED_THRESHOLD']:
                return False, "侧车相对速度过高"
            return True, "安全"

        elif side == "right":
            # 检查车道宽度
            if vd['r_lane_width'] < cfg['MIN_LANE_WIDTH']:
                return False, "车道过窄⚠️禁止变道"

            if vd['right_blindspot'] or vd['r_front_blind']:
                return False, "盲区有车"
            if vd['right_lead_distance'] > 0 and vd['right_lead_distance'] < cfg['SIDE_LEAD_DISTANCE_MIN']:
                return False, "侧前车距离过近"
            if abs(vd['right_lead_relative_speed']) > cfg['SIDE_RELATIVE_SPEED_THRESHOLD']:
                return False, "侧车相对速度过高"
            return True, "安全"

        return False, "未知方向"

    def evaluate_lane_suitability(self, side):
        """评估车道适合度"""
        vd = self.vehicle_data
        cfg = self.config
        current_lane = cfg['current_lane_number']
        total_lanes = cfg['lane_count']
        
        # 检查目标车道编号
        if side == "left":
            target_lane = current_lane - 1
        else:  # right
            target_lane = current_lane + 1
        
        # 应急车道检查
        if self.is_emergency_lane(target_lane):
            return 0, ["🚫 应急车道，禁止行驶"]
        
        penalty_score = 0
        analysis = []
        weights = cfg['PENALTY_WEIGHTS']
        
        if side == "left":
            # 盲区检查（最高优先级）
            if vd['left_blindspot'] or vd['l_front_blind']:
                penalty_score += 100
                analysis.append("❌ 盲区有车")
                return penalty_score, analysis
            
            # 车道宽度检查
            lane_width = vd['l_lane_width']
            if lane_width < cfg['MIN_LANE_WIDTH']:
                penalty_score += 80
                analysis.append(f"❌ 车道过窄: {lane_width}m")
            elif lane_width < cfg['SAFE_LANE_WIDTH']:
                penalty_score += (cfg['SAFE_LANE_WIDTH'] - lane_width) * weights['lane_width'] * 10
                analysis.append(f"⚠️ 车道略窄: {lane_width}m")
            else:
                analysis.append(f"✅ 车道宽度正常: {lane_width}m")
            
            # 高速公路最左车道特殊处理
            if cfg['road_type'] == 'highway' and target_lane == 1:
                analysis.append("🚀 快车道 - 超车优先")
                # 快车道有额外优势
                penalty_score -= 15
            
            # 侧前车距离检查
            side_distance = vd['left_lead_distance']
            if side_distance > 0:
                if side_distance < cfg['SIDE_LEAD_DISTANCE_MIN']:
                    penalty_score += (cfg['SIDE_LEAD_DISTANCE_MIN'] - side_distance) * weights['side_lead_distance']
                    analysis.append(f"⚠️ 侧前车过近: {side_distance}m")
                else:
                    # 距离越远，惩罚越小（负惩罚）
                    distance_advantage = side_distance - cfg['SIDE_LEAD_DISTANCE_MIN']
                    penalty_score -= min(distance_advantage * 0.5, 20)  # 最大奖励20分
                    analysis.append(f"✅ 侧前车安全距离: {side_distance}m")
            
            # 侧前车相对速度检查
            side_relative_speed = vd['left_lead_relative_speed']
            if side_relative_speed != 0:
                if side_relative_speed < -weights['min_speed_advantage']:  # 侧前车明显更慢
                    penalty_score += abs(side_relative_speed) * weights['side_relative_speed']
                    analysis.append(f"❌ 侧前车较慢: {side_relative_speed}km/h")
                elif side_relative_speed > weights['min_speed_advantage']:  # 侧前车明显更快
                    # 速度优势，减少惩罚
                    speed_advantage = min(side_relative_speed * 0.8, 25)  # 最大奖励25分
                    penalty_score -= speed_advantage
                    analysis.append(f"✅ 侧前车较快: +{side_relative_speed}km/h")
                else:
                    analysis.append(f"➖ 侧前车速度相当: {side_relative_speed}km/h")
        
        elif side == "right":
            # 盲区检查
            if vd['right_blindspot'] or vd['r_front_blind']:
                penalty_score += 100
                analysis.append("❌ 盲区有车")
                return penalty_score, analysis
            
            # 车道宽度检查
            lane_width = vd['r_lane_width']
            if lane_width < cfg['MIN_LANE_WIDTH']:
                penalty_score += 80
                analysis.append(f"❌ 车道过窄: {lane_width}m")
            elif lane_width < cfg['SAFE_LANE_WIDTH']:
                penalty_score += (cfg['SAFE_LANE_WIDTH'] - lane_width) * weights['lane_width'] * 10
                analysis.append(f"⚠️ 车道略窄: {lane_width}m")
            else:
                analysis.append(f"✅ 车道宽度正常: {lane_width}m")
            
            # 高速公路应急车道检查
            if self.is_emergency_lane(target_lane):
                return 0, ["🚫 应急车道，禁止行驶"]
            
            # 高速公路最右车道（非应急车道）通常较慢
            if cfg['road_type'] == 'highway' and target_lane == total_lanes - 1:
                analysis.append("⚠️ 右侧车道通常较慢")
                penalty_score += 10
            
            # 侧前车距离检查
            side_distance = vd['right_lead_distance']
            if side_distance > 0:
                if side_distance < cfg['SIDE_LEAD_DISTANCE_MIN']:
                    penalty_score += (cfg['SIDE_LEAD_DISTANCE_MIN'] - side_distance) * weights['side_lead_distance']
                    analysis.append(f"⚠️ 侧前车过近: {side_distance}m")
                else:
                    distance_advantage = side_distance - cfg['SIDE_LEAD_DISTANCE_MIN']
                    penalty_score -= min(distance_advantage * 0.5, 20)
                    analysis.append(f"✅ 侧前车安全距离: {side_distance}m")
            
            # 侧前车相对速度检查
            side_relative_speed = vd['right_lead_relative_speed']
            if side_relative_speed != 0:
                if side_relative_speed < -weights['min_speed_advantage']:
                    penalty_score += abs(side_relative_speed) * weights['side_relative_speed']
                    analysis.append(f"❌ 侧前车较慢: {side_relative_speed}km/h")
                elif side_relative_speed > weights['min_speed_advantage']:
                    speed_advantage = min(side_relative_speed * 0.8, 25)
                    penalty_score -= speed_advantage
                    analysis.append(f"✅ 侧前车较快: +{side_relative_speed}km/h")
                else:
                    analysis.append(f"➖ 侧前车速度相当: {side_relative_speed}km/h")
        
        # 确保惩罚分数不为负
        penalty_score = max(0, penalty_score)
        
        # 计算适合度分数（0-100，越高越适合）
        suitability_score = max(0, 100 - penalty_score)
        
        analysis.insert(0, f"适合度评分: {suitability_score:.1f}/100")
        
        return suitability_score, analysis

    def get_current_lane_penalty(self):
        """计算当前车道的惩罚分数（用于比较）"""
        vd = self.vehicle_data
        cfg = self.config
        
        penalty = 0
        analysis = []
        
        # 当前车道前车相对速度惩罚
        if vd['lead_relative_speed'] < -cfg['MIN_SPEED_ADVANTAGE']:
            speed_penalty = abs(vd['lead_relative_speed']) * cfg['PENALTY_WEIGHTS']['lead_relative_speed']
            penalty += speed_penalty
            analysis.append(f"当前前车较慢: {vd['lead_relative_speed']}km/h → +{speed_penalty:.1f}惩罚")
        
        # 当前车道跟车距离惩罚
        time_gap = self.calculate_time_gap()
        if time_gap > 0 and time_gap < cfg['FOLLOW_TIME_GAP_THRESHOLD']:
            distance_penalty = (cfg['FOLLOW_TIME_GAP_THRESHOLD'] - time_gap) * 10
            penalty += distance_penalty
            analysis.append(f"跟车时间较近: {time_gap:.1f}秒 → +{distance_penalty:.1f}惩罚")
        
        return penalty, analysis

    def get_available_overtake_directions(self):
        """获取可用的超车方向，考虑车道边界和应急车道"""
        vd = self.vehicle_data
        cfg = self.config
        current_lane = cfg['current_lane_number']
        total_lanes = cfg['lane_count']
        
        available_directions = []
        
        # 检查左侧车道可用性
        if current_lane > 1:  # 不是最左车道
            available_directions.append("LEFT")
        elif current_lane == 1 and cfg['road_type'] != 'highway':
            # 普通道路的最左车道也可以向左变道（如果有路肩）
            available_directions.append("LEFT")
        
        # 检查右侧车道可用性
        if current_lane < total_lanes:  # 不是最右车道
            available_directions.append("RIGHT")
        elif current_lane == total_lanes and cfg['road_type'] != 'highway':
            # 普通道路的最右车道也可以向右变道（如果有路肩）
            available_directions.append("RIGHT")
        
        # 高速公路特殊处理：考虑应急车道
        if cfg['road_type'] == 'highway':
            # 高速公路最右车道通常是应急车道，应该向左变道
            if current_lane == total_lanes:
                # 在最右车道时，优先考虑向左变道
                if "LEFT" in available_directions:
                    available_directions.remove("RIGHT")  # 移除向右变道选项
                    available_directions.insert(0, "LEFT")  # 优先向左
            # 高速公路最左车道是快车道，向右变道应该谨慎
            elif current_lane == 1:
                # 在最左快车道时，可以向右变道但优先级较低
                if "RIGHT" in available_directions:
                    available_directions.remove("RIGHT")
                    available_directions.append("RIGHT")  # 放在最后
        
        return available_directions

    def is_emergency_lane(self, lane_number):
        """判断是否为应急车道"""
        cfg = self.config
        # 高速公路的最右车道通常是应急车道
        if cfg['road_type'] == 'highway' and lane_number == cfg['lane_count']:
            return True
        return False

    def perform_auto_overtake(self):
        """执行自动超车 - 包含有效性评估"""
        if not self.config['autoOvertakeEnabled'] or self.control_state['isOvertaking']:
            return

        # 检查系统自动控制状态
        if self.vehicle_data['system_auto_control'] == 1:
            self.control_state['overtakeState'] = "OP控制中"
            self.control_state['overtakeReason'] = "OP自动控制中，暂停超车"
            return

        if not self.check_overtake_conditions():
            return

        # 获取可用的超车方向
        available_directions = self.get_available_overtake_directions()
        
        if not available_directions:
            self.control_state['overtakeState'] = "无可用变道方向"
            self.control_state['overtakeReason'] = "当前车道位置限制"
            return

        # 计算当前车道惩罚
        current_penalty, current_analysis = self.get_current_lane_penalty()
        
        # 评估可用方向的车道
        direction_scores = {}
        direction_analysis = {}
        direction_effectiveness = {}
        
        for direction in available_directions:
            side = "left" if direction == "LEFT" else "right"
            
            # 安全性评估
            safety_score, safety_analysis = self.evaluate_lane_suitability(side)
            
            # 有效性评估
            is_effective, effectiveness_score, effectiveness_reasons = self.is_overtake_effective(direction)
            
            # 综合评分 = 安全性评分 × 有效性系数
            effectiveness_factor = effectiveness_score / 100.0
            combined_score = safety_score * effectiveness_factor
            
            direction_scores[direction] = combined_score
            direction_effectiveness[direction] = {
                'score': effectiveness_score,
                'is_effective': is_effective,
                'reasons': effectiveness_reasons
            }
            
            # 合并分析
            full_analysis = safety_analysis.copy()
            if effectiveness_reasons:
                full_analysis.extend([f"⚠️ {reason}" for reason in effectiveness_reasons])
            if is_effective:
                full_analysis.append(f"✅ 超车有效性: {effectiveness_score}%")
            else:
                full_analysis.append(f"❌ 超车无效: {effectiveness_score}%")
                
            direction_analysis[direction] = full_analysis

        # 智能决策
        best_direction = None
        best_score = 0
        detailed_reason = ""
        
        for direction in available_directions:
            score = direction_scores[direction]
            effectiveness_info = direction_effectiveness[direction]
            
            # 必须满足有效性要求
            if not effectiveness_info['is_effective']:
                continue
                
            # 高速公路特殊策略
            if self.config['road_type'] == 'highway':
                current_lane = self.config['current_lane_number']
                
                # 在最右车道时优先向左变道
                if current_lane == self.config['lane_count'] and direction == "LEFT":
                    score += 20  # 向左变道额外加分
                    direction_analysis[direction].append("🔄 最右车道优先向左")
                
                # 在快车道（最左）时，向右变道要谨慎
                elif current_lane == 1 and direction == "RIGHT":
                    score -= 15  # 向右变道惩罚
                    direction_analysis[direction].append("⚠️ 快车道向右需谨慎")
            
            if score > self.config['PENALTY_THRESHOLD'] and score > best_score:
                best_direction = direction
                best_score = score
                
                # 构建详细原因
                effectiveness_text = f"有效性{effectiveness_info['score']}%"
                safety_text = f"安全性{score:.1f}%"
                analysis_text = " | ".join(direction_analysis[direction])
                detailed_reason = f"{direction}车道 {effectiveness_text} | {safety_text} | {analysis_text}"

        # 执行超车决策
        if best_direction and best_score > self.config['PENALTY_THRESHOLD']:
            target_advantage = best_score - (100 - current_penalty)
            
            # 调整优势阈值，考虑车道边界情况
            min_advantage = 5  # 基本优势阈值
            if self.config['road_type'] == 'highway':
                # 高速公路可以更灵活
                min_advantage = 3
            
            if target_advantage >= min_advantage:
                self.execute_overtake(best_direction)
                self.control_state['overtakeReason'] = detailed_reason
                print(f"🎯 智能车道选择: {best_direction}变道 | 综合评分: {best_score:.1f}%")
            else:
                self.control_state['overtakeState'] = "目标车道优势不足"
                self.control_state['overtakeReason'] = f"目标车道优势不足: +{target_advantage:.1f}% | 需要至少+{min_advantage}%"
        else:
            # 提供详细的不可超车原因
            no_overtake_reasons = []
            
            for direction in available_directions:
                score = direction_scores[direction]
                effectiveness_info = direction_effectiveness[direction]
                
                if not effectiveness_info['is_effective']:
                    reason = f"{direction}:无效超车({effectiveness_info['score']}%)"
                    if effectiveness_info['reasons']:
                        reason += f"[{','.join(effectiveness_info['reasons'])}]"
                elif score <= self.config['PENALTY_THRESHOLD']:
                    reason = f"{direction}:安全性不足({score:.1f}%)"
                else:
                    reason = f"{direction}:条件满足({score:.1f}%)"
                
                no_overtake_reasons.append(reason)
            
            if not available_directions:
                no_overtake_reasons.append("无可用变道方向")
            
            self.control_state['overtakeState'] = "分析车道中"
            self.control_state['overtakeReason'] = f"超车条件分析: {', '.join(no_overtake_reasons)}"

    def execute_overtake(self, direction):
        """执行超车操作"""
        # 记录当前的成功计数，用于检测变道完成
        current_success_count = self.control_state['overtakeSuccessCount']
        
        success = self.send_command("LANECHANGE", direction)
        if success:
            self.control_state['isOvertaking'] = True
            self.control_state['lane_change_in_progress'] = True
            self.control_state['lastOvertakeDirection'] = direction
            self.control_state['lastLaneChangeCommandTime'] = time.time() * 1000

            # 重置跟车计时器
            self.control_state['follow_start_time'] = None
            self.control_state['is_following_slow_vehicle'] = False
            self.control_state['max_follow_time_reached'] = False

            # 记录超车开始时的原车道和成功计数
            self.control_state['original_lane'] = self.config['current_lane_number']
            self.control_state['return_to_original_lane_pending'] = False
            self.control_state['overtake_start_count'] = current_success_count

            if direction == "LEFT":
                self.control_state['overtakeState'] = "← 准备向左变道超车"
                self.control_state['current_status'] = "自动左变道"
            else:
                self.control_state['overtakeState'] = "→ 准备向右变道超车"
                self.control_state['current_status'] = "自动右变道"
                
            print(f"🚀 开始超车: {direction}变道 | 动态冷却: {self.control_state['dynamic_cooldown']/1000}秒")

    def check_overtake_completion(self):
        """检查超车完成状态 - 更新冷却时间逻辑"""
        if not self.control_state['lane_change_in_progress']:
            return

        current_count = self.control_state['overtakeSuccessCount']
        start_count = self.control_state.get('overtake_start_count', current_count)
        
        # 如果成功计数增加了，说明变道完成
        if current_count > start_count:
            self.control_state['isOvertaking'] = False
            self.control_state['lane_change_in_progress'] = False
            self.control_state['overtakingCompleted'] = True
            
            # 超车成功，设置冷却时间
            self.control_state['lastOvertakeTime'] = time.time() * 1000
            self.control_state['last_overtake_result'] = 'success'
            
            direction = self.control_state['lastOvertakeDirection']
            direction_text = "左" if direction == "LEFT" else "右"
            
            # 更新状态显示
            if self.config['shouldReturnToLane'] and self.control_state['original_lane'] is not None:
                self.control_state['overtakeState'] = f"{direction_text}超车成功，等待返回原车道"
                self.control_state['overtakeReason'] = f"检测到变道完成，将返回车道"
            else:
                self.control_state['overtakeState'] = f"{direction_text}超车成功"
                self.control_state['overtakeReason'] = f"检测到变道完成"
            
            self.control_state['current_status'] = "超车完成"
            print(f"✅ 变道完成检测: {direction_text}变道成功 | 进入成功冷却")
            
            # 清除开始计数记录
            if 'overtake_start_count' in self.control_state:
                del self.control_state['overtake_start_count']
        
        # 添加超时检查，防止变道指令发出后长时间没有完成
        elif time.time() * 1000 - self.control_state['lastLaneChangeCommandTime'] > 15000:  # 15秒超时
            self.control_state['lane_change_in_progress'] = False
            self.control_state['isOvertaking'] = False
            
            # 超车失败，设置较短的冷却时间
            self.control_state['lastOvertakeTime'] = time.time() * 1000
            self.control_state['last_overtake_result'] = 'failed'
            
            self.control_state['overtakeState'] = "变道超时"
            self.control_state['overtakeReason'] = "15秒内未检测到变道完成，快速重试"
            print("❌ 变道超时，未检测到完成信号，进入失败冷却")

    def check_return_to_original_lane(self):
        """检查是否需要返回原车道 - 移除固定延迟，使用智能判断"""
        cs = self.control_state
        cfg = self.config
        
        # 如果返回原车道功能关闭，或者没有原车道记录，或者正在超车中，则跳过
        if not cfg['shouldReturnToLane'] or cs['original_lane'] is None or cs['isOvertaking']:
            return
        
        current_lane = cfg['current_lane_number']
        original_lane = cs['original_lane']
        
        # 如果已经在原车道，清除状态
        if current_lane == original_lane:
            cs['original_lane'] = None
            cs['return_to_original_lane_pending'] = False
            # 清除返回开始计数
            if 'return_start_count' in cs:
                del cs['return_start_count']
            return
        
        # 检查返回变道是否完成
        if cs.get('return_start_count') is not None:
            current_count = cs['overtakeSuccessCount']
            start_count = cs['return_start_count']
            
            # 如果成功计数增加了，说明返回变道完成
            if current_count > start_count:
                cs['return_to_original_lane_pending'] = False
                # 清除返回开始计数
                del cs['return_start_count']
                print(f"✅ 返回原车道完成: 计数 {start_count} → {current_count}")
                
                # 再次检查是否已经回到原车道
                if current_lane == original_lane:
                    cs['original_lane'] = None
                    cs['overtakeState'] = "已返回原车道"
                    cs['overtakeReason'] = "返回原车道完成"
                else:
                    # 计数增加但还没回到原车道，可能需要再次返回
                    cs['return_to_original_lane_pending'] = True
                    cs['return_start_time'] = time.time()
        
        # 如果超车完成且不在原车道，且没有等待返回，则设置等待返回状态
        if (cs['overtakingCompleted'] and 
            not cs['return_to_original_lane_pending'] and 
            current_lane != original_lane and
            cs.get('return_start_count') is None):  # 确保没有正在进行返回
            
            cs['return_to_original_lane_pending'] = True
            cs['return_start_time'] = time.time()
            print(f"🔄 超车完成，准备返回原车道 {original_lane} (当前: {current_lane})")
        
        # 执行返回原车道
        if cs['return_to_original_lane_pending'] and cs.get('return_start_count') is None:
            self.perform_return_to_original_lane()

    def perform_return_to_original_lane(self):
        """执行返回原车道操作 - 移除固定延迟"""
        cs = self.control_state
        cfg = self.config
        current_lane = cfg['current_lane_number']
        original_lane = cs['original_lane']
        
        # 智能返回时机判断
        return_delay = 5  # 基础延迟5秒
        if cfg['road_type'] == 'highway':
            return_delay = 3  # 高速公路更快返回
        
        # 检查是否超时（30秒内未完成返回）
        if time.time() - cs.get('return_start_time', time.time()) > 30:
            cs['return_to_original_lane_pending'] = False
            cs['original_lane'] = None
            cs['overtakeState'] = "返回原车道超时"
            cs['overtakeReason'] = "30秒内未完成返回原车道"
            return
        
        # 等待基础延迟时间
        if time.time() - cs.get('return_start_time', time.time()) < return_delay:
            return
        
        # 记录当前的成功计数，用于检测返回变道完成
        current_success_count = cs['overtakeSuccessCount']
        
        # 确定返回方向
        if current_lane < original_lane:
            return_direction = "RIGHT"
            safety_side = "right"
        else:
            return_direction = "LEFT" 
            safety_side = "left"
        
        # 检查返回方向的安全性
        is_safe, safety_reason = self.check_lane_safety(safety_side)
        
        if is_safe:
            # 发送返回指令
            success = self.send_command("LANECHANGE", return_direction)
            if success:
                cs['lane_change_in_progress'] = True
                cs['return_to_original_lane_pending'] = False
                cs['lastLaneChangeCommandTime'] = time.time() * 1000
                cs['return_start_count'] = current_success_count  # 记录返回开始时的计数
                
                direction_text = "左" if return_direction == "LEFT" else "右"
                cs['overtakeState'] = f"{direction_text}返回原车道 {original_lane}"
                cs['overtakeReason'] = "安全条件满足，正在返回原车道"
                print(f"🔄 开始返回原车道: {current_lane} → {original_lane}")
        else:
            # 安全条件不满足，等待
            cs['overtakeState'] = "等待返回原车道时机"
            cs['overtakeReason'] = f"返回条件不满足: {safety_reason}"

    def get_no_overtake_reasons(self):
        """获取未超车的具体原因"""
        vd = self.vehicle_data
        cfg = self.config
        cs = self.control_state
        
        reasons = []
        
        # 系统状态原因
        if vd['system_auto_control'] == 1:
            reasons.append("OP自动控制中")
            return reasons
        
        if not vd['IsOnroad']:
            reasons.append("车辆不在道路上")
            return reasons
        
        if not vd['engaged']:
            reasons.append("巡航未激活")
            return reasons
        
        # 前车状态原因
        if vd['lead_distance'] <= 0:
            reasons.append("前方无车辆")
            return reasons
        
        # 检查三个触发条件是否满足
        speed_ratio = vd['v_ego_kph'] / vd['v_cruise_kph'] if vd['v_cruise_kph'] > 0 else 1.0
        time_gap = self.calculate_time_gap()
        
        trigger_conditions_met = []
        
        # 前车相对速度
        if vd['lead_relative_speed'] < cfg['LEAD_RELATIVE_SPEED_THRESHOLD']:
            trigger_conditions_met.append("前车相对速度")
        
        # 跟车时间距离
        if 0 < time_gap <= cfg['FOLLOW_TIME_GAP_THRESHOLD']:
            trigger_conditions_met.append("跟车时间距离")
        
        # 速度比例
        if speed_ratio < cfg['CRUISE_SPEED_RATIO_THRESHOLD']:
            trigger_conditions_met.append("速度比例")
        
        # 如果没有触发条件满足
        if not trigger_conditions_met:
            reasons.append("未满足任何超车触发条件")
            # 添加具体数值
            reasons.append(f"相对速度:{vd['lead_relative_speed']}km/h(阈值:{cfg['LEAD_RELATIVE_SPEED_THRESHOLD']}km/h)")
            reasons.append(f"时间距离:{time_gap:.1f}秒(阈值:{cfg['FOLLOW_TIME_GAP_THRESHOLD']}秒)")
            reasons.append(f"速度比例:{speed_ratio*100:.0f}%(阈值:{cfg['CRUISE_SPEED_RATIO_THRESHOLD']*100:.0f}%)")
            return reasons
        
        # 最低速度原因
        if cfg['road_type'] == 'highway' and vd['v_ego_kph'] < cfg['HIGHWAY_MIN_SPEED']:
            reasons.append(f"高速车速{vd['v_ego_kph']}km/h过低(阈值:{cfg['HIGHWAY_MIN_SPEED']}km/h)")
        
        if cfg['road_type'] == 'normal' and vd['v_ego_kph'] < cfg['NORMAL_ROAD_MIN_SPEED']:
            reasons.append(f"普通路车速{vd['v_ego_kph']}km/h过低(阈值:{cfg['NORMAL_ROAD_MIN_SPEED']}km/h)")
        
        # 冷却时间原因
        now = time.time() * 1000
        if cs['lastOvertakeTime'] > 0 and now - cs['lastOvertakeTime'] < cs['dynamic_cooldown']:
            remaining = (cs['dynamic_cooldown'] - (now - cs['lastOvertakeTime'])) / 1000
            reasons.append(f"冷却时间剩余{remaining:.1f}秒")
        
        # 车道安全性原因
        available_directions = self.get_available_overtake_directions()
        if not available_directions:
            reasons.append("无可用变道方向")
        else:
            for direction in available_directions:
                side = "left" if direction == "LEFT" else "right"
                safety_score, safety_analysis = self.evaluate_lane_suitability(side)
                is_effective, effectiveness_score, effectiveness_reasons = self.is_overtake_effective(direction)
                
                if not is_effective:
                    reasons.append(f"{direction}车道无效超车")
                elif safety_score < cfg['PENALTY_THRESHOLD']:
                    reasons.append(f"{direction}车道不安全")
        
        # 如果有触发条件但被其他原因阻止，显示触发条件
        if trigger_conditions_met and reasons:
            reasons.insert(0, f"触发条件: {', '.join(trigger_conditions_met)}")
        
        return reasons

    def send_command(self, cmd_type, arg):
        self.cmd_index += 1
        """发送控制命令"""
        command = {
            "index": self.cmd_index,
            "cmd": cmd_type,
            "arg": arg,
            "timestamp": int(time.time() * 1000)
        }

        try:
            message = json.dumps(command).encode('utf-8')
            self.udp_socket.sendto(message, (self.remote_ip, self.remote_port))
            self.control_state['last_command'] = f"{cmd_type}: {arg}"
            self.last_command_time = time.time()
            print(f"📤 发送指令: {command}")
            return True
        except Exception as e:
            print(f"❌ 发送指令错误: {e}")
            return False

    def manual_overtake(self, lane):
        """手动变道 - 强制发送指令，不受system_auto_control影响"""
        # 记录当前的成功计数
        current_success_count = self.control_state['overtakeSuccessCount']
        
        direction = "LEFT" if lane == "left" else "RIGHT"
        success = self.send_command("LANECHANGE", direction)
        if success:
            self.control_state['lastOvertakeDirection'] = direction
            self.control_state['lastLaneChangeCommandTime'] = time.time() * 1000
            self.control_state['manual_start_count'] = current_success_count  # 记录手动变道开始计数

            if lane == "left":
                self.control_state['current_status'] = "强制左变道"
                self.control_state['overtakeState'] = "← 手动左变道"
            else:
                self.control_state['current_status'] = "强制右变道"
                self.control_state['overtakeState'] = "→ 手动右变道"
            self.control_state['overtakeReason'] = "用户强制变道指令（忽略系统自动控制）"
            print(f"🔧 手动变道指令: {direction} | 开始计数: {current_success_count}")

    def check_manual_lane_change_completion(self):
        """检查手动变道是否完成"""
        cs = self.control_state
        
        if cs.get('manual_start_count') is not None:
            current_count = cs['overtakeSuccessCount']
            start_count = cs['manual_start_count']
            
            # 如果成功计数增加了，说明手动变道完成
            if current_count > start_count:
                direction = cs['lastOvertakeDirection']
                direction_text = "左" if direction == "LEFT" else "右"
                cs['current_status'] = f"手动{direction_text}变道完成"
                cs['overtakeState'] = f"手动{direction_text}变道完成"
                cs['overtakeReason'] = "手动变道完成"
                print(f"✅ 手动变道完成: {direction_text}变道 | 计数: {start_count} → {current_count}")
                
                # 清除手动变道开始计数
                del cs['manual_start_count']

    def cancel_overtake(self):
        """取消超车"""
        success = self.send_command("CANCEL_OVERTAKE", "true")
        if success:
            self.control_state['current_status'] = "取消超车"
            self.control_state['isOvertaking'] = False
            self.control_state['lane_change_in_progress'] = False
            self.control_state['overtakingCompleted'] = False

    def change_speed(self, direction):
        """改变速度"""
        if direction == "UP":
            self.send_command("SPEED", direction)
        elif direction == "DOWN":
            self.send_command("SPEED", direction)

    def update_lane_number(self):
        """更新车道编号"""
        vd = self.vehicle_data
        cfg = self.config

        if vd['r_lane_width'] > 0 and vd['r_edge_dist'] > 0:
            calculated_lane = round((vd['r_edge_dist'] / vd['r_lane_width']) + 0.5)
            calculated_lane = max(1, min(cfg['lane_count'], calculated_lane))
            if calculated_lane != cfg['current_lane_number']:
                cfg['current_lane_number'] = calculated_lane

    def update_curve_detection(self):
        """更新弯道检测"""
        vd = self.vehicle_data
        cfg = self.config

        is_curve = (vd['max_curve'] >= 1.0 or
                   abs(vd['road_curvature']) > cfg['CURVATURE_THRESHOLD'] or
                   abs(vd['steering_angle']) > cfg['STEERING_THRESHOLD'])

        if is_curve and self.control_state['isOvertaking']:
            self.cancel_overtake()
            self.control_state['current_status'] = "弯道中取消超车"
            self.control_state['overtakeReason'] = "检测到弯道，安全第一"

    def run_data_loop(self):
        """数据循环"""
        ratekeeper = Ratekeeper(10)  # 10Hz

        while self.running:
            try:
                self.update_vehicle_data()
                self.update_lane_number()
                self.update_curve_detection()
                self.update_following_status()  # 新增：更新跟车状态

                if self.config['autoOvertakeEnabled']:
                    self.perform_auto_overtake()
                    self.check_overtake_completion()
                    self.check_return_to_original_lane()

                # 检查手动变道是否完成
                self.check_manual_lane_change_completion()

                ratekeeper.keep_time()
            except Exception as e:
                print(f"数据循环错误: {e}")
                time.sleep(0.1)

    def get_status_data(self):
        """获取状态数据 - 包含详细的触发条件和未超车原因"""
        vd = self.vehicle_data
        cs = self.control_state
        cfg = self.config
        
        # 计算时间距离和速度比例
        time_gap = self.calculate_time_gap()
        speed_ratio = vd['v_ego_kph'] / vd['v_cruise_kph'] if vd['v_cruise_kph'] > 0 else 1.0
        
        # 计算剩余冷却时间
        remaining_cooldown = 0
        now = time.time() * 1000
        if cs['lastOvertakeTime'] > 0:
            elapsed = now - cs['lastOvertakeTime']
            remaining_cooldown = max(0, cs['dynamic_cooldown'] - elapsed) / 1000
        
        # 获取触发条件和未超车原因
        trigger_conditions = self.get_trigger_conditions()
        no_overtake_reasons = self.get_no_overtake_reasons()
        
        # 检查车道宽度警告
        left_lane_narrow = vd.get('l_lane_width', 3.2) < cfg.get('MIN_LANE_WIDTH', 2.5)
        right_lane_narrow = vd.get('r_lane_width', 3.2) < cfg.get('MIN_LANE_WIDTH', 2.5)

        return {
            'w': True,
            'ip': self.get_local_ip(),
            's': vd.get('v_ego_kph', 0),
            'c': vd.get('v_cruise_kph', 0),
            'd': vd.get('desire_speed', 0),
            'ls': vd.get('lead_speed', 0),
            'ld': vd.get('lead_distance', 0),
            'lrs': vd.get('lead_relative_speed', 0),
            'lb': bool(vd.get('left_blindspot', False)),
            'rb': bool(vd.get('right_blindspot', False)),
            'l_front_blind': bool(vd.get('l_front_blind', False)),
            'r_front_blind': bool(vd.get('r_front_blind', False)),
            'llw': float(vd.get('l_lane_width', 3.2)),
            'rlw': float(vd.get('r_lane_width', 3.2)),
            'led': float(vd.get('l_edge_dist', 1.5)),
            'red': float(vd.get('r_edge_dist', 1.5)),
            'lls': vd.get('left_lead_speed', 0),
            'lld': vd.get('left_lead_distance', 0),
            'llrs': vd.get('left_lead_relative_speed', 0),
            'rls': vd.get('right_lead_speed', 0),
            'rld': vd.get('right_lead_distance', 0),
            'rlrs': vd.get('right_lead_relative_speed', 0),
            'rt': cfg.get('road_type', 'highway'),
            'lc': cfg.get('lane_count', 3),
            'cl': cfg.get('current_lane_number', 2),
            'alc': cfg.get('autoLaneCountEnabled', True),
            'os': cs.get('overtakeState', '等待超车条件'),
            'or': cs.get('overtakeReason', '分析道路情况中...'),
            'oc': cs.get('overtakeSuccessCount', 0),
            'hms': cfg.get('HIGHWAY_MIN_SPEED', 75),
            'nms': cfg.get('NORMAL_ROAD_MIN_SPEED', 40),
            'sr': cfg.get('CRUISE_SPEED_RATIO_THRESHOLD', 0.8),
            'ftg': cfg.get('FOLLOW_TIME_GAP_THRESHOLD', 3.0),
            'mft': cfg.get('MAX_FOLLOW_TIME', 120000),
            'mlw': cfg.get('MIN_LANE_WIDTH', 2.5),
            'slw': cfg.get('SAFE_LANE_WIDTH', 3.0),
            'sld': cfg.get('SIDE_LEAD_DISTANCE_MIN', 15),
            'srs': cfg.get('SIDE_RELATIVE_SPEED_THRESHOLD', 20),
            'aoe': cfg.get('autoOvertakeEnabled', True),
            'srtl': cfg.get('shouldReturnToLane', True),
            'lrs_threshold': cfg.get('LEAD_RELATIVE_SPEED_THRESHOLD', -5.0),
            'left_lane_narrow': left_lane_narrow,
            'right_lane_narrow': right_lane_narrow,
            'system_auto_control': vd.get('system_auto_control', 0),
            'original_lane': cs.get('original_lane'),
            'return_pending': cs.get('return_to_original_lane_pending', False),
            'should_return': cfg.get('shouldReturnToLane', True),
            'remaining_cooldown': remaining_cooldown,
            'dynamic_cooldown': cs.get('dynamic_cooldown', 8000),
            'last_overtake_result': cs.get('last_overtake_result', 'none'),
            'consecutive_failures': cs.get('consecutive_failures', 0),
            'time_gap': time_gap,
            'speed_ratio': speed_ratio,
            'sr_threshold': cfg.get('CRUISE_SPEED_RATIO_THRESHOLD', 0.8),
            'trigger_conditions': trigger_conditions,
            'no_overtake_reasons': no_overtake_reasons
        }

    def get_local_ip(self):
        """获取本地IP地址"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "127.0.0.1"

    def start_web_server(self):
        """启动Web服务器"""
        from http.server import HTTPServer
        handler = self.create_web_handler()
        self.web_server = HTTPServer(('0.0.0.0', 8088), handler)
        print("🌐 Web服务器启动在端口 8088")
        self.web_server.serve_forever()

    def create_web_handler(self):
        """创建Web处理器"""
        controller = self

        class OvertakeHTTPHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == '/':
                    self.send_html_response()
                elif self.path == '/status':
                    self.send_json_status()
                else:
                  print(f"page {self.path} not found!")

            def do_POST(self):
                try:
                    content_length = int(self.headers.get('Content-Length', 0))
                    post_data = self.rfile.read(content_length).decode('utf-8')
                    data = json.loads(post_data) if post_data else {}

                    if self.path == '/control':
                        self.handle_control(data)
                    elif self.path == '/overtake':
                        self.handle_overtake(data)
                    elif self.path == '/config':
                        self.handle_config(data)
                    elif self.path == '/params':
                        self.handle_params(data)
                    else:
                        self.send_error(404, "接口未找到")
                except Exception as e:
                    print(f"请求处理错误: {e}")
                    self.send_error(400, "请求解析错误")

            def handle_control(self, data):
                cmd_type = data.get('type', '')
                value = data.get('value', '')
                if cmd_type == 'SPEED':
                    controller.change_speed(value)
                self.send_json_response({'status': 'success', 'command': f'{cmd_type}: {value}'})

            def handle_overtake(self, data):
                if 'manual' in data:
                    controller.manual_overtake(data['manual'])
                    self.send_json_response({'status': 'success', 'action': f'manual_{data["manual"]}'})
                elif 'cancel' in data:
                    controller.cancel_overtake()
                    self.send_json_response({'status': 'success', 'action': 'cancel'})
                elif 'auto' in data:
                    controller.config['autoOvertakeEnabled'] = bool(data['auto'])
                    controller.save_persistent_config()
                    self.send_json_response({'status': 'success', 'autoOvertake': controller.config['autoOvertakeEnabled']})
                elif 'return' in data:
                    controller.config['shouldReturnToLane'] = bool(data['return'])
                    controller.save_persistent_config()
                    self.send_json_response({'status': 'success', 'returnToLane': controller.config['shouldReturnToLane']})
                else:
                    self.send_json_response({'status': 'error', 'message': '未知操作'})

            def handle_config(self, data):
                if 'lanes' in data and not controller.config['autoLaneCountEnabled']:
                    controller.config['lane_count'] = int(data['lanes'])
                if 'road_type' in data:
                    controller.config['road_type'] = data['road_type']
                    controller.save_persistent_config()
                if 'auto_lane_count' in data:
                    controller.config['autoLaneCountEnabled'] = bool(data['auto_lane_count'])
                    controller.save_persistent_config()
                self.send_json_response({'status': 'success', 'config': controller.config})

            def handle_params(self, data):
                param_map = {
                    'highwayMinSpeed': 'HIGHWAY_MIN_SPEED',
                    'normalMinSpeed': 'NORMAL_ROAD_MIN_SPEED',
                    'speedRatio': 'CRUISE_SPEED_RATIO_THRESHOLD',
                    'followTimeGap': 'FOLLOW_TIME_GAP_THRESHOLD',
                    'maxFollowTime': 'MAX_FOLLOW_TIME',
                    'minLaneWidth': 'MIN_LANE_WIDTH',
                    'safeLaneWidth': 'SAFE_LANE_WIDTH',
                    'sideLeadDist': 'SIDE_LEAD_DISTANCE_MIN',
                    'sideRelSpeed': 'SIDE_RELATIVE_SPEED_THRESHOLD',
                    'leadRelSpeed': 'LEAD_RELATIVE_SPEED_THRESHOLD'
                }

                for web_key, config_key in param_map.items():
                    if web_key in data:
                        if web_key == 'maxFollowTime':
                            # 将分钟转换为毫秒
                            controller.config[config_key] = int(data[web_key]) * 60 * 1
                        elif web_key == 'speedRatio':
                            # 将百分比转换为小数
                            controller.config[config_key] = float(data[web_key]) / 100.0
                        else:
                            controller.config[config_key] = float(data[web_key])

                controller.save_persistent_config()
                self.send_json_response({'status': 'success', 'message': '参数已保存'})

            def send_html_response(self):
                html = self.get_html_content()
                self.send_response(200)
                self.send_header('Content-type', 'text/html; charset=utf-8')
                self.end_headers()
                self.wfile.write(html.encode('utf-8'))

            def send_json_status(self):
                status_data = controller.get_status_data()
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(status_data, ensure_ascii=False).encode('utf-8'))

            def send_json_response(self, data):
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(data, ensure_ascii=False).encode('utf-8'))

            def get_html_content(self):
                # 读取HTML文件内容
                html_file_path = os.path.join(os.path.dirname(__file__), 'web_interface.html')
                try:
                    with open(html_file_path, 'r', encoding='utf-8') as f:
                        return f.read()
                except FileNotFoundError:
                    return "<html><body><h1>错误：未找到HTML界面文件</h1></body></html>"

            def log_message(self, format, *args):
                pass

        return OvertakeHTTPHandler

    def start(self):
        print("🚗 启动现代汽车自动超车控制器...")
        self.data_thread = threading.Thread(target=self.run_data_loop, daemon=True)
        self.data_thread.start()
        self.start_web_server()

    def stop(self):
        self.running = False
        if self.web_server:
            self.web_server.shutdown()
        if self.udp_socket:
            self.udp_socket.close()
        print("现代汽车自动超车控制器已停止")

def main():
    print("="*50)
    print("现代汽车自动超车控制器")
    print("访问地址: http://<op_ip>:8088")
    print("="*50)

    controller = AutoOvertakeController()
    try:
        controller.start()
    except KeyboardInterrupt:
        print("\n收到停止信号...")
    except Exception as e:
        print(f"运行错误: {e}")
    finally:
        controller.stop()

if __name__ == "__main__":
    main()