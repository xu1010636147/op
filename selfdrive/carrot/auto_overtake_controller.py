#!/usr/bin/env python3
"""
自动超车主控制器 - 修复完整版
集成所有模块的核心控制器类
"""

import os
import sys
import json
import time
import threading
import socket
import math
from collections import Counter, deque

# 导入OpenPilot消息类型
from cereal import log
LaneChangeState = log.LaneChangeState

# 导入自定义模块
try:
    from selfdrive.carrot.auto_overtake.config import Config
    from selfdrive.carrot.auto_overtake.vehicle_tracker import SideVehicleTracker
    from selfdrive.carrot.auto_overtake.lane_change_verification import LaneChangeVerificationSystem
    from selfdrive.carrot.auto_overtake.overtake_decision import OvertakeDecisionEngine
    from selfdrive.carrot.auto_overtake.return_strategy import ReturnStrategy
    from selfdrive.carrot.auto_overtake.status_management import StatusManager
    from selfdrive.carrot.auto_overtake.web_interface import WebInterface
except ImportError:
    # 备用导入方式
    import sys
    import os
    current_dir = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, current_dir)
    
    from config import Config
    from vehicle_tracker import SideVehicleTracker
    from lane_change_verification import LaneChangeVerificationSystem
    from overtake_decision import OvertakeDecisionEngine
    from return_strategy import ReturnStrategy
    from status_management import StatusManager
    from web_interface import WebInterface

# 导入OpenPilot相关
try:
    import cereal.messaging as messaging
    from common.realtime import Ratekeeper
    from common.params import Params
    from common.filter_simple import FirstOrderFilter
    OP_AVAILABLE = True
    print("✅ OpenPilot环境检测成功")
except ImportError:
    print("❌ 错误：未找到OpenPilot环境")
    sys.exit(1)

DEBUG = False
def debug_print(*args, **kwargs):
    if DEBUG:
        print(*args, **kwargs)

class AutoOvertakeController:
    """
    自动超车控制器主类 - v3.7 多源验证与远距离超车优化版
    """

    def __init__(self):
        """初始化自动超车控制器"""
        # 初始化各个模块
        self.config_manager = Config()
        self.config = self.config_manager.config
        self.status_manager = StatusManager()
        self.vehicle_data = self.status_manager.vehicle_data
        self.control_state = self.status_manager.control_state
        
        # 初始化功能模块
        self.verification_system = LaneChangeVerificationSystem()
        self.overtake_decision = OvertakeDecisionEngine(self.config)
        self.return_strategy = ReturnStrategy(self.config)
        self.web_interface = WebInterface(self)
        
        # 状态变量初始化
        self.lane_change_cnt = 0
        self.lane_change_finishing = False
        self.last_lane_count_calc = 0

        # 消息系统初始化
        self.pm = messaging.PubMaster(['autoOvertake'])
        self.sm = messaging.SubMaster([
            'carState', 'carControl', 'radarState',
            'modelV2', 'selfdriveState', 'liveLocationKalman', 'carrotMan'
        ])
        self.params = Params()

        # 侧方车辆跟踪器
        self.left_tracker = SideVehicleTracker('left')
        self.right_tracker = SideVehicleTracker('right')

        # UDP客户端用于发送指令
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.remote_ip = "127.0.0.1"  # 目标IP地址
        self.remote_port = 4211       # 目标端口

        # 指令索引和计时
        self.cmd_index = 0
        self.last_command_time = 0

        # 线程控制
        self.running = True
        self.data_thread = None
        self.web_server = None

        print("✅ 控制器初始化完成 - v3.7 多源验证与远距离超车优化版")

    def calculate_lane_count(self):
        """根据当前模式计算车道总数"""
        mode = self.config['lane_count_mode']

        if mode == 'manual':
            self.config['lane_count'] = self.config['manual_lane_count']
            return self.config['manual_lane_count']
        elif mode == 'auto':
            lane_count = self._calculate_auto_lane_count()
            self.config['lane_count'] = lane_count
            return lane_count
        elif mode == 'op':
            op_lane_count = self._get_op_lane_count()
            if op_lane_count is not None:
                self.config['lane_count'] = op_lane_count
                return op_lane_count
            else:
                debug_print("⚠️ OP车道总数获取失败，使用自动模式")
                lane_count = self._calculate_auto_lane_count()
                self.config['lane_count'] = lane_count
                return lane_count

        self.config['lane_count'] = 3
        return 3

    def _calculate_auto_lane_count(self):
        """自动计算车道总数"""
        vd = self.vehicle_data

        left_edge_dist = vd.get('l_edge_dist', 0)
        right_edge_dist = vd.get('r_edge_dist', 0)
        left_lane_width = vd.get('l_lane_width', 3.2)
        right_lane_width = vd.get('r_lane_width', 3.2)

        avg_lane_width = (left_lane_width + right_lane_width) / 2
        if avg_lane_width <= 0:
            avg_lane_width = 3.2

        total_road_width = left_edge_dist + right_edge_dist

        if total_road_width > 0 and avg_lane_width > 0:
            estimated_lanes = total_road_width / avg_lane_width

            self.verification_system.lane_count_history.append(estimated_lanes)
            if len(self.verification_system.lane_count_history) > self.verification_system.max_history_size:
                self.verification_system.lane_count_history.pop(0)

            smoothed_lanes = sum(self.verification_system.lane_count_history) / len(self.verification_system.lane_count_history)

            lane_count = max(2, min(5, round(smoothed_lanes)))

            if self.config['road_type'] == 'highway':
                lane_count = max(2, min(4, lane_count))
            else:
                lane_count = max(2, min(3, lane_count))

            debug_print(f"🛣️ 自动计算车道总数: {estimated_lanes:.1f} → {lane_count}车道")
            return lane_count
        else:
            default_lanes = 3 if self.config['road_type'] == 'highway' else 2
            debug_print(f"⚠️ 自动计算数据不足，使用默认值: {default_lanes}车道")
            return default_lanes

    def _get_op_lane_count(self):
        """从OpenPilot获取车道总数"""
        try:
            if self.sm.alive['modelV2']:
                return None
            return None
        except Exception as e:
            debug_print(f"❌ 获取OP车道总数失败: {e}")
            return None

    def update_lane_number(self):
        """更新车道编号 - 增强稳定性检测"""
        vd = self.vehicle_data

        self.calculate_lane_count()

        left_lane_width = vd.get('l_lane_width', 3.2)
        right_lane_width = vd.get('r_lane_width', 3.2)
        left_edge_dist = vd.get('l_edge_dist', 1.5)
        right_edge_dist = vd.get('r_edge_dist', 1.5)

        total_lanes = self.config['lane_count']

        avg_lane_width = (left_lane_width + right_lane_width) / 2
        if avg_lane_width <= 0:
            avg_lane_width = 3.2

        if left_edge_dist > 0 and right_edge_dist > 0 and avg_lane_width > 0:
            total_road_width = left_edge_dist + right_edge_dist
            relative_position = left_edge_dist / total_road_width

            lane_number = 1 + round(relative_position * (total_lanes - 1))
            lane_number = max(1, min(total_lanes, lane_number))

            # 🆕 增强稳定性检测
            self.verification_system.lane_number_history.append(lane_number)
            if len(self.verification_system.lane_number_history) > self.verification_system.max_history_size:
                self.verification_system.lane_number_history.pop(0)

            # 🆕 关键改进：只有当车道序号稳定时才更新
            if len(self.verification_system.lane_number_history) >= 3:
                # 检查最近3次读数是否一致
                recent_lanes = self.verification_system.lane_number_history[-3:]
                if len(set(recent_lanes)) == 1:  # 最近3次读数相同
                    stable_lane = recent_lanes[0]
                    if stable_lane != self.config['current_lane_number']:
                        self.config['current_lane_number'] = stable_lane
                        debug_print(f"🛣️ 稳定车道编号: {stable_lane} (总数: {total_lanes})")
                else:
                    # 读数不稳定，使用众数
                    counter = Counter(self.verification_system.lane_number_history)
                    most_common_lane, count = counter.most_common(1)[0]
                    if count > len(self.verification_system.lane_number_history) * 0.6:  # 超过60%的读数相同
                        if most_common_lane != self.config['current_lane_number']:
                            self.config['current_lane_number'] = most_common_lane
                            debug_print(f"🛣️ 众数车道编号: {most_common_lane} (置信度: {count}/{len(self.verification_system.lane_number_history)})")
            else:
                # 数据不足时直接使用
                if lane_number != self.config['current_lane_number']:
                    self.config['current_lane_number'] = lane_number
                    debug_print(f"🛣️ 初始车道编号: {lane_number} (总数: {total_lanes})")
        else:
            # 数据无效时保持原车道
            debug_print(f"⚠️ 车道数据无效，保持车道: {self.config['current_lane_number']}")

    def update_vehicle_data(self):
        """更新车辆数据 - 优化侧车数据准确性"""
        try:
            isOnroad = self.params.get_bool("IsOnroad")
            self.vehicle_data['IsOnroad'] = isOnroad

            if isOnroad:
                self.sm.update(100)
            else:
                self.sm.update(0)

            if isOnroad:
                if self.sm.alive['carState']:
                    carState = self.sm['carState']

                    v_ego_kph = int(carState.vEgoCluster * 3.6 + 0.5) if hasattr(carState, "vEgoCluster") and carState.vEgoCluster else 0
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

                    if carState.aEgo:
                        self.vehicle_data['lat_a'] = round(carState.aEgo, 1)

                if self.sm.alive['radarState']:
                    radarState = self.sm['radarState']

                    if radarState.leadOne.status:
                        leadOne = radarState.leadOne
                        self.vehicle_data.update({
                            'lead_distance': int(leadOne.dRel),
                            'lead_speed': int(leadOne.vLead * 3.6),
                            'lead_relative_speed': int(leadOne.vRel * 3.6)
                        })
                    else:
                        self.vehicle_data.update({
                            'lead_distance': 0,
                            'lead_speed': 0,
                            'lead_relative_speed': 0
                        })

                    # 使用跟踪器更新侧方车辆数据
                    self._update_side_vehicle_data(radarState)

                self.vehicle_data['desire_speed'] = 90

            carrot_left_blind = False
            carrot_right_blind = False
            current_time = time.time() * 1000

            old_op_control = self.vehicle_data['system_auto_control']

            if self.sm.alive['carrotMan']:
                carrotMan = self.sm['carrotMan']
                is_op_controlling = ("none" not in carrotMan.atcType and
                                   "prepare" not in carrotMan.atcType and
                                   "standby" not in carrotMan.atcType and
                                   "隧道" not in carrotMan.szPosRoadName)

                if is_op_controlling:
                    if "隧道" in carrotMan.szPosRoadName:
                        self.vehicle_data['system_auto_control'] = 2
                    else:
                        self.vehicle_data['system_auto_control'] = 1
                    self.vehicle_data['last_op_control_time'] = current_time
                    if old_op_control == 0:
                        if "隧道" in carrotMan.szPosRoadName:
                            debug_print("🔄 隧道禁止超车，重置自动超车状态")
                        else:
                            debug_print("🔄 OP控制开始，重置自动超车状态")
                        self.control_state['op_control_cooldown'] = 0
                        self.control_state['last_op_control_end_time'] = 0
                else:
                    self.vehicle_data['system_auto_control'] = 0
                    if old_op_control >= 1:
                        self.control_state['last_op_control_end_time'] = current_time
                        self.control_state['op_control_cooldown'] = 3000
                        if old_op_control == 2:
                            debug_print(f"🔄 隧道结束，开始{self.control_state['op_control_cooldown']}ms冷却")
                        else:
                            debug_print(f"🔄 OP控制结束，开始{self.control_state['op_control_cooldown']}ms冷却")

                carrot_left_blind = carrotMan.leftBlind
                carrot_right_blind = carrotMan.rightBlind

                #道路类型
                if carrotMan.roadCate == 0 or carrotMan.roadCate == 1:
                    self.config['road_type'] = 'highway'
                else:
                    self.config['road_type'] = 'normal'

            if self.sm.alive['modelV2']:
                modelV2 = self.sm['modelV2']
                meta = modelV2.meta

                self.vehicle_data.update({
                    'blinker': meta.blinker,
                    'l_front_blind': meta.leftFrontBlind or carrot_left_blind,
                    'r_front_blind': meta.rightFrontBlind or carrot_right_blind,
                    'l_lane_width': round(meta.laneWidthLeft, 1),
                    'r_lane_width': round(meta.laneWidthRight, 1),
                    'l_edge_dist': round(meta.distanceToRoadEdgeLeft, 1),
                    'r_edge_dist': round(meta.distanceToRoadEdgeRight, 1)
                })

                if self.lane_change_finishing and meta.laneChangeState != LaneChangeState.laneChangeFinishing:
                    self.lane_change_cnt += 1
                    self.control_state['overtakeSuccessCount'] += 1
                    self.lane_change_finishing = False
                    debug_print("🔄 检测到变道完成，强制更新车道序号")
                    self.update_lane_number()
                if meta.laneChangeState == LaneChangeState.laneChangeFinishing:
                    self.lane_change_finishing = True

            if self.sm.alive['selfdriveState']:
                selfdriveState = self.sm['selfdriveState']
                self.vehicle_data['active'] = "on" if selfdriveState.active else "off"

            # 基于多源验证修正净变道数
            self.verification_system.update_lane_based_net_changes(
                self.config['current_lane_number'],
                self.verification_system.lane_change_verification['last_confirmed_lane'],
                self.vehicle_data,
                self.config,
                self.control_state
            )

        except Exception as e:
            debug_print(f"更新车辆数据错误: {e}")

    def _update_side_vehicle_data(self, radarState):
        """更新侧方车辆数据 - 使用跟踪器但保持原有字段"""
        try:
            # 使用跟踪器更新左右侧车辆数据
            self.left_tracker.update(radarState)
            self.right_tracker.update(radarState)
            
            # 获取滤波后的数据
            left_data = self.left_tracker.get_filtered_data()
            right_data = self.right_tracker.get_filtered_data()
            
            # 关键修复：使用跟踪器数据
            # 左侧车辆数据 - 使用跟踪器数据
            if left_data['distance'] > 0 and left_data['track_quality'] > 20:  # 跟踪质量阈值
                self.vehicle_data.update({
                    'left_lead_distance': int(left_data['distance']),
                    'left_lead_speed': int(left_data['speed']),
                    'left_lead_relative_speed': int(left_data['relative_speed'])
                })
            else:
                # 使用传统方法作为后备
                if radarState.leadLeft.status:
                    leadLeft = radarState.leadLeft
                    self.vehicle_data.update({
                        'left_lead_distance': int(leadLeft.dRel),
                        'left_lead_speed': int(leadLeft.vLead * 3.6),
                        'left_lead_relative_speed': int(leadLeft.vRel * 3.6)
                    })
                else:
                    self.vehicle_data.update({
                        'left_lead_distance': 0,
                        'left_lead_speed': 0,
                        'left_lead_relative_speed': 0
                    })
            
            # 右侧车辆数据 - 使用跟踪器数据
            if right_data['distance'] > 0 and right_data['track_quality'] > 20:  # 跟踪质量阈值
                self.vehicle_data.update({
                    'right_lead_distance': int(right_data['distance']),
                    'right_lead_speed': int(right_data['speed']),
                    'right_lead_relative_speed': int(right_data['relative_speed'])
                })
            else:
                # 使用传统方法作为后备
                if radarState.leadRight.status:
                    leadRight = radarState.leadRight
                    self.vehicle_data.update({
                        'right_lead_distance': int(leadRight.dRel),
                        'right_lead_speed': int(leadRight.vLead * 3.6),
                        'right_lead_relative_speed': int(leadRight.vRel * 3.6)
                    })
                else:
                    self.vehicle_data.update({
                        'right_lead_distance': 0,
                        'right_lead_speed': 0,
                        'right_lead_relative_speed': 0
                    })
                
            # 调试输出
            if DEBUG and (left_data['vehicle_count'] > 0 or right_data['vehicle_count'] > 0):
                debug_print(f"🚗 侧车跟踪: 左{left_data['vehicle_count']}辆(质量:{left_data['track_quality']}%), " +
                           f"右{right_data['vehicle_count']}辆(质量:{right_data['track_quality']}%)")
                
        except Exception as e:
            debug_print(f"❌ 更新侧车数据错误: {e}")
            # 出错时使用传统方法
            try:
                if radarState.leadLeft.status:
                    leadLeft = radarState.leadLeft
                    self.vehicle_data.update({
                        'left_lead_distance': int(leadLeft.dRel),
                        'left_lead_speed': int(leadLeft.vLead * 3.6),
                        'left_lead_relative_speed': int(leadLeft.vRel * 3.6)
                    })
                else:
                    self.vehicle_data.update({
                        'left_lead_distance': 0,
                        'left_lead_speed': 0,
                        'left_lead_relative_speed': 0
                    })

                if radarState.leadRight.status:
                    leadRight = radarState.leadRight
                    self.vehicle_data.update({
                        'right_lead_distance': int(leadRight.dRel),
                        'right_lead_speed': int(leadRight.vLead * 3.6),
                        'right_lead_relative_speed': int(leadRight.vRel * 3.6)
                    })
                else:
                    self.vehicle_data.update({
                        'right_lead_distance': 0,
                        'right_lead_speed': 0,
                        'right_lead_relative_speed': 0
                    })
            except Exception as e2:
                debug_print(f"❌ 传统侧车数据更新也失败: {e2}")

    def _get_blinker_state(self, left_blinker, right_blinker):
        """获取转向灯状态"""
        if left_blinker and right_blinker:
            return "hazard"
        elif left_blinker:
            return "left"
        elif right_blinker:
            return "right"
        else:
            return "none"

    def update_following_status(self):
        """更新跟车状态"""
        self.overtake_decision.update_following_status(self.vehicle_data, self.control_state)

    def update_curve_detection(self):
        """更新弯道检测"""
        vd = self.vehicle_data

        is_curve = (vd['max_curve'] >= 1.0 or
                   abs(vd['road_curvature']) > self.config['CURVATURE_THRESHOLD'] or
                   abs(vd['steering_angle']) > self.config['STEERING_THRESHOLD'])

        if is_curve and self.control_state['isOvertaking']:
            self.cancel_overtake()
            self.control_state['current_status'] = "弯道中取消超车"
            self.control_state['overtakeReason'] = "检测到弯道，安全第一"

    def _execute_overtake_decision(self):
        """执行超车决策 - 修复版：加强有效性检查"""
        available_directions = self.overtake_decision.get_available_overtake_directions(self.vehicle_data)

        if not available_directions:
            self.control_state['overtakeState'] = "无可用变道方向"
            self.control_state['overtakeReason'] = "当前车道位置限制"
            return

        self.control_state['return_timer_start'] = 0
        self.control_state['return_conditions_met'] = False
        self.control_state['original_lane_clear'] = False

        current_penalty, current_analysis = self.overtake_decision.get_current_lane_penalty(self.vehicle_data)

        direction_scores = {}
        direction_analysis = {}
        direction_effectiveness = {}

        for direction in available_directions:
            side = "left" if direction == "LEFT" else "right"

            safety_score, safety_analysis = self.overtake_decision.evaluate_lane_suitability(self.vehicle_data, side)
            is_effective, effectiveness_score, effectiveness_reasons = self.overtake_decision.is_overtake_effective(self.vehicle_data, direction)

            # 关键修复：只有真正有效的超车才考虑
            if not is_effective:
                direction_scores[direction] = 0  # 无效超车得分为0
                direction_effectiveness[direction] = {
                    'score': effectiveness_score,
                    'is_effective': False,
                    'reasons': effectiveness_reasons
                }
                full_analysis = [f"❌ 超车无效: {', '.join(effectiveness_reasons)}"]
                direction_analysis[direction] = full_analysis
                continue

            # 有效超车的评分计算
            effectiveness_factor = effectiveness_score / 100.0
            combined_score = safety_score * effectiveness_factor

            direction_scores[direction] = combined_score
            direction_effectiveness[direction] = {
                'score': effectiveness_score,
                'is_effective': True,
                'reasons': effectiveness_reasons
            }

            full_analysis = safety_analysis.copy()
            if effectiveness_reasons:
                full_analysis.extend([f"✅ {reason}" for reason in effectiveness_reasons if '优势充足' in reason])
                full_analysis.extend([f"⚠️ {reason}" for reason in effectiveness_reasons if '优势不足' in reason or '通常较慢' in reason])
            
            full_analysis.append(f"🎯 综合评分: {combined_score:.1f}%")
            direction_analysis[direction] = full_analysis

        # 选择最佳方向（只考虑有效超车）
        best_direction = None
        best_score = 0
        detailed_reason = ""

        for direction in available_directions:
            score = direction_scores[direction]
            effectiveness_info = direction_effectiveness[direction]

            # 只考虑有效超车
            if not effectiveness_info['is_effective']:
                continue

            # 道路类型特殊处理
            if self.config['road_type'] == 'highway':
                current_lane = self.config['current_lane_number']

                if current_lane == self.config['lane_count'] and direction == "LEFT":
                    score += 15  # 最右车道优先向左，但幅度减小
                    direction_analysis[direction].append("🔄 最右车道优先向左")

                elif current_lane == 1 and direction == "RIGHT":
                    score -= 10  # 快车道向右惩罚减小
                    direction_analysis[direction].append("⚠️ 快车道向右需谨慎")

            if score > self.config['PENALTY_THRESHOLD'] and score > best_score:
                best_direction = direction
                best_score = score

                effectiveness_text = f"有效性{effectiveness_info['score']}%"
                safety_text = f"安全性{score:.1f}%"
                analysis_text = " | ".join(direction_analysis[direction])
                detailed_reason = f"{direction}车道 {effectiveness_text} | {safety_text} | {analysis_text}"

        # 执行超车决策
        if best_direction and best_score > self.config['PENALTY_THRESHOLD']:
            target_advantage = best_score - (100 - current_penalty)
            min_advantage = 8  # 提高最小优势要求

            if self.config['road_type'] == 'highway':
                min_advantage = 5

            # 检查是否真的有速度优势
            vd = self.vehicle_data
            if best_direction == "LEFT":
                target_speed = vd['left_lead_speed'] if vd['left_lead_speed'] > 0 else vd['v_ego_kph'] + 10
            else:
                target_speed = vd['right_lead_speed'] if vd['right_lead_speed'] > 0 else vd['v_ego_kph'] + 10
            
            current_speed_expected = vd['lead_speed'] if vd['lead_speed'] > 0 else vd['v_ego_kph']
            actual_speed_advantage = target_speed - current_speed_expected

            #  最终检查：确保目标车道确实比当前车道快
            if actual_speed_advantage >= 3:  # 至少3km/h的实际速度优势
                self.execute_overtake(best_direction)
                self.control_state['overtakeReason'] = f"{detailed_reason} | 实际速度优势: +{actual_speed_advantage:.1f}km/h"
                debug_print(f"🎯 智能车道选择: {best_direction}变道 | 评分: {best_score:.1f}% | 速度优势: +{actual_speed_advantage:.1f}km/h")
            else:
                self.control_state['overtakeState'] = "目标车道无速度优势"
                self.control_state['overtakeReason'] = f"目标车道速度{target_speed}km/h vs 当前{current_speed_expected}km/h (优势{actual_speed_advantage:.1f}km/h)"
        else:
            # 提供详细的未超车原因
            no_overtake_reasons = []
            for direction in available_directions:
                effectiveness_info = direction_effectiveness[direction]
                score = direction_scores[direction]
                
                if not effectiveness_info['is_effective']:
                    reason = f"{direction}:{effectiveness_info['reasons'][-1] if effectiveness_info['reasons'] else '无效超车'}"
                elif score <= self.config['PENALTY_THRESHOLD']:
                    reason = f"{direction}:安全性不足({score:.1f}%)"
                else:
                    reason = f"{direction}:条件满足但未选择"
                
                no_overtake_reasons.append(reason)

            self.control_state['overtakeState'] = "无合适超车车道"
            self.control_state['overtakeReason'] = f"车道分析: {', '.join(no_overtake_reasons)}"

    def perform_auto_overtake(self):
        """执行自动超车 - 彻底移除返回功能状态显示"""
        if not self.config['autoOvertakeEnabled'] or self.control_state['isOvertaking']:
            return

        # 修复：返回功能关闭时，只重置返回相关状态，不阻止超车
        if not self.config['shouldReturnToLane']:
            # 重置返回相关状态，但允许继续超车
            if self.control_state['net_lane_changes'] != 0:
                debug_print("🛣️ 返回功能已关闭，重置净变道次数，但允许超车")
                self.reset_net_lane_changes()

        if self.vehicle_data['system_auto_control'] >= 1:
            if self.vehicle_data['system_auto_control'] == 2:
                self.control_state['overtakeState'] = "隧道中"
                self.control_state['overtakeReason'] = "隧道中，暂停超车"
            else:
                self.control_state['overtakeState'] = "OP控制中"
                self.control_state['overtakeReason'] = "OP自动控制中，暂停超车"
            return

        if self.overtake_decision.check_op_control_cooldown(self.control_state):
            return

        if self.overtake_decision.check_overtake_conditions(self.vehicle_data, self.control_state):
            self._execute_overtake_decision()
            return

        # 修复：只有在返回功能启用时才执行返回逻辑
        if self.config['shouldReturnToLane']:
            road_type = self.config['road_type']
            return_enabled = self.config['RETURN_STRATEGY'][road_type]['enabled']

            if (return_enabled and
                self.control_state['net_lane_changes'] != 0 and
                self.control_state['is_auto_overtake']):

                return_ready = self.return_strategy.check_smart_return_conditions(
                    self.vehicle_data, self.control_state, self.config)
                if return_ready:
                    self.perform_smart_return()
                else:
                    self._handle_return_fallback()

                self.check_return_completion()

        # 新增：确保状态及时恢复
        self.status_manager.ensure_status_refresh(self.control_state)

    def _handle_return_fallback(self):
        """处理返回失败的情况"""
        # 关键修复：只有在返回功能启用时才处理返回失败
        if not self.config['shouldReturnToLane']:
            return

        current_time = time.time() * 1000
        if (self.control_state['return_timer_start'] > 0 and
            current_time - self.control_state['return_timer_start'] > 20000):

            debug_print("⏰ 返回条件长时间不满足，放弃本次返回")
            self.control_state['return_timer_start'] = 0
            self.control_state['return_attempts'] += 1

            if self.control_state['return_attempts'] >= self.control_state['max_return_attempts']:
                debug_print(f"⚠️ 达到最大返回尝试次数({self.control_state['max_return_attempts']})，放弃返回")
                self.reset_net_lane_changes()

    def perform_smart_return(self):
        """执行智能返回 - 优化版本"""
        if not self.control_state['return_conditions_met']:
            return

        # 🆕 基于原车道记忆确定返回方向
        current_lane = self.config['current_lane_number']
        target_lane = self.control_state['original_lane_number']
        
        if current_lane < target_lane:
            return_direction = "RIGHT"
        elif current_lane > target_lane:
            return_direction = "LEFT"
        else:
            # 已经在原车道，重置状态
            debug_print("✅ 已在原车道，重置状态")
            self.reset_net_lane_changes()
            return

        current_count = self.control_state['overtakeSuccessCount']
        success = self.send_command("OVERTAKE", return_direction)

        if success:
            self.control_state['lane_change_in_progress'] = True
            self.control_state['isOvertaking'] = True
            self.control_state['return_conditions_met'] = False
            self.control_state['return_attempts'] += 1
            self.control_state['lastLaneChangeCommandTime'] = time.time() * 1000
            self.control_state['return_start_count'] = current_count
            self.control_state['last_return_direction'] = return_direction

            # 重置返回相关状态
            self.control_state['target_vehicle_tracker'] = None
            self.control_state['overtake_complete_timer'] = 0
            self.control_state['consecutive_overtake_count'] = 0

            direction_text = "右" if return_direction == "RIGHT" else "左"
            attempt_text = f"第{self.control_state['return_attempts']}次"
            self.control_state['overtakeState'] = f"{attempt_text}{direction_text}返回"

            # 🆕 基于原车道的详细返回原因
            current_net = target_lane - current_lane
            self.control_state['overtakeReason'] = f"返回原车道{target_lane} (当前:{current_lane}, 需要{abs(current_net)}次{direction_text}变道)"

            debug_print(f"🔄 {attempt_text}返回: {direction_text}变道 | 目标车道:{target_lane}")

    def check_return_completion(self):
        """检查返回是否完成 - 验证是否回到原车道"""
        if not self.control_state.get('lane_change_in_progress') or self.control_state.get('return_start_count') is None:
            return

        current_count = self.control_state['overtakeSuccessCount']
        start_count = self.control_state['return_start_count']

        if current_count > start_count:
            self.control_state['lane_change_in_progress'] = False
            self.control_state['isOvertaking'] = False

            # 🆕 验证是否回到原车道
            current_lane = self.config['current_lane_number']
            target_lane = self.control_state['original_lane_number']
            
            if current_lane == target_lane:
                # 成功回到原车道
                self.control_state['net_lane_changes'] = 0
                self.control_state['last_auto_overtake_time'] = time.time() * 1000
                
                self.control_state['return_timer_start'] = 0
                self.control_state['original_lane_clear'] = False

                del self.control_state['return_start_count']

                self.control_state['overtakeState'] = f"返回原车道完成"
                self.control_state['overtakeReason'] = "返回完成，分析道路情况中..."
                self.control_state['current_status'] = "返回完成"

                debug_print(f"✅ 返回完成: 成功回到原车道{target_lane}")

                # 重置原车道记忆
                self.control_state['original_lane_number'] = 0
                self.control_state['lane_memory_start_time'] = 0
                self.control_state['return_timeout_timer'] = 0

                # 关键修复：重置超车冷却时间
                self.control_state['lastOvertakeTime'] = 0
                self.control_state['last_overtake_result'] = 'none'
                self.control_state['consecutive_failures'] = 0

            else:
                # 变道完成但未回到原车道，继续返回
                remaining_changes = target_lane - current_lane
                self.control_state['net_lane_changes'] = remaining_changes
                
                direction = "右" if remaining_changes > 0 else "左"
                self.control_state['overtakeState'] = f"继续返回原车道"
                self.control_state['overtakeReason'] = f"还需{abs(remaining_changes)}次{direction}变道回到原车道{target_lane}"
                
                debug_print(f"🔄 变道完成但未回到原车道，仍需{abs(remaining_changes)}次{direction}变道")
                
                # 重置返回尝试次数，允许继续尝试
                self.control_state['return_attempts'] = 0

    def execute_overtake(self, direction):
        """执行超车操作 - 优化版本"""
        current_success_count = self.control_state['overtakeSuccessCount']

        success = self.send_command("OVERTAKE", direction)
        if success:
            # 🆕 开始记录原车道（第一次超车时）
            if self.control_state['original_lane_number'] == 0:
                self.return_strategy.start_lane_memory(self.control_state, self.config['current_lane_number'])

            self.control_state['isOvertaking'] = True
            self.control_state['lane_change_in_progress'] = True
            self.control_state['lastOvertakeDirection'] = direction
            self.control_state['lastLaneChangeCommandTime'] = time.time() * 1000

            # 重置目标车辆跟踪（开始新的超车）
            self.control_state['target_vehicle_tracker'] = None
            self.control_state['overtake_complete_timer'] = 0

            road_type = self.config['road_type']
            
            # 修复：只有在返回功能启用时才记录净变道次数
            if self.config['shouldReturnToLane']:
                return_enabled = self.config['RETURN_STRATEGY'][road_type]['enabled']
                if return_enabled:
                    self.control_state['return_timer_start'] = 0
                    self.control_state['return_conditions_met'] = False
                    self.control_state['original_lane_clear'] = False
                    self.update_net_lane_changes(direction, is_auto_overtake=True)

                    debug_print(f"🛣️ 高速公路超车: {direction}变道 | 净变道: {self.control_state['net_lane_changes']}")
                else:
                    self.reset_net_lane_changes()
                    debug_print(f"🛣️ 普通道路超车: {direction}变道 | 返回功能已禁用")
            else:
                # 返回功能关闭时，不记录净变道次数
                debug_print(f"🛣️ 超车执行: {direction}变道 | 返回功能已关闭")

            self.control_state['follow_start_time'] = None
            self.control_state['is_following_slow_vehicle'] = False
            self.control_state['max_follow_time_reached'] = False

            self.control_state['overtake_start_count'] = current_success_count

            if direction == "LEFT":
                self.control_state['overtakeState'] = "← 准备向左变道超车"
                self.control_state['current_status'] = "自动左变道"
            else:
                self.control_state['overtakeState'] = "→ 准备向右变道超车"
                self.control_state['current_status'] = "自动右变道"

            debug_print(f"🚀 开始超车: {direction}变道 | 道路类型: {road_type}")

    def check_overtake_completion(self):
        """检查超车完成状态 - 修复状态显示"""
        if not self.control_state['lane_change_in_progress']:
            return

        current_count = self.control_state['overtakeSuccessCount']
        start_count = self.control_state.get('overtake_start_count', current_count)

        if current_count > start_count:
            self.control_state['isOvertaking'] = False
            self.control_state['lane_change_in_progress'] = False
            self.control_state['overtakingCompleted'] = True

            self.control_state['original_lane_clear'] = False

            self.control_state['lastOvertakeTime'] = time.time() * 1000
            self.control_state['last_overtake_result'] = 'success'

            direction = self.control_state['lastOvertakeDirection']
            direction_text = "左" if direction == "LEFT" else "右"
            net_changes = self.control_state['net_lane_changes']

            # 修复：超车完成后立即显示完成状态，然后快速恢复
            self.control_state['overtakeState'] = f"{direction_text}变道完成"
            self.control_state['overtakeReason'] = "变道完成，分析道路情况中..."
            self.control_state['current_status'] = "变道完成"

            debug_print(f"✅ 变道完成: {direction_text}变道成功 | 净变道: {net_changes}")

            # 设置定时器，2秒后恢复就绪状态
            self.control_state['completion_timer'] = time.time() * 1000

            if 'overtake_start_count' in self.control_state:
                del self.control_state['overtake_start_count']

        elif time.time() * 1000 - self.control_state['lastLaneChangeCommandTime'] > 15000:
            self.control_state['lane_change_in_progress'] = False
            self.control_state['isOvertaking'] = False

            self.control_state['lastOvertakeTime'] = time.time() * 1000
            self.control_state['last_overtake_result'] = 'failed'

            self.control_state['overtakeState'] = "变道超时"
            self.control_state['overtakeReason'] = "15秒内未检测到变道完成，快速重试"
            debug_print("❌ 变道超时，未检测到完成信号")

            # 设置定时器，3秒后恢复就绪状态
            self.control_state['completion_timer'] = time.time() * 1000

    def check_manual_lane_change_completion(self):
        """检查手动变道是否完成 - 修复状态显示"""
        if self.control_state.get('manual_start_count') is not None:
            current_count = self.control_state['overtakeSuccessCount']
            start_count = self.control_state['manual_start_count']

            if current_count > start_count:
                direction = self.control_state['lastOvertakeDirection']
                direction_text = "左" if direction == "LEFT" else "右"

                # 修复：手动变道完成后显示完成状态，然后快速恢复
                self.control_state['current_status'] = "手动变道完成"
                self.control_state['overtakeState'] = f"手动{direction_text}变道完成"
                self.control_state['overtakeReason'] = "手动变道完成，分析道路情况中..."
                self.control_state['isOvertaking'] = False
                self.control_state['lane_change_in_progress'] = False
                self.control_state['overtakingCompleted'] = False

                # 设置定时器，2秒后恢复就绪状态
                self.control_state['completion_timer'] = time.time() * 1000

                # 重置手动变道相关状态
                del self.control_state['manual_start_count']

                # 修复：确保自动超车功能可以继续工作
                self.control_state['lastOvertakeTime'] = time.time() * 1000
                self.control_state['last_overtake_result'] = 'success'

                debug_print(f"✅ 手动变道完成: {direction_text}变道 | 状态已重置，自动超车已恢复")

    def check_return_timeout(self):
        """检查返回超时"""
        current_time = time.time() * 1000

        # 修复：只有在返回功能启用时才检查返回超时
        if not self.config['shouldReturnToLane']:
            return False

        if self.control_state['net_lane_changes'] != 0 and self.control_state['last_auto_overtake_time'] > 0:
            time_since_last_auto = current_time - self.control_state['last_auto_overtake_time']
            if time_since_last_auto > self.control_state['return_timeout']:
                debug_print(f"⏰ 返回超时({time_since_last_auto/1000:.1f}秒)，清零净变道次数")
                self.reset_net_lane_changes()
                return True
        return False

    def update_net_lane_changes(self, direction, is_auto_overtake=True):
        """更新净变道次数"""
        # 修复：只有在返回功能启用时才更新净变道次数
        if not self.config['shouldReturnToLane']:
            self.reset_net_lane_changes()
            debug_print(f"🔄 返回功能已关闭，清零净变道次数")
            return

        if is_auto_overtake:
            if direction == "LEFT":
                self.control_state['net_lane_changes'] += 1
                self.control_state['lastOvertakeDirection'] = "LEFT"
                self.control_state['last_auto_overtake_time'] = time.time() * 1000
                self.control_state['is_auto_overtake'] = True
                debug_print(f"🔄 自动超车净变道次数更新: {self.control_state['net_lane_changes']} (方向: {direction})")
            elif direction == "RIGHT":
                self.control_state['net_lane_changes'] -= 1
                self.control_state['lastOvertakeDirection'] = "RIGHT"
                self.control_state['last_auto_overtake_time'] = time.time() * 1000
                self.control_state['is_auto_overtake'] = True
                debug_print(f"🔄 自动超车净变道次数更新: {self.control_state['net_lane_changes']} (方向: {direction})")
        else:
            self.reset_net_lane_changes()
            debug_print(f"🔄 手动变道，清零净变道次数")

    def reset_net_lane_changes(self):
        """重置净变道次数"""
        self.status_manager.reset_net_lane_changes(self.control_state, self.verification_system)

    def get_no_overtake_reasons(self):
        """获取未超车的具体原因"""
        return self.status_manager.get_no_overtake_reasons(
            self.vehicle_data, self.config, self.control_state, self.overtake_decision)

    def run_data_loop(self):
        """数据循环 - 集成所有改进"""
        ratekeeper = Ratekeeper(10)

        while self.running:
            try:
                self.update_vehicle_data()
                self.update_lane_number()
                self.update_curve_detection()
                self.update_following_status()

                current_time = time.time() * 1000
                
                # 🆕 多源验证系统状态监控
                if current_time % 5000 < 100:  # 每5秒检查一次
                    confidence = self.verification_system.lane_change_verification['confidence_score']
                    if confidence < self.verification_system.lane_change_verification['min_confidence']:
                        debug_print(f"⚠️ 系统置信度较低: {confidence}%，考虑重置")
                        # 可以在这里添加自动重置逻辑
                
                # 🆕 修复：使用正确的变量名和逻辑
                if current_time - self.last_lane_count_calc > 5000:
                    self.calculate_lane_count()
                    self.last_lane_count_calc = current_time

                self.check_return_timeout()

                # 🆕 定期检查原车道记忆超时
                if (self.control_state.get('original_lane_number', 0) > 0 and 
                    current_time - self.control_state.get('return_timeout_timer', 0) > 30000):
                    debug_print("⏰ 返回超时(30秒)，强制重置")
                    self.reset_net_lane_changes()

                # 🆕 调试信息输出
                if DEBUG and current_time % 10000 < 100:  # 每10秒输出一次调试信息
                    debug_print(f"🔍 系统状态: 置信度{self.verification_system.lane_change_verification['confidence_score']}%, " +
                               f"净变道{self.control_state['net_lane_changes']}, " +
                               f"车道{self.config['current_lane_number']}")

                # 新增：检查完成定时器，恢复就绪状态
                if self.control_state.get('completion_timer') and current_time - self.control_state['completion_timer'] > 2000:
                    # 完成状态显示2秒后恢复就绪
                    if not self.control_state['isOvertaking'] and not self.control_state['lane_change_in_progress']:
                        self.control_state['overtakeState'] = "等待超车条件"
                        self.control_state['overtakeReason'] = "分析道路情况中..."
                        self.control_state['current_status'] = "就绪"
                    del self.control_state['completion_timer']

                if ((self.config['autoOvertakeEnabled'] and self.config['road_type'] == 'highway') or
                    (self.config['autoOvertakeEnabledL'] and self.config['road_type'] != 'highway')):
                    self.perform_auto_overtake()
                    self.check_overtake_completion()

                    # 修复：确保返回完成检查被正确调用
                    if (self.config['shouldReturnToLane'] and 
                        self.control_state['net_lane_changes'] != 0 and
                        self.control_state['is_auto_overtake']):
                        self.check_return_completion()
                    else:
                        # 修复：如果没有净变道，确保状态正确
                        if (self.control_state['isOvertaking'] and
                            not self.control_state['lane_change_in_progress']):
                            self.control_state['isOvertaking'] = False
                            debug_print("🔄 检测到异常状态，重置超车标志")

                # 修复：确保手动变道完成检查被调用
                self.check_manual_lane_change_completion()

                # 新增：确保状态及时刷新
                self.status_manager.ensure_status_refresh(self.control_state)

                ratekeeper.keep_time()
            except Exception as e:
                print(f"数据循环错误: {e}")
                time.sleep(0.1)

    def get_status_data(self):
        """获取状态数据 - 保持与Web界面完全兼容"""
        return self.status_manager.get_status_data(self.vehicle_data, self.control_state, self.config, self.overtake_decision)

    def start(self):
        """启动控制器"""
        print("🚗 启动现代汽车自动超车控制器 v3.7...")
        print("🎯 多源验证净变道数计算系统")
        print("🚀 远距离超车触发条件")
        print("🛡️ 前车最低速度限制")
        print("="*50)
        
        self.data_thread = threading.Thread(target=self.run_data_loop, daemon=True)
        self.data_thread.start()
        self.web_interface.start_web_server()

    def stop(self):
        """停止控制器"""
        self.running = False
        if hasattr(self.web_interface, 'web_server') and self.web_interface.web_server:
            self.web_interface.web_server.shutdown()
        if self.udp_socket:
            self.udp_socket.close()
        print("现代汽车自动超车控制器已停止")

    def change_speed(self, direction):
        """改变速度"""
        self.send_command("SPEED", direction)

    def manual_overtake(self, lane):
        """手动变道"""
        direction = "LEFT" if lane == "left" else "RIGHT"
        success = self.send_command("LANECHANGE", direction)
        if success:
            # 修复：手动变道时正确设置状态
            self.control_state['lastOvertakeDirection'] = direction
            self.control_state['lastLaneChangeCommandTime'] = time.time() * 1000
            self.control_state['manual_start_count'] = self.control_state['overtakeSuccessCount']

            # 修复：确保手动变道不会阻塞自动超车
            self.control_state['isOvertaking'] = False
            self.control_state['lane_change_in_progress'] = False

            self.update_net_lane_changes(direction, is_auto_overtake=False)

            if lane == "left":
                self.control_state['current_status'] = "手动左变道中"
                self.control_state['overtakeState'] = "← 手动左变道"
            else:
                self.control_state['current_status'] = "手动右变道中"
                self.control_state['overtakeState'] = "→ 手动右变道"
            self.control_state['overtakeReason'] = "用户手动变道指令"
            debug_print(f"🔧 手动变道指令: {direction} | 等待变道完成")

    def cancel_overtake(self):
        """取消超车"""
        success = self.send_command("CANCEL_OVERTAKE", "true")
        if success:
            self.control_state['current_status'] = "取消超车"
            self.control_state['isOvertaking'] = False
            self.control_state['lane_change_in_progress'] = False
            self.control_state['overtakingCompleted'] = False

    def send_command(self, cmd_type, arg):
        """发送控制命令"""
        self.cmd_index += 1
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

    def save_persistent_config(self):
        """保存配置"""
        self.config_manager.save_persistent_config()
