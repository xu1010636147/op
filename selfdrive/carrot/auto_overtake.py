#!/usr/bin/env python3
"""
现代汽车自动变道超车控制器
集成到OpenPilot中的自动变道超车控制器
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

class OvertakeEffectivenessEvaluator:
    """超车有效性评估器"""

    def __init__(self, controller):
        self.controller = controller

    def evaluate_overtake_effectiveness(self, target_lane):
        """评估超车有效性"""
        vd = self.controller.vehicle_data

        # 获取当前车道和目标车道的前车信息
        current_lead_speed = vd['lead_speed']
        current_lead_distance = vd['lead_distance']

        if target_lane == "left":
            target_lead_speed = vd['left_lead_speed']
            target_lead_distance = vd['left_lead_distance']
        else:
            target_lead_speed = vd['right_lead_speed']
            target_lead_distance = vd['right_lead_distance']

        # 情况1: 目标车道无前车 - 高度有效
        if target_lead_distance <= 0:
            return True, "目标车道无前车", 1.0

        # 情况2: 目标车道前车速度明显更快 - 高度有效
        speed_advantage = target_lead_speed - current_lead_speed
        if speed_advantage >= self.controller.config['MIN_EFFECTIVE_SPEED_ADVANTAGE']:
            return True, f"目标车道前车快{speed_advantage:.1f}km/h", 0.9

        # 情况3: 目标车道前车距离很远 - 中等有效
        safe_distance = self.controller.calculate_dynamic_follow_distance()
        if target_lead_distance > safe_distance * 2:
            return True, f"目标车道前车距离较远({target_lead_distance}m)", 0.7

        # 情况4: 预测未来速度对比
        predicted_effectiveness = self._predict_future_effectiveness(
            current_lead_speed, target_lead_speed,
            current_lead_distance, target_lead_distance
        )

        if predicted_effectiveness >= 0.6:
            return True, f"预测超车有效({predicted_effectiveness:.2f})", predicted_effectiveness

        # 情况5: 无效超车
        return False, f"目标车道前车较慢({target_lead_speed}km/h)", 0.0

    def _predict_future_effectiveness(self, current_speed, target_speed, current_dist, target_dist):
        """预测未来超车有效性"""
        # 简化的预测模型
        ego_speed = self.controller.vehicle_data['v_ego_kph']

        # 计算相对速度
        current_relative_speed = ego_speed - current_speed
        target_relative_speed = ego_speed - target_speed

        # 如果目标车道相对速度更差，有效性降低
        if target_relative_speed < current_relative_speed:
            speed_penalty = (current_relative_speed - target_relative_speed) / 10.0
            effectiveness = max(0, 1.0 - speed_penalty)
        else:
            effectiveness = 1.0

        # 考虑距离因素
        if target_dist < current_dist:
            distance_penalty = (current_dist - target_dist) / 50.0
            effectiveness = max(0, effectiveness - distance_penalty)

        return effectiveness

    def is_ineffective_overtake_cooldown(self):
        """检查无效超车冷却期"""
        cs = self.controller.control_state
        cfg = self.controller.config

        now = time.time() * 1000
        if now - cs['last_ineffective_overtake_time'] < cfg['INEFFECTIVE_OVERTAKE_COOLDOWN']:
            remaining = (cfg['INEFFECTIVE_OVERTAKE_COOLDOWN'] - (now - cs['last_ineffective_overtake_time'])) / 1000
            return True, f"无效超车冷却中({remaining:.1f}s)"

        return False, ""

class IntelligentDecisionEngine:
    """智能决策引擎"""

    def __init__(self, controller):
        self.controller = controller
        self.effectiveness_evaluator = OvertakeEffectivenessEvaluator(controller)
        self.decision_history = []

    def select_best_lane(self):
        """选择最佳超车车道 - 考虑有效性"""
        # 检查无效超车冷却
        in_cooldown, cooldown_reason = self.effectiveness_evaluator.is_ineffective_overtake_cooldown()
        if in_cooldown:
            return None, cooldown_reason

        # 评估两侧车道
        left_evaluation = self._evaluate_lane_with_effectiveness("left")
        right_evaluation = self._evaluate_lane_with_effectiveness("right")

        # 过滤掉无效的车道
        valid_lanes = []
        if left_evaluation['safe'] and left_evaluation['effective']:
            valid_lanes.append(("left", left_evaluation))
        if right_evaluation['safe'] and right_evaluation['effective']:
            valid_lanes.append(("right", right_evaluation))

        # 选择最佳有效车道
        if valid_lanes:
            # 按有效性评分排序
            valid_lanes.sort(key=lambda x: x[1]['effectiveness_score'], reverse=True)
            best_lane, best_eval = valid_lanes[0]
            return best_lane, best_eval['reason']
        else:
            # 分析为什么没有有效车道
            reasons = []
            if not left_evaluation['effective'] and left_evaluation['safe']:
                reasons.append(f"左侧:{left_evaluation['effectiveness_reason']}")
            if not right_evaluation['effective'] and right_evaluation['safe']:
                reasons.append(f"右侧:{right_evaluation['effectiveness_reason']}")
            if not left_evaluation['safe'] and not right_evaluation['safe']:
                reasons.append("两侧都不安全")

            return None, " | ".join(reasons) if reasons else "无有效超车机会"

    def _evaluate_lane_with_effectiveness(self, side):
        """评估车道安全性和有效性"""
        evaluation = {
            'safe': False,
            'effective': False,
            'effectiveness_score': 0.0,
            'safety_reason': '',
            'effectiveness_reason': '',
            'reason': ''
        }

        # 安全性评估
        safe, safety_reason = self.controller.check_lane_safety(side)
        evaluation['safe'] = safe
        evaluation['safety_reason'] = safety_reason

        if not safe:
            evaluation['reason'] = safety_reason
            return evaluation

        # 有效性评估
        effective, effectiveness_reason, effectiveness_score = \
            self.effectiveness_evaluator.evaluate_overtake_effectiveness(side)

        evaluation['effective'] = effective
        evaluation['effectiveness_reason'] = effectiveness_reason
        evaluation['effectiveness_score'] = effectiveness_score
        evaluation['reason'] = effectiveness_reason

        return evaluation

    def record_ineffective_attempt(self):
        """记录无效超车尝试"""
        self.controller.control_state['ineffective_overtake_attempts'] += 1
        self.controller.control_state['last_ineffective_overtake_time'] = time.time() * 1000
        print(f"⚠️ 记录无效超车尝试，总次数: {self.controller.control_state['ineffective_overtake_attempts']}")

    def record_effective_attempt(self):
        """记录有效超车尝试"""
        self.controller.control_state['effective_overtake_attempts'] += 1

class MultiLayerSafetyValidator:
    """多层安全校验系统"""

    def __init__(self, controller):
        self.controller = controller
        self.safety_layers = [
            ('基础校验', self._check_basic_conditions),
            ('盲区校验', self._check_blindspots),
            ('车道校验', self._check_lane_conditions),
            ('速度校验', self._check_speed_conditions),
            ('环境校验', self._check_environmental_conditions),
            ('系统校验', self._check_system_conditions)
        ]

    def multi_layer_safety_check(self):
        """执行多层安全校验"""
        failed_checks = []

        for layer_name, check_func in self.safety_layers:
            is_safe, reason = check_func()

            if not is_safe:
                failed_checks.append(f"{layer_name}:{reason}")

                # 如果是关键层级失败，立即返回
                if layer_name in ['基础校验', '系统校验']:
                    return False, " | ".join(failed_checks)

        if failed_checks:
            return False, " | ".join(failed_checks)
        else:
            return True, "所有安全校验通过"

    def _check_basic_conditions(self):
        """基础条件校验"""
        vd = self.controller.vehicle_data

        # 车辆是否在道路上
        if not vd['IsOnroad']:
            return False, "车辆不在道路上"

        # 巡航是否激活
        if not vd['engaged']:
            return False, "巡航未激活"

        # 是否有前车
        if vd['lead_distance'] <= 0:
            return False, "前方无车辆"

        return True, "基础条件满足"

    def _check_blindspots(self):
        """盲区校验"""
        vd = self.controller.vehicle_data

        # 检查侧盲区
        if vd['left_blindspot']:
            return False, "左侧盲区有车"

        if vd['right_blindspot']:
            return False, "右侧盲区有车"

        # 检查前盲区
        if vd['l_front_blind']:
            return False, "左前盲区有车"

        if vd['r_front_blind']:
            return False, "右前盲区有车"

        return True, "盲区安全"

    def _check_lane_conditions(self):
        """车道条件校验"""
        vd = self.controller.vehicle_data
        cfg = self.controller.config

        # 检查左侧车道
        if cfg['current_lane_number'] >= cfg['lane_count']:
            return False, "已在最左侧车道"

        # 检查右侧车道 - 高速公路预留应急车道
        if cfg['road_type'] == 'highway' and cfg['current_lane_number'] <= 1:
            return False, "右侧为应急车道，禁止变道"
        elif cfg['current_lane_number'] <= 1:
            return False, "已在最右侧车道"

        # 车道宽度检查
        min_width = cfg['MIN_LANE_WIDTH']
        if vd['l_lane_width'] < min_width and vd['r_lane_width'] < min_width:
            return False, "两侧车道都过窄"

        return True, "车道条件良好"

    def _check_speed_conditions(self):
        """速度条件校验 - 移除相对速度考核"""
        vd = self.controller.vehicle_data
        cfg = self.controller.config

        current_speed = vd['v_ego_kph']

        # 基于道路类型的最低速度
        if cfg['road_type'] == 'highway':
            min_speed = cfg['HIGHWAY_MIN_SPEED']
            if current_speed < min_speed:
                return False, f"高速车速{current_speed}km/h过低"
        else:
            min_speed = cfg['NORMAL_ROAD_MIN_SPEED']
            if current_speed < min_speed:
                return False, f"普通道路车速{current_speed}km/h过低"

        return True, "速度条件合适"

    def _check_environmental_conditions(self):
        """环境条件校验"""
        vd = self.controller.vehicle_data

        # 道路曲率检查
        if abs(vd['road_curvature']) > self.controller.config['CURVATURE_THRESHOLD']:
            return False, "道路曲率过大"

        # 转向角检查
        if abs(vd['steering_angle']) > self.controller.config['STEERING_THRESHOLD']:
            return False, "转向角度过大"

        return True, "环境条件良好"

    def _check_system_conditions(self):
        """系统条件校验"""
        vd = self.controller.vehicle_data
        cs = self.controller.control_state
        cfg = self.controller.config

        # 系统自动控制检查 - system_auto_control为1时表示OP正在控制转向，暂停自动变道
        if vd.get('system_auto_control', 0) == 1:
            return False, "系统自动转向控制中"

        # 冷却时间检查
        now = time.time() * 1000
        if now - cs['lastOvertakeTime'] < cfg['OVERTAKE_COOLDOWN']:
            remaining = (cfg['OVERTAKE_COOLDOWN'] - (now - cs['lastOvertakeTime'])) / 1000
            return False, f"超车冷却中({remaining:.1f}s)"

        return True, "系统状态正常"

class EmergencyAbortSystem:
    """紧急中止系统"""

    def __init__(self, controller):
        self.controller = controller
        self.abort_conditions = [
            ('突然制动', self._check_sudden_braking, True),
            ('驾驶员干预', self._check_driver_override, True),
            ('障碍物检测', self._check_obstacle_detected, True),
            ('系统故障', self._check_system_failure, True),
            ('车道偏离', self._check_lane_departure, False),
            ('超时保护', self._check_timeout, False)
        ]
        self.last_abort_time = 0
        self.abort_cooldown = 5000  # 5秒冷却

    def monitor_abort_conditions(self):
        """监控中止条件"""
        # 冷却期检查
        now = time.time() * 1000
        if now - self.last_abort_time < self.abort_cooldown:
            return False

        critical_abort = False
        abort_reasons = []

        for condition_name, check_func, is_critical in self.abort_conditions:
            should_abort, reason = check_func()

            if should_abort:
                abort_reasons.append(f"{condition_name}:{reason}")

                if is_critical:
                    critical_abort = True
                    break  # 关键条件立即中止

        if critical_abort or (len(abort_reasons) >= 2):  # 两个非关键条件也触发中止
            abort_reason = " | ".join(abort_reasons)
            self.execute_emergency_abort(abort_reason)
            return True

        return False

    def _check_sudden_braking(self):
        """检查突然制动"""
        vd = self.controller.vehicle_data

        # 检测本车紧急制动
        if vd['break_press'] and vd['v_ego_kph'] > 60:
            return True, "本车紧急制动"

        # 检测前车紧急减速
        if (vd['lead_relative_speed'] < -20 and
            vd['lead_distance'] < 30):
            return True, "前车紧急减速"

        return False, ""

    def _check_driver_override(self):
        """检查驾驶员干预"""
        vd = self.controller.vehicle_data

        # 驾驶员主动转向
        if abs(vd['steering_angle']) > 45:  # 大角度转向
            return True, "驾驶员主动转向"

        # 驾驶员主动加速
        if vd['gas_press'] and self.controller.control_state['isOvertaking']:
            return True, "驾驶员主动加速"

        # 驾驶员取消巡航
        if not vd['engaged'] and self.controller.control_state['isOvertaking']:
            return True, "驾驶员取消巡航"

        return False, ""

    def _check_obstacle_detected(self):
        """检查障碍物检测"""
        vd = self.controller.vehicle_data

        # 近距离障碍物
        if vd['lead_distance'] > 0 and vd['lead_distance'] < 5:
            return True, "前方近距离障碍物"

        # 侧方近距离车辆
        if ((vd['left_lead_distance'] > 0 and vd['left_lead_distance'] < 3) or
            (vd['right_lead_distance'] > 0 and vd['right_lead_distance'] < 3)):
            return True, "侧方近距离车辆"

        # 前盲区紧急情况
        if (vd['l_front_blind'] and vd['left_lead_distance'] > 0 and vd['left_lead_distance'] < 10):
            return True, "左前盲区紧急情况"

        if (vd['r_front_blind'] and vd['right_lead_distance'] > 0 and vd['right_lead_distance'] < 10):
            return True, "右前盲区紧急情况"

        return False, ""

    def _check_system_failure(self):
        """检查系统故障"""
        # 传感器数据异常
        vd = self.controller.vehicle_data
        if (vd['v_ego_kph'] == 0 and vd['IsOnroad']) or \
           (vd['lead_distance'] < 0) or \
           (vd['l_lane_width'] == 0 or vd['r_lane_width'] == 0):
            return True, "传感器数据异常"

        # 通信故障
        if time.time() - self.controller.last_command_time > 30:
            return True, "通信故障"

        return False, ""

    def _check_lane_departure(self):
        """检查车道偏离"""
        vd = self.controller.vehicle_data

        # 异常车道位置
        if (vd['l_edge_dist'] < 0.3 or vd['r_edge_dist'] < 0.3):
            return True, "车道偏离警告"

        return False, ""

    def _check_timeout(self):
        """检查超时"""
        if self.controller.control_state['isOvertaking']:
            overtake_duration = time.time() * 1000 - self.controller.control_state['lastOvertakeTime']
            if overtake_duration > 30000:  # 30秒超时
                return True, "超车过程超时"

        return False, ""

    def execute_emergency_abort(self, reason):
        """执行紧急中止"""
        print(f"🚨 紧急中止: {reason}")

        # 记录中止事件
        self.last_abort_time = time.time() * 1000

        # 发送取消指令
        self.controller.cancel_overtake()

        # 更新状态
        self.controller.control_state['overtakeState'] = f"⚠️ 紧急中止"
        self.controller.control_state['overtakeReason'] = reason
        self.controller.control_state['isOvertaking'] = False

class OvertakeStateMachine:
    """自动超车状态机"""

    def __init__(self, controller):
        self.controller = controller
        self.current_state = "READY"
        self.previous_state = None
        self.state_start_time = time.time()
        self.state_history = []

        # 状态处理器映射
        self.state_handlers = {
            "READY": self._handle_ready,
            "EVALUATING": self._handle_evaluating,
            "SAFETY_CHECK": self._handle_safety_check,
            "PREPARING": self._handle_preparing,
            "EXECUTING": self._handle_executing,
            "COMPLETING": self._handle_completing,
            "RETURNING": self._handle_returning,
            "ABORTED": self._handle_aborted,
            "COMPLETED": self._handle_completed
        }

        # 状态转移表
        self.transitions = {
            "READY": ["EVALUATING", "ABORTED"],
            "EVALUATING": ["SAFETY_CHECK", "READY", "ABORTED"],
            "SAFETY_CHECK": ["PREPARING", "EVALUATING", "ABORTED"],
            "PREPARING": ["EXECUTING", "SAFETY_CHECK", "ABORTED"],
            "EXECUTING": ["COMPLETING", "ABORTED"],
            "COMPLETING": ["RETURNING", "READY", "ABORTED"],
            "RETURNING": ["COMPLETED", "ABORTED"],
            "ABORTED": ["READY"],
            "COMPLETED": ["READY"]
        }

    def transition_to(self, new_state, reason=""):
        """安全的状态转移"""
        if new_state in self.transitions.get(self.current_state, []):
            self.previous_state = self.current_state
            self.current_state = new_state
            self.state_start_time = time.time()

            # 记录状态历史
            self.state_history.append({
                'timestamp': time.time(),
                'from': self.previous_state,
                'to': new_state,
                'reason': reason,
                'duration': time.time() - self.state_start_time
            })

            # 限制历史记录长度
            if len(self.state_history) > 100:
                self.state_history.pop(0)

            print(f"🔄 状态转移: {self.previous_state} → {new_state} | 原因: {reason}")
            return True
        else:
            print(f"❌ 非法状态转移: {self.current_state} → {new_state}")
            return False

    def _handle_ready(self):
        """就绪状态处理"""
        vd = self.controller.vehicle_data

        # 系统自动控制检查
        if vd.get('system_auto_control', 0) == 1:
            self.controller.control_state['overtakeState'] = "系统自动转向控制中"
            self.controller.control_state['overtakeReason'] = "等待系统自动转向控制结束"
            return

        self.controller.control_state['overtakeState'] = "就绪"
        self.controller.control_state['overtakeReason'] = "等待超车条件"

        # 检查是否满足开始条件
        if (self.controller.config['autoOvertakeEnabled'] and
            self.controller.check_overtake_conditions()):
            self.transition_to("EVALUATING", "检测到超车条件")

    def _handle_evaluating(self):
        """评估状态处理"""
        vd = self.controller.vehicle_data

        # 系统自动控制检查
        if vd.get('system_auto_control', 0) == 1:
            self.controller.control_state['overtakeState'] = "系统自动转向控制中"
            self.controller.control_state['overtakeReason'] = "等待系统自动转向控制结束"
            # 如果正在评估，回到就绪状态
            if self.current_state == "EVALUATING":
                self.transition_to("READY", "系统自动转向控制中")
            return

        self.controller.control_state['overtakeState'] = "评估超车机会"

        # 检查超车条件是否仍然满足
        if not self.controller.check_overtake_conditions():
            self.transition_to("READY", "超车条件不再满足")
            return

        # 评估两侧车道
        left_safe, left_reason = self.controller.check_lane_safety("left")
        right_safe, right_reason = self.controller.check_lane_safety("right")

        if left_safe or right_safe:
            self.transition_to("SAFETY_CHECK", "找到可行车道")
        else:
            reasons = []
            if not left_safe: reasons.append(f"左侧:{left_reason}")
            if not right_safe: reasons.append(f"右侧:{right_reason}")
            self.controller.control_state['overtakeReason'] = " | ".join(reasons)

    def _handle_safety_check(self):
        """安全校验状态处理"""
        vd = self.controller.vehicle_data

        # 系统自动控制检查
        if vd.get('system_auto_control', 0) == 1:
            self.controller.control_state['overtakeState'] = "系统自动转向控制中"
            self.controller.control_state['overtakeReason'] = "等待系统自动转向控制结束"
            self.transition_to("READY", "系统自动转向控制中")
            return

        self.controller.control_state['overtakeState'] = "多层安全校验"

        # 执行多层安全校验
        safety_ok, safety_details = self.controller.safety_system.multi_layer_safety_check()

        if safety_ok:
            # 智能选择最佳有效车道
            best_lane, decision_reason = self.controller.decision_engine.select_best_lane()

            if best_lane:
                self.controller.selected_lane = best_lane
                self.transition_to("PREPARING", decision_reason)
            else:
                # 检查是否是因无效而拒绝
                if "无效" in decision_reason or "较慢" in decision_reason:
                    self.controller.decision_engine.record_ineffective_attempt()
                self.controller.control_state['overtakeReason'] = decision_reason
                self.transition_to("EVALUATING", "无有效超车机会")
        else:
            self.controller.control_state['overtakeReason'] = safety_details
            self.transition_to("EVALUATING", f"安全校验失败: {safety_details}")

    def _handle_preparing(self):
        """准备状态处理"""
        vd = self.controller.vehicle_data

        # 系统自动控制检查
        if vd.get('system_auto_control', 0) == 1:
            self.controller.control_state['overtakeState'] = "系统自动转向控制中"
            self.controller.control_state['overtakeReason'] = "等待系统自动转向控制结束"
            self.transition_to("READY", "系统自动转向控制中")
            return

        direction = "LEFT" if self.controller.selected_lane == "left" else "RIGHT"
        self.controller.control_state['overtakeState'] = f"准备{direction}变道"

        # 记录发送变道指令前的成功计数
        self.controller.pre_overtake_count = self.controller.control_state['overtakeSuccessCount']

        # 重置成功接收标志
        self.controller.overtake_success_received = False

        # 执行变道
        success = self.controller.execute_overtake(self.controller.selected_lane)
        if success:
            self.transition_to("EXECUTING", "开始执行变道")
        else:
            self.transition_to("ABORTED", "变道指令发送失败")

    def _handle_executing(self):
        """执行状态处理"""
        vd = self.controller.vehicle_data

        # 系统自动控制检查
        if vd.get('system_auto_control', 0) == 1:
            self.controller.control_state['overtakeState'] = "系统自动转向控制中"
            self.controller.control_state['overtakeReason'] = "等待系统自动转向控制结束"
            self.controller.cancel_overtake()  # 取消当前超车
            self.transition_to("READY", "系统自动转向控制中")
            return

        direction = "LEFT" if self.controller.selected_lane == "left" else "RIGHT"
        self.controller.control_state['overtakeState'] = f"执行{direction}变道"

        # 同时检测超车成功
        if not hasattr(self, '_executing_success_count'):
            self._executing_success_count = self.controller.control_state['overtakeSuccessCount']

        current_count = self.controller.control_state['overtakeSuccessCount']
        if current_count > self._executing_success_count:
            print(f"🎉 在执行状态检测到超车成功！直接进入完成状态")
            self.controller.overtake_success_received = True
            self.transition_to("COMPLETING", "检测到超车成功")
            return

        # 安全监控
        if self.controller.abort_system.monitor_abort_conditions():
            self.transition_to("ABORTED", "安全监控触发中止")

    def _handle_completing(self):
        """完成状态处理 - 通过检测成功计数变化来判断超车成功"""
        # 记录进入状态时的计数
        if not hasattr(self, '_entering_success_count'):
            self._entering_success_count = self.controller.control_state['overtakeSuccessCount']

        current_count = self.controller.control_state['overtakeSuccessCount']

        # 如果计数增加了，说明超车成功
        if current_count > self._entering_success_count:
            self.controller.control_state['overtakeState'] = "超车完成"
            self.controller.overtake_success_received = True
            print(f"🎉 检测到超车成功！计数从{self._entering_success_count}增加到{current_count}")

            # 清理状态
            self._entering_success_count = None

            if self.controller.config['shouldReturnToLane']:
                self.transition_to("RETURNING", "开始返回原车道")
            else:
                self.transition_to("COMPLETED", "超车流程完成")
        else:
            # 等待OP计数增加
            wait_time = time.time() - self.state_start_time
            if wait_time > 15:  # 15秒超时
                print("⏰ 超车完成等待超时，强制进入下一步")
                self.transition_to("COMPLETED", "超车完成等待超时")

    def _handle_returning(self):
        """返回状态处理"""
        self.controller.control_state['overtakeState'] = "返回原车道"

        return_complete = self.controller.try_return_to_original_lane()

        if return_complete:
            self.transition_to("COMPLETED", "成功返回原车道")
        elif time.time() - self.state_start_time > 60:  # 60秒超时
            self.transition_to("COMPLETED", "返回超时，放弃返回")

    def _handle_aborted(self):
        """中止状态处理"""
        self.controller.cancel_overtake()
        self.controller.control_state['overtakeState'] = "超车已中止"

        # 在中止状态停留2秒后回到就绪
        if time.time() - self.state_start_time > 2:
            self.transition_to("READY", "中止恢复")

    def _handle_completed(self):
        """完成状态处理"""
        self.controller.control_state['overtakeState'] = "✓ 超车流程完成"
        self.controller.control_state['isOvertaking'] = False

        # 在完成状态停留3秒后回到就绪
        if time.time() - self.state_start_time > 3:
            self.transition_to("READY", "准备下一次超车")

    def update(self):
        """更新状态机"""
        if self.current_state in self.state_handlers:
            self.state_handlers[self.current_state]()

        # 更新状态持续时间显示
        state_duration = time.time() - self.state_start_time
        self.controller.control_state['stateDuration'] = f"{state_duration:.1f}s"

class AutoOvertakeController:
    def __init__(self):
        # 初始化车辆数据、控制状态和配置
        self.vehicle_data = self._init_vehicle_data()
        self.control_state = self._init_control_state()
        self.config = self._init_config()

        # 返回原车道相关状态 - 简化：只记录净变道次数
        self.return_state = {
            'net_lane_changes': 0,
            'return_start_time': 0,
            'is_returning': False,
            'return_timeout': 0
        }

        # 消息发布/订阅 - 移除不存在的pathPlan
        self.pm = messaging.PubMaster(['autoOvertake'])
        self.sm = messaging.SubMaster([
            'carState', 'carControl', 'radarState',
            'modelV2', 'selfdriveState', 'liveLocationKalman', 'carrotMan'
        ])
        self.params = Params()

        # UDP客户端用于发送指令
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.remote_ip = "127.0.0.1"
        self.remote_port = 4211

        # 指令索引
        self.cmd_index = 0
        self.last_command_time = 0

        # 变道检测相关
        self.last_steering_angle = 0
        self.lane_change_start_time = 0
        self.lane_change_direction = ""

        # 盲区延时相关
        self.blindspot_detected_time = 0
        self.blindspot_side = ""

        # 跟车时间相关
        self.follow_start_time = 0
        self.is_following = False

        # 新增智能系统
        self.state_machine = OvertakeStateMachine(self)
        self.safety_system = MultiLayerSafetyValidator(self)
        self.abort_system = EmergencyAbortSystem(self)
        self.decision_engine = IntelligentDecisionEngine(self)

        # 新增状态变量
        self.selected_lane = None
        self.overtake_success_received = False
        self.pre_overtake_count = 0  # 变道前的成功计数

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
            # OP会自动给这个变量赋值，我们只需要定义它
            'system_auto_control': 0
        }

    def _init_control_state(self):
        return {
            'current_status': '就绪',
            'last_command': '',
            'blinker_state': 'none',
            'cruise_active': False,
            'isOvertaking': False,
            'overtakeState': '等待超车条件',
            'overtakeReason': '分析道路情况中...',
            'overtakingCompleted': False,
            # OP会自动给这个变量赋值，我们只需要定义它
            'overtakeSuccessCount': 0,
            'lastOvertakeDirection': '',
            'lastOvertakeTime': 0,
            'lastLaneChangeCommandTime': 0,
            'lane_change_in_progress': False,
            # 新增有效性评估状态
            'last_ineffective_overtake_time': 0,
            'effective_overtake_attempts': 0,
            'ineffective_overtake_attempts': 0,
            # 新增前盲区统计
            'front_blind_detection_count': 0,
            'last_front_blind_detection_time': 0
        }

    def _init_config(self):
        return {
            'road_type': 'highway', 'lane_count': 3, 'preferred_lane': 2,
            'current_lane_number': 2, 'autoOvertakeEnabled': False,
            'shouldReturnToLane': True, 'autoLaneCountEnabled': True,
            'HIGHWAY_MIN_SPEED': 75.0, 'NORMAL_ROAD_MIN_SPEED': 40.0,
            'CRUISE_SPEED_RATIO_THRESHOLD': 0.8,  # 修正：80%巡航速度
            'SAFETY_TIME_GAP': 3.0,
            'MIN_FOLLOW_TIME': 120000,
            'OVERTAKE_COOLDOWN': 8000,
            'RETURN_DELAY': 10000,
            'RETURN_TIMEOUT': 60000,
            'MIN_LANE_WIDTH': 2.5, 'SAFE_LANE_WIDTH': 3.0,
            'SIDE_LEAD_DISTANCE_MIN': 15.0,
            'SIDE_RELATIVE_SPEED_THRESHOLD': 20, 'CURVATURE_THRESHOLD': 0.02,
            'STEERING_THRESHOLD': 20.0, 'LEAD_RELATIVE_SPEED_THRESHOLD': -5.0,
            'BLINDSPOT_DELAY': 2000,
            # 新增有效性评估配置
            'MIN_EFFECTIVE_SPEED_ADVANTAGE': 5.0,  # 最小有效速度优势 (km/h)
            'INEFFECTIVE_OVERTAKE_COOLDOWN': 30000, # 无效超车冷却时间
        }

    def load_persistent_config(self):
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
        try:
            self.params.put("AutoOvertakeConfig", json.dumps(self.config))
            print("✅ 配置已保存")
        except Exception as e:
            print(f"⚠️ 保存配置失败: {e}")

    def calculate_dynamic_follow_distance(self):
        current_speed_kph = self.vehicle_data['v_ego_kph']
        time_gap_seconds = self.config['SAFETY_TIME_GAP']

        speed_mps = current_speed_kph / 3.6
        dynamic_distance = speed_mps * time_gap_seconds
        return max(20, min(dynamic_distance, 120))

    def update_vehicle_data(self):
        try:
            isOnroad = self.params.get_bool("IsOnroad")
            self.vehicle_data['IsOnroad'] = isOnroad

            if isOnroad:
              self.sm.update(100)
            else:
              self.sm.update(0)

            if isOnroad:
                if self.sm.updated['carState']:
                    carState = self.sm['carState']

                    # 使用属性访问而不是下标访问
                    v_ego_kph = int(carState.vEgo * 3.6 + 0.5) if hasattr(carState, 'vEgo') and carState.vEgo else 0
                    v_cruise_kph = carState.vCruise if hasattr(carState, 'vCruise') else 0

                    # 安全地获取属性值
                    steering_angle = round(carState.steeringAngleDeg, 1) if hasattr(carState, 'steeringAngleDeg') and carState.steeringAngleDeg else 0.0
                    left_blinker = carState.leftBlinker if hasattr(carState, 'leftBlinker') else False
                    right_blinker = carState.rightBlinker if hasattr(carState, 'rightBlinker') else False
                    gas_press = carState.gasPressed if hasattr(carState, 'gasPressed') else False
                    break_press = carState.brakePressed if hasattr(carState, 'brakePressed') else False
                    engaged = carState.cruiseState.enabled if hasattr(carState, 'cruiseState') and hasattr(carState.cruiseState, 'enabled') else False
                    left_blindspot = bool(carState.leftBlindspot) if hasattr(carState, 'leftBlindspot') else False
                    right_blindspot = bool(carState.rightBlindspot) if hasattr(carState, 'rightBlindspot') else False
                    # 注意：这里不再手动设置 system_auto_control，OP会自动更新它

                    self.vehicle_data.update({
                        'v_ego_kph': v_ego_kph,
                        'v_cruise_kph': v_cruise_kph,
                        'cruise_speed': v_cruise_kph,
                        'steering_angle': steering_angle,
                        'blinker': self._get_blinker_state(left_blinker, right_blinker),
                        'gas_press': gas_press,
                        'break_press': break_press,
                        'engaged': engaged,
                        'left_blindspot': left_blindspot,
                        'right_blindspot': right_blindspot
                        # 注意：不再设置 system_auto_control，OP会自动更新
                    })

                    if hasattr(carState, 'aEgo') and carState.aEgo:
                        self.vehicle_data['lat_a'] = round(carState.aEgo, 1)

                if self.sm.updated['radarState']:
                    radarState = self.sm['radarState']

                    # 安全地访问雷达数据
                    if hasattr(radarState, 'leadOne') and radarState.leadOne.status:
                        leadOne = radarState.leadOne
                        self.vehicle_data.update({
                            'lead_distance': int(leadOne.dRel) if hasattr(leadOne, 'dRel') else 0,
                            'lead_speed': int(leadOne.vLead * 3.6) if hasattr(leadOne, 'vLead') else 0,
                            'lead_relative_speed': int(leadOne.vRel * 3.6) if hasattr(leadOne, 'vRel') else 0
                        })
                    else:
                        # 重置前车数据
                        self.vehicle_data.update({
                            'lead_distance': 0,
                            'lead_speed': 0,
                            'lead_relative_speed': 0
                        })

                    if hasattr(radarState, 'leadLeft') and radarState.leadLeft.status:
                        leadLeft = radarState.leadLeft
                        self.vehicle_data.update({
                            'left_lead_distance': int(leadLeft.dRel) if hasattr(leadLeft, 'dRel') else 0,
                            'left_lead_speed': int(leadLeft.vLead * 3.6) if hasattr(leadLeft, 'vLead') else 0,
                            'left_lead_relative_speed': int(leadLeft.vRel * 3.6) if hasattr(leadLeft, 'vRel') else 0
                        })
                    else:
                        # 重置左侧前车数据
                        self.vehicle_data.update({
                            'left_lead_distance': 0,
                            'left_lead_speed': 0,
                            'left_lead_relative_speed': 0
                        })

                    if hasattr(radarState, 'leadRight') and radarState.leadRight.status:
                        leadRight = radarState.leadRight
                        self.vehicle_data.update({
                            'right_lead_distance': int(leadRight.dRel) if hasattr(leadRight, 'dRel') else 0,
                            'right_lead_speed': int(leadRight.vLead * 3.6) if hasattr(leadRight, 'vLead') else 0,
                            'right_lead_relative_speed': int(leadRight.vRel * 3.6) if hasattr(leadRight, 'vRel') else 0
                        })
                    else:
                        # 重置右侧前车数据
                        self.vehicle_data.update({
                            'right_lead_distance': 0,
                            'right_lead_speed': 0,
                            'right_lead_relative_speed': 0
                        })

                self.vehicle_data['desire_speed'] = 90

            if self.sm.updated['carrotMan']:
                carrotMan = self.sm['carrotMan']
                if "none" not in carrotMan.atcType or "prepare" not in carrotMan.atcType:
                    self.vehicle_data['system_auto_control'] = 1
                else:
                  self.vehicle_data['system_auto_control'] = 0

            if self.sm.updated['modelV2']:
                modelV2 = self.sm['modelV2']
                if hasattr(modelV2, 'meta'):
                    meta = modelV2.meta

                    # 安全地访问模型数据
                    desire = meta.desire if hasattr(meta, 'desire') else 'none'
                    left_front_blind = meta.leftFrontBlind if hasattr(meta, 'leftFrontBlind') else False
                    right_front_blind = meta.rightFrontBlind if hasattr(meta, 'rightFrontBlind') else False
                    lane_width_left = round(meta.laneWidthLeft, 1) if hasattr(meta, 'laneWidthLeft') and meta.laneWidthLeft else 3.2
                    lane_width_right = round(meta.laneWidthRight, 1) if hasattr(meta, 'laneWidthRight') and meta.laneWidthRight else 3.2
                    dist_to_edge_left = round(meta.distanceToRoadEdgeLeft, 1) if hasattr(meta, 'distanceToRoadEdgeLeft') and meta.distanceToRoadEdgeLeft else 1.5
                    dist_to_edge_right = round(meta.distanceToRoadEdgeRight, 1) if hasattr(meta, 'distanceToRoadEdgeRight') and meta.distanceToRoadEdgeRight else 1.5

                    self.vehicle_data.update({
                        'blinker': desire,
                        'l_front_blind': left_front_blind,
                        'r_front_blind': right_front_blind,
                        'l_lane_width': lane_width_left,
                        'r_lane_width': lane_width_right,
                        'l_edge_dist': dist_to_edge_left,
                        'r_edge_dist': dist_to_edge_right
                    })

                    # 前盲区检测统计
                    if left_front_blind or right_front_blind:
                        self.control_state['front_blind_detection_count'] += 1
                        self.control_state['last_front_blind_detection_time'] = time.time()
                        if self.control_state['front_blind_detection_count'] % 10 == 0:
                            print(f"⚠️ 前盲区检测到车辆，总次数: {self.control_state['front_blind_detection_count']}")

            if self.sm.updated['selfdriveState']:
                selfdriveState = self.sm['selfdriveState']
                self.vehicle_data['active'] = "on" if hasattr(selfdriveState, 'active') and selfdriveState.active else "off"

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

    def send_command(self, cmd_type, arg):
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

    def update_follow_time(self):
        vd = self.vehicle_data
        cfg = self.config
        now = time.time() * 1000

        is_following_now = (vd['lead_distance'] > 0)

        if is_following_now:
            if not self.is_following:
                self.follow_start_time = now
                self.is_following = True
                print(f"🚗 开始跟车计时: 前车距离{vd['lead_distance']}m")
        else:
            if self.is_following:
                print("🚗 结束跟车计时")
            self.is_following = False
            self.follow_start_time = 0

    def check_overtake_conditions(self):
        vd = self.vehicle_data
        cs = self.control_state
        cfg = self.config
        now = time.time() * 1000

        # 优先检查：系统自动控制检查 - OP正在控制转向时暂停所有超车行为
        if vd.get('system_auto_control', 0) == 1:
            cs['overtakeReason'] = "系统自动转向控制中，暂停自动变道"
            return False

        if not vd['IsOnroad']:
            cs['overtakeReason'] = "车辆不在道路上"
            return False

        if not vd['engaged']:
            cs['overtakeReason'] = "巡航未激活"
            return False

        if vd['lead_distance'] <= 0:
            cs['overtakeReason'] = "前方无车辆"
            return False

        if now - cs['lastOvertakeTime'] < cfg['OVERTAKE_COOLDOWN']:
            remaining = (cfg['OVERTAKE_COOLDOWN'] - (now - cs['lastOvertakeTime'])) / 1000
            cs['overtakeReason'] = f"超车冷却中，请等待{remaining:.1f}秒"
            return False

        if cfg['road_type'] == 'highway' and vd['v_ego_kph'] < cfg['HIGHWAY_MIN_SPEED']:
            cs['overtakeReason'] = f"高速公路车速{vd['v_ego_kph']}km/h低于最低超车速度"
            return False

        if cfg['road_type'] == 'normal' and vd['v_ego_kph'] < cfg['NORMAL_ROAD_MIN_SPEED']:
            cs['overtakeReason'] = f"普通公路车速{vd['v_ego_kph']}km/h低于最低超车速度"
            return False

        dynamic_distance_threshold = self.calculate_dynamic_follow_distance()

        trigger_conditions = []

        # 保留：前车相对速度条件（作为触发条件之一）
        if vd['lead_relative_speed'] <= cfg['LEAD_RELATIVE_SPEED_THRESHOLD']:
            trigger_conditions.append(f"前车相对速度{vd['lead_relative_speed']}km/h")

        # 保留：前车距离条件
        if vd['lead_distance'] <= dynamic_distance_threshold:
            trigger_conditions.append(f"前车距离{vd['lead_distance']}m≤{dynamic_distance_threshold:.0f}m")

        # 修正：巡航速度比例计算
        speed_ratio = vd['v_ego_kph'] / vd['v_cruise_kph'] if vd['v_cruise_kph'] > 0 else 1.0
        if speed_ratio <= cfg['CRUISE_SPEED_RATIO_THRESHOLD']:
            trigger_conditions.append(f"巡航速度比例{speed_ratio*100:.0f}%")

        # 保留：长时间跟车条件
        if self.is_following and self.follow_start_time > 0:
            follow_duration = now - self.follow_start_time
            if follow_duration >= cfg['MIN_FOLLOW_TIME']:
                trigger_conditions.append(f"长时间跟车{follow_duration/1000:.0f}秒")

        if trigger_conditions:
            cs['overtakeReason'] = f"触发条件: {', '.join(trigger_conditions)}"
            return True
        else:
            status_info = []
            # 保留：前车相对速度状态显示（仅用于信息展示）
            if vd['lead_relative_speed'] > cfg['LEAD_RELATIVE_SPEED_THRESHOLD']:
                status_info.append(f"相对速度{vd['lead_relative_speed']}km/h")

            if vd['lead_distance'] > dynamic_distance_threshold:
                status_info.append(f"距离{vd['lead_distance']}m")
            if speed_ratio > cfg['CRUISE_SPEED_RATIO_THRESHOLD']:
                status_info.append(f"速度比例{speed_ratio*100:.0f}%")
            if self.is_following and self.follow_start_time > 0:
                follow_duration = now - self.follow_start_time
                status_info.append(f"跟车{follow_duration/1000:.0f}秒")
            elif not self.is_following:
                status_info.append("未跟车")

            cs['overtakeReason'] = f"当前状态: {', '.join(status_info)}"
            return False

    def check_lane_safety(self, side):
        vd = self.vehicle_data
        cfg = self.config
        now = time.time() * 1000

        if (side == "left" and vd['left_blindspot']) or (side == "right" and vd['right_blindspot']):
            if self.blindspot_detected_time == 0:
                self.blindspot_detected_time = now
                self.blindspot_side = side
                return False, "盲区有车，等待2秒"
            elif now - self.blindspot_detected_time < cfg['BLINDSPOT_DELAY']:
                remaining = (cfg['BLINDSPOT_DELAY'] - (now - self.blindspot_detected_time)) / 1000
                return False, f"盲区有车，等待{remaining:.1f}秒"
            else:
                self.blindspot_detected_time = 0
                self.blindspot_side = ""

        if side == "left":
            if cfg['current_lane_number'] >= cfg['lane_count']:
                return False, "已在最左侧车道"

            if cfg['road_type'] == 'highway' and cfg['current_lane_number'] == cfg['lane_count']:
                return False, "左侧为对向车道，禁止变道"

            if vd['l_lane_width'] < cfg['MIN_LANE_WIDTH']:
                return False, "车道过窄⚠️禁止变道"

            # 增强前盲区判断
            front_blind_danger = False
            front_blind_reasons = []

            if vd['left_blindspot']:
                front_blind_reasons.append("侧盲区有车")
                front_blind_danger = True

            if vd['l_front_blind']:
                front_blind_reasons.append("前盲区有车")
                front_blind_danger = True

            if front_blind_danger:
                return False, f"盲区危险: {'+'.join(front_blind_reasons)}"

            # 修改：侧车条件判断 - 添加具体警告信息
            side_warnings = []

            # 检查侧车距离
            if vd['left_lead_distance'] > 0 and vd['left_lead_distance'] < cfg['SIDE_LEAD_DISTANCE_MIN']:
                side_warnings.append("侧车太近⚠️超车危险")

            # 检查侧车相对速度 - 只检查侧车过慢的情况
            if (vd['left_lead_speed'] > 0 and
                vd['left_lead_relative_speed'] < -cfg['SIDE_RELATIVE_SPEED_THRESHOLD']):
                side_warnings.append("侧车过慢⚠️超车危险")

            if side_warnings:
                return False, " | ".join(side_warnings)

            return True, "安全"

        elif side == "right":
            # 修正：高速公路预留应急车道
            if cfg['road_type'] == 'highway' and cfg['current_lane_number'] <= 1:
                return False, "右侧为应急车道，禁止变道"
            elif cfg['current_lane_number'] <= 1:
                return False, "已在最右侧车道"

            if vd['r_lane_width'] < cfg['MIN_LANE_WIDTH']:
                return False, "车道过窄⚠️禁止变道"

            # 增强前盲区判断
            front_blind_danger = False
            front_blind_reasons = []

            if vd['right_blindspot']:
                front_blind_reasons.append("侧盲区有车")
                front_blind_danger = True

            if vd['r_front_blind']:
                front_blind_reasons.append("前盲区有车")
                front_blind_danger = True

            if front_blind_danger:
                return False, f"盲区危险: {'+'.join(front_blind_reasons)}"

            # 修改：侧车条件判断 - 添加具体警告信息
            side_warnings = []

            # 检查侧车距离
            if vd['right_lead_distance'] > 0 and vd['right_lead_distance'] < cfg['SIDE_LEAD_DISTANCE_MIN']:
                side_warnings.append("侧车太近⚠️超车危险")

            # 检查侧车相对速度 - 只检查侧车过慢的情况
            if (vd['right_lead_speed'] > 0 and
                vd['right_lead_relative_speed'] < -cfg['SIDE_RELATIVE_SPEED_THRESHOLD']):
                side_warnings.append("侧车过慢⚠️超车危险")

            if side_warnings:
                return False, " | ".join(side_warnings)

            return True, "安全"

        return False, "未知方向"

    def perform_auto_overtake(self):
        if not self.config['autoOvertakeEnabled'] or self.control_state['isOvertaking']:
            return

        if not self.check_overtake_conditions():
            return

        left_safe, left_reason = self.check_lane_safety("left")
        right_safe, right_reason = self.check_lane_safety("right")

        if left_safe:
            self.execute_overtake("LEFT")
        elif right_safe:
            self.execute_overtake("RIGHT")
        else:
            reasons = []
            if not left_safe: reasons.append(f"左侧:{left_reason}")
            if not right_safe: reasons.append(f"右侧:{right_reason}")
            self.control_state['overtakeState'] = "等待安全变道时机"
            self.control_state['overtakeReason'] = " | ".join(reasons)

    def execute_overtake(self, direction):
        if not self.control_state['isOvertaking']:
            # 简化：只记录净变道次数，不记录原始车道
            print(f"📝 开始变道，当前净变道次数: {self.return_state['net_lane_changes']}")

        # 发送变道指令
        success = self.send_command("LANECHANGE", direction)
        if success:
            self.control_state['isOvertaking'] = True
            self.control_state['lane_change_in_progress'] = True
            self.control_state['lastOvertakeTime'] = time.time() * 1000
            self.control_state['lastOvertakeDirection'] = direction
            self.control_state['lastLaneChangeCommandTime'] = time.time() * 1000

            # 记录为有效尝试
            self.decision_engine.record_effective_attempt()

            # 更新车道变化计数
            if direction == "LEFT":
                self.return_state['net_lane_changes'] += 1
            else:
                self.return_state['net_lane_changes'] -= 1

            # 重置成功接收标志
            self.overtake_success_received = False

            # 设置变道开始时间（用于超时保护）
            self.lane_change_start_time = time.time()
            self.lane_change_direction = direction

            # 重置跟车计时
            self.is_following = False
            self.follow_start_time = 0

            # 更新状态显示
            if direction == "LEFT":
                self.control_state['overtakeState'] = "← 向左有效变道超车"
                self.control_state['current_status'] = "自动左变道"
            else:
                self.control_state['overtakeState'] = "→ 向右有效变道超车"
                self.control_state['current_status'] = "自动右变道"

            print(f"📤 发送{direction}有效变道指令，净变道次数: {self.return_state['net_lane_changes']}")

    def handle_overtake_success_signal(self):
        """处理超车成功信号 - 现在主要通过计数变化检测"""
        # 这个方法可以保留作为备用的成功信号处理
        if not self.overtake_success_received:
            self.control_state['overtakingCompleted'] = True
            self.control_state['isOvertaking'] = False
            self.control_state['lane_change_in_progress'] = False
            self.overtake_success_received = True
            print("✅ 收到超车成功信号")

    def check_overtake_completion(self):
        if not self.control_state['lane_change_in_progress']:
            return

        now = time.time()

        if now - self.lane_change_start_time > 15:
            self.control_state['lane_change_in_progress'] = False
            self.control_state['isOvertaking'] = False
            self.control_state['overtakeState'] = "变道超时"
            self.control_state['overtakeReason'] = "未检测到变道动作"
            return

        # 简化的超车成功检测：假设变道指令发送后5秒内没有中止就认为成功
        if now - self.lane_change_start_time > 5 and not self.overtake_success_received:
            self.handle_overtake_success_signal()

    def try_return_to_original_lane(self):
        rs = self.return_state
        cfg = self.config

        if not rs['is_returning']:
            return False

        now = time.time() * 1000

        if now > rs['return_timeout']:
            print("⏰ 返回原车道超时，停止返回")
            rs['is_returning'] = False
            rs['net_lane_changes'] = 0
            return True  # 认为完成

        # 检查是否已回到原车道（净变道次数为0）
        if rs['net_lane_changes'] == 0:
            print("✅ 已成功返回原车道")
            rs['is_returning'] = False
            return True  # 完成

        return False

    def start_return_to_original_lane(self):
        if not self.config['shouldReturnToLane']:
            return

        rs = self.return_state
        if rs['net_lane_changes'] == 0:
            return

        print(f"🔄 开始返回原车道流程，净变道次数: {rs['net_lane_changes']}")
        rs['is_returning'] = True
        rs['return_start_time'] = time.time() * 1000
        rs['return_timeout'] = rs['return_start_time'] + self.config['RETURN_TIMEOUT']

        # 发送返回指令
        if rs['net_lane_changes'] > 0:
            self.send_command("LANECHANGE", "RIGHT")
            print("↪️ 发送右变道指令返回原车道")
        else:
            self.send_command("LANECHANGE", "LEFT")
            print("↩️ 发送左变道指令返回原车道")

    def manual_overtake(self, lane):
        # 手动变道是用户强制指令，不受系统自动控制影响
        direction = "LEFT" if lane == "left" else "RIGHT"
        # 使用原文件的确切变道指令格式
        success = self.send_command("LANECHANGE", direction)
        if success:
            self.control_state['lastOvertakeDirection'] = direction
            self.control_state['lastLaneChangeCommandTime'] = time.time() * 1000

            if lane == "left":
                self.control_state['current_status'] = "强制左变道"
                self.control_state['overtakeState'] = "← 手动左变道"
            else:
                self.control_state['current_status'] = "强制右变道"
                self.control_state['overtakeState'] = "→ 手动右变道"
            self.control_state['overtakeReason'] = "用户强制变道指令"
            print(f"🔧 手动变道指令: {direction}")

    def cancel_overtake(self):
        # 使用原文件的确切取消指令格式
        success = self.send_command("CANCEL", "")
        if success:
            self.control_state['current_status'] = "取消超车"
            self.control_state['isOvertaking'] = False
            self.control_state['lane_change_in_progress'] = False
            self.control_state['overtakingCompleted'] = False

    def change_speed(self, direction):
        if direction == "UP":
            # 使用原文件的确切速度增加指令格式
            self.send_command("CRUISE_UP", "")
        elif direction == "DOWN":
            # 使用原文件的确切速度减少指令格式
            self.send_command("CRUISE_DOWN", "")

    def run_data_loop(self):
        ratekeeper = Ratekeeper(10)

        while self.running:
            try:
                self.update_vehicle_data()
                self.update_lane_number()
                self.update_curve_detection()
                self.update_follow_time()

                # 检查超车完成状态
                self.check_overtake_completion()

                if self.config['autoOvertakeEnabled']:
                    # 系统自动控制检查：如果OP正在控制转向，暂停状态机更新
                    if self.vehicle_data.get('system_auto_control', 0) != 1:
                        # 使用状态机驱动超车流程
                        self.state_machine.update()

                        # 简化的返回逻辑 - 只在收到成功信号后开始返回
                        if (self.control_state['overtakingCompleted'] and
                            not self.return_state['is_returning'] and
                            self.config['shouldReturnToLane']):
                            self.start_return_to_original_lane()

                        if self.return_state['is_returning']:
                            # 简化返回进度检查
                            self.try_return_to_original_lane()
                    else:
                        # 系统自动控制中，确保状态显示正确
                        if self.control_state['isOvertaking']:
                            self.cancel_overtake()
                        self.control_state['overtakeState'] = "系统自动转向控制中"
                        self.control_state['overtakeReason'] = "等待系统自动转向控制结束"

                # 持续安全监控（即使系统自动控制中也要监控安全）
                self.abort_system.monitor_abort_conditions()

                ratekeeper.keep_time()
            except Exception as e:
                print(f"数据循环错误: {e}")
                time.sleep(0.1)

    def update_lane_number(self):
        vd = self.vehicle_data
        cfg = self.config

        if vd['r_edge_dist'] > 0 and vd['r_lane_width'] > 0:
            if cfg['road_type'] == 'highway':
                lane_width = 3.5
                calculated_lane = int(vd['r_edge_dist'] / lane_width) + 1
            else:
                lane_width = 3.2
                calculated_lane = int(vd['r_edge_dist'] / lane_width) + 1

            calculated_lane = max(1, min(cfg['lane_count'], calculated_lane))

            if calculated_lane != cfg['current_lane_number']:
                cfg['current_lane_number'] = calculated_lane

    def update_curve_detection(self):
        vd = self.vehicle_data
        cfg = self.config

        is_curve = (vd['max_curve'] >= 1.0 or
                   abs(vd['road_curvature']) > cfg['CURVATURE_THRESHOLD'] or
                   abs(vd['steering_angle']) > cfg['STEERING_THRESHOLD'])

        if is_curve and self.control_state['isOvertaking']:
            self.cancel_overtake()
            self.control_state['current_status'] = "弯道中取消超车"
            self.control_state['overtakeReason'] = "检测到弯道，安全第一"

    def start_web_server(self):
        from http.server import HTTPServer
        handler = self.create_web_handler()
        self.web_server = HTTPServer(('0.0.0.0', 8088), handler)
        print("🌐 Web服务器启动在端口 8088")
        self.web_server.serve_forever()

    def create_web_handler(self):
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
                if 'preferred_lane' in data:
                    controller.config['preferred_lane'] = int(data['preferred_lane'])
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
                    'safetyTimeGap': 'SAFETY_TIME_GAP',
                    'minFollowTime': 'MIN_FOLLOW_TIME',
                    'minLaneWidth': 'MIN_LANE_WIDTH',
                    'safeLaneWidth': 'SAFE_LANE_WIDTH',
                    'sideLeadDist': 'SIDE_LEAD_DISTANCE_MIN',
                    'sideRelSpeed': 'SIDE_RELATIVE_SPEED_THRESHOLD',
                    'leadRelSpeed': 'LEAD_RELATIVE_SPEED_THRESHOLD',
                    'minEffectiveSpeedAdvantage': 'MIN_EFFECTIVE_SPEED_ADVANTAGE'
                }

                for web_key, config_key in param_map.items():
                    if web_key in data:
                        if web_key == 'minFollowTime':
                            controller.config[config_key] = int(data[web_key]) * 60 * 1000
                        elif web_key == 'speedRatio':
                            controller.config[config_key] = float(data[web_key]) / 100
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
                status_data = self.get_status_data()
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(status_data, ensure_ascii=False).encode('utf-8'))

            def send_json_response(self, data):
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps(data, ensure_ascii=False).encode('utf-8'))

            def get_status_data(self):
                vd = controller.vehicle_data
                cs = controller.control_state
                cfg = controller.config
                rs = controller.return_state

                # 前盲区状态
                l_front_blind = vd.get('l_front_blind', False)
                r_front_blind = vd.get('r_front_blind', False)

                # 有效性统计
                effective_attempts = cs.get('effective_overtake_attempts', 0)
                ineffective_attempts = cs.get('ineffective_overtake_attempts', 0)
                total_attempts = effective_attempts + ineffective_attempts
                effectiveness_rate = (effective_attempts / total_attempts * 100) if total_attempts > 0 else 0

                dynamic_distance = controller.calculate_dynamic_follow_distance()

                left_lane_narrow = vd.get('l_lane_width', 3.2) < cfg.get('MIN_LANE_WIDTH', 2.5)
                right_lane_narrow = vd.get('r_lane_width', 3.2) < cfg.get('MIN_LANE_WIDTH', 2.5)

                detailed_reason_parts = []

                if cs.get('overtakeReason'):
                    detailed_reason_parts.append(cs['overtakeReason'])

                detailed_reason_parts.append(f"动态距离:{dynamic_distance:.0f}m")

                if rs['is_returning']:
                    time_remaining = (rs['return_timeout'] - (time.time() * 1000)) / 1000
                    if time_remaining > 0:
                        detailed_reason_parts.append(f"返回中({abs(rs['net_lane_changes'])}次变道,{time_remaining:.0f}s)")
                    else:
                        detailed_reason_parts.append("返回超时")
                elif rs['net_lane_changes'] != 0 and not rs['is_returning']:
                    detailed_reason_parts.append(f"待返回({abs(rs['net_lane_changes'])}次变道)")

                detailed_reason_parts.append(f"净变道次数:{rs['net_lane_changes']} 当前车道:{cfg['current_lane_number']}")

                if controller.is_following and controller.follow_start_time > 0:
                    follow_duration = (time.time() * 1000 - controller.follow_start_time) / 1000
                    if follow_duration >= cfg['MIN_FOLLOW_TIME'] / 1000:
                        detailed_reason_parts.append(f"跟车{follow_duration:.0f}s")

                now = time.time() * 1000
                if now - cs['lastOvertakeTime'] < cfg['OVERTAKE_COOLDOWN']:
                    remaining = (cfg['OVERTAKE_COOLDOWN'] - (now - cs['lastOvertakeTime'])) / 1000
                    detailed_reason_parts.append(f"冷却{remaining:.0f}s")

                # 检查侧车条件是否满足超车要求 - 更新判断逻辑
                # 左侧车：只检查过慢情况（相对速度为负且绝对值过大）
                left_speed_safe = (vd.get('left_lead_speed', 0) <= 0 or
                                  vd.get('left_lead_relative_speed', 0) >= -cfg.get('SIDE_RELATIVE_SPEED_THRESHOLD', 20))
                left_distance_safe = (vd.get('left_lead_distance', 0) <= 0 or
                                     vd.get('left_lead_distance', 0) >= cfg.get('SIDE_LEAD_DISTANCE_MIN', 15))

                # 右侧车：只检查过慢情况
                right_speed_safe = (vd.get('right_lead_speed', 0) <= 0 or
                                   vd.get('right_lead_relative_speed', 0) >= -cfg.get('SIDE_RELATIVE_SPEED_THRESHOLD', 20))
                right_distance_safe = (vd.get('right_lead_distance', 0) <= 0 or
                                      vd.get('right_lead_distance', 0) >= cfg.get('SIDE_LEAD_DISTANCE_MIN', 15))

                # 在详细原因中添加侧车警告
                if not left_speed_safe and vd.get('left_lead_speed', 0) > 0:
                    detailed_reason_parts.append("左侧车过慢⚠️")
                if not left_distance_safe and vd.get('left_lead_distance', 0) > 0:
                    detailed_reason_parts.append("左侧车太近⚠️")
                if not right_speed_safe and vd.get('right_lead_speed', 0) > 0:
                    detailed_reason_parts.append("右侧车过慢⚠️")
                if not right_distance_safe and vd.get('right_lead_distance', 0) > 0:
                    detailed_reason_parts.append("右侧车太近⚠️")

                detailed_reason = " | ".join(detailed_reason_parts)

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
                    'lfb': bool(l_front_blind),
                    'rfb': bool(r_front_blind),
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
                    'pl': cfg.get('preferred_lane', 2),
                    'cl': cfg.get('current_lane_number', 2),
                    'alc': cfg.get('autoLaneCountEnabled', True),
                    'os': cs.get('overtakeState', '等待超车条件'),
                    'or': detailed_reason,
                    'oc': cs.get('overtakeSuccessCount', 0),  # 直接从OP读取的值
                    'hms': cfg.get('HIGHWAY_MIN_SPEED', 75),
                    'nms': cfg.get('NORMAL_ROAD_MIN_SPEED', 40),
                    'sr': cfg.get('CRUISE_SPEED_RATIO_THRESHOLD', 0.8),
                    'stg': cfg.get('SAFETY_TIME_GAP', 3.0),
                    'dynamic_distance': dynamic_distance,
                    'mft': cfg.get('MIN_FOLLOW_TIME', 120000),
                    'mlw': cfg.get('MIN_LANE_WIDTH', 2.5),
                    'slw': cfg.get('SAFE_LANE_WIDTH', 3.0),
                    'sld': cfg.get('SIDE_LEAD_DISTANCE_MIN', 15),
                    'srs': cfg.get('SIDE_RELATIVE_SPEED_THRESHOLD', 20),
                    'lrs_threshold': cfg.get('LEAD_RELATIVE_SPEED_THRESHOLD', -5.0),
                    'aoe': cfg.get('autoOvertakeEnabled', True),
                    'srtl': cfg.get('shouldReturnToLane', True),
                    'left_lane_narrow': left_lane_narrow,
                    'right_lane_narrow': right_lane_narrow,
                    # 侧车安全状态
                    'left_speed_safe': left_speed_safe,
                    'left_distance_safe': left_distance_safe,
                    'right_speed_safe': right_speed_safe,
                    'right_distance_safe': right_distance_safe,
                    # 系统自动控制状态
                    'system_auto_control': vd.get('system_auto_control', 0)
                }

            def get_local_ip(self):
                try:
                    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    s.connect(("8.8.8.8", 80))
                    ip = s.getsockname()[0]
                    s.close()
                    return ip
                except:
                    return "127.0.0.1"

            def get_html_content(self):
                html_file_path = os.path.join(os.path.dirname(__file__), 'web_interface.html')
                try:
                    with open(html_file_path, 'r', encoding='utf-8') as f:
                        html_content = f.read()

                    # 动态替换配置值为当前值
                    html_content = html_content.replace(
                        '"autoOvertakeEnabled":false',
                        f'"autoOvertakeEnabled":{str(controller.config["autoOvertakeEnabled"]).lower()}'
                    )
                    html_content = html_content.replace(
                        '"shouldReturnToLane":true',
                        f'"shouldReturnToLane":{str(controller.config["shouldReturnToLane"]).lower()}'
                    )
                    html_content = html_content.replace(
                        '"road_type":"highway"',
                        f'"road_type":"{controller.config["road_type"]}"'
                    )
                    html_content = html_content.replace(
                        '"lane_count":3',
                        f'"lane_count":{controller.config["lane_count"]}'
                    )
                    html_content = html_content.replace(
                        '"preferred_lane":2',
                        f'"preferred_lane":{controller.config["preferred_lane"]}'
                    )
                    html_content = html_content.replace(
                        '"autoLaneCountEnabled":true',
                        f'"autoLaneCountEnabled":{str(controller.config["autoLaneCountEnabled"]).lower()}'
                    )

                    return html_content
                except FileNotFoundError:
                    return "<html><body><h1>错误：未找到HTML界面文件</h1></body></html>"

            def log_message(self, format, *args):
                pass

        return OvertakeHTTPHandler

    def start(self):
        print("🚗 启动现代汽车自动变道超车控制器...")
        self.data_thread = threading.Thread(target=self.run_data_loop, daemon=True)
        self.data_thread.start()
        self.start_web_server()

    def stop(self):
        self.running = False
        if self.web_server:
            self.web_server.shutdown()
        if self.udp_socket:
            self.udp_socket.close()
        print("现代汽车自动变道超车控制器已停止")

def main():
    print("="*50)
    print("现代汽车自动变道超车控制器")
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
