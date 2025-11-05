#!/usr/bin/env python3
"""
返回策略模块
负责智能返回原车道的决策和执行
"""

import time

# 导入配置模块
try:
    from selfdrive.carrot.auto_overtake.config import Config
except ImportError:
    from config import Config

class ReturnStrategy:
    """智能返回策略"""
    
    def __init__(self, config):
        self.config = config

    def start_lane_memory(self, control_state, current_lane):
        """开始记录原车道"""
        if control_state['original_lane_number'] == 0:
            control_state['original_lane_number'] = current_lane
            control_state['target_return_lane'] = current_lane
            control_state['lane_memory_start_time'] = time.time() * 1000
            control_state['return_timeout_timer'] = time.time() * 1000
            print(f"🎯 开始原车道记忆: 车道{control_state['original_lane_number']}")

    def check_lane_memory_timeout(self, control_state):
        """检查原车道记忆超时（30秒）"""
        current_time = time.time() * 1000
        
        if (control_state['original_lane_number'] > 0 and 
            current_time - control_state['return_timeout_timer'] > control_state['max_lane_memory_time']):
            print("⏰ 返回超时(30秒)，重置状态")
            return True
        return False

    def update_target_vehicle_tracking(self, vehicle_data, control_state):
        """更新目标车辆跟踪"""
        # 如果没有正在跟踪的目标车辆，尝试识别
        if control_state['target_vehicle_tracker'] is None and control_state['net_lane_changes'] != 0:
            # 根据净变道方向确定要跟踪的目标车辆在哪一侧
            if control_state['net_lane_changes'] > 0:  # 当前在左侧，需要返回右侧
                target_side = 'right'
                target_distance = vehicle_data['right_lead_distance']
                target_speed = vehicle_data['right_lead_speed']
                target_relative_speed = vehicle_data['right_lead_relative_speed']
            else:  # 当前在右侧，需要返回左侧
                target_side = 'left'
                target_distance = vehicle_data['left_lead_distance']
                target_speed = vehicle_data['left_lead_speed']
                target_relative_speed = vehicle_data['left_lead_relative_speed']
            
            # 只有在目标侧有车辆时才建立跟踪
            if target_distance > 0 and target_distance < 80:  # 只跟踪80米内的车辆
                control_state['target_vehicle_tracker'] = {
                    'side': target_side,
                    'initial_distance': target_distance,
                    'initial_speed': target_speed,
                    'last_seen_distance': target_distance,
                    'last_seen_time': time.time() * 1000,
                    'tracking_start_time': time.time() * 1000
                }
                control_state['target_vehicle_speed'] = target_speed
                control_state['target_vehicle_distance'] = target_distance
                control_state['target_vehicle_side'] = target_side
                
                print(f"🎯 开始跟踪目标车辆: {target_side}侧, 距离{target_distance}m, 速度{target_speed}km/h")
        
        # 更新已跟踪的目标车辆
        elif control_state['target_vehicle_tracker'] is not None:
            tracker = control_state['target_vehicle_tracker']
            target_side = tracker['side']
            
            if target_side == 'right':
                current_distance = vehicle_data['right_lead_distance']
                current_speed = vehicle_data['right_lead_speed']
            else:
                current_distance = vehicle_data['left_lead_distance']
                current_speed = vehicle_data['left_lead_speed']
            
            # 检查目标车辆是否还存在
            if current_distance > 0 and current_distance < 100:  # 100米内
                tracker['last_seen_distance'] = current_distance
                tracker['last_seen_time'] = time.time() * 1000
                control_state['target_vehicle_distance'] = current_distance
                control_state['target_vehicle_speed'] = current_speed
                
                # 检查是否需要切换跟踪目标（出现更近的慢车）
                if current_distance < tracker['initial_distance'] - 10:
                    print(f"🔄 发现更近的目标车辆: {current_distance}m vs {tracker['initial_distance']}m")
                    tracker['initial_distance'] = current_distance
                    tracker['initial_speed'] = current_speed
            else:
                # 目标车辆消失，可能是已超越或超出范围
                print(f"🎯 目标车辆消失，可能已超越")
                control_state['target_vehicle_tracker'] = None

    def has_completely_overtaken_target(self, vehicle_data, control_state):
        """检查是否完全超越了目标车辆"""
        if control_state['target_vehicle_tracker'] is None:
            # 没有跟踪目标车辆，检查目标侧是否有任何车辆
            if control_state['net_lane_changes'] > 0:  # 需要返回右侧
                return vehicle_data['right_lead_distance'] <= 0 or vehicle_data['right_lead_distance'] > 50
            else:  # 需要返回左侧
                return vehicle_data['left_lead_distance'] <= 0 or vehicle_data['left_lead_distance'] > 50
        
        tracker = control_state['target_vehicle_tracker']
        target_side = tracker['side']
        
        if target_side == 'right':
            current_distance = vehicle_data['right_lead_distance']
            current_speed = vehicle_data['right_lead_speed']
        else:
            current_distance = vehicle_data['left_lead_distance']
            current_speed = vehicle_data['left_lead_speed']
        
        # 核心逻辑：判断是否完全超越
        # 条件1：目标车辆消失或距离很远，50米以上
        if current_distance <= 0 or current_distance > 50:
            return True
        
        # 条件2：目标车辆距离明显增加（我们正在超越）
        distance_increase = current_distance - tracker['last_seen_distance']
        if distance_increase > 20:  # 距离增加了20米以上
            return True
        
        # 条件3：相对速度为正且持续一段时间（我们比目标车辆快）
        current_relative_speed = vehicle_data['v_ego_kph'] - current_speed
        if current_relative_speed > 10:  # 比目标车辆快10km/h以上
            time_since_tracking = time.time() * 1000 - tracker['tracking_start_time']
            if time_since_tracking > 8000:  # 跟踪超过8秒且一直保持速度优势
                return True
        
        return False

    def is_return_efficient(self, vehicle_data, return_direction):
        """检查返回是否有效率优势"""
        current_speed = vehicle_data['v_ego_kph']
        
        # 获取目标车道（返回方向）的速度预期
        if return_direction == "RIGHT":
            target_lead_speed = vehicle_data['right_lead_speed']
            target_lead_distance = vehicle_data['right_lead_distance']
            target_relative_speed = vehicle_data['right_lead_relative_speed']
        else:
            target_lead_speed = vehicle_data['left_lead_speed']
            target_lead_distance = vehicle_data['left_lead_distance']
            target_relative_speed = vehicle_data['left_lead_relative_speed']
        
        # 计算目标车道的预期速度
        if target_lead_distance <= 0:
            # 优化：目标车道无车，预期速度为巡航速度或当前速度+10
            expected_target_speed = vehicle_data['v_cruise_kph'] if vehicle_data['v_cruise_kph'] > 0 else current_speed + 10
        else:
            # 目标车道有车，预期速度受前车限制
            if target_relative_speed > 5:  # 目标车道前车比我们快
                expected_target_speed = min(target_lead_speed, vehicle_data['v_cruise_kph'])
            else:  # 目标车道前车比我们慢或相当
                expected_target_speed = target_lead_speed
        
        # 计算当前车道的预期速度
        if vehicle_data['lead_distance'] <= 0:
            expected_current_speed = vehicle_data['v_cruise_kph'] if vehicle_data['v_cruise_kph'] > 0 else current_speed
        else:
            if vehicle_data['lead_relative_speed'] > 5:  # 当前前车比我们快
                expected_current_speed = min(vehicle_data['lead_speed'], vehicle_data['v_cruise_kph'])
            else:  # 当前前车比我们慢
                expected_current_speed = vehicle_data['lead_speed']
        
        # 效率判断：只有目标车道明显快于当前车道才返回
        speed_advantage = expected_target_speed - expected_current_speed
        min_advantage = 8  # 至少需要8km/h的速度优势
        
        is_efficient = speed_advantage >= min_advantage
        
        print(f"🔄 返回效率分析: 目标车道{expected_target_speed}km/h vs 当前{expected_current_speed}km/h, 优势{speed_advantage:.1f}km/h, 效率{'✅' if is_efficient else '❌'}")
        
        return is_efficient, speed_advantage

    def is_return_safe(self, vehicle_data, check_side):
        """检查返回原车道是否安全 - 只关注目标车道情况"""
        current_speed = vehicle_data['v_ego_kph']
        
        if check_side == "right":
            target_distance = vehicle_data['right_lead_distance']
            target_relative_speed = vehicle_data['right_lead_relative_speed']
            blindspot = vehicle_data['right_blindspot'] or vehicle_data['r_front_blind']
        else:
            target_distance = vehicle_data['left_lead_distance']
            target_relative_speed = vehicle_data['left_lead_relative_speed']
            blindspot = vehicle_data['left_blindspot'] or vehicle_data['l_front_blind']
        
        # 🎯 安全条件1：盲区检查
        if blindspot:
            print(f"❌ {check_side}侧盲区有车，返回不安全")
            return False, "盲区有车"
        
        # 🎯 安全条件2：目标车道车辆情况
        if target_distance <= 0:
            # 目标车道无车，安全返回
            print(f"✅ {check_side}侧无车辆，安全返回")
            return True, "车道畅通"
        
        # 🎯 安全条件3：目标车道有车，判断是否安全
        # 情况1：目标车道车辆比我们快+5km/h以上，安全返回
        if target_relative_speed > 5:
            safe_distance = max(30, current_speed * 0.4)
            if target_distance > safe_distance:
                print(f"✅ {check_side}侧车辆较快(+{target_relative_speed}km/h)，距离安全{target_distance}m")
                return True, "前车较快且距离安全"
            else:
                print(f"⚠️ {check_side}侧车辆较快但距离较近{target_distance}m")
                return False, "前车较快但距离过近"
        
        # 情况2：目标车道车辆距离超过50米，安全返回
        elif target_distance > 50:
            print(f"✅ {check_side}侧车辆距离较远{target_distance}m，安全返回")
            return True, "前车距离安全"
        
        # 情况3：目标车道车辆比我们慢，不应该返回（继续超车）
        else:
            print(f"❌ {check_side}侧车辆较慢({target_relative_speed}km/h)且距离近{target_distance}m，不应返回")
            return False, "前车较慢，继续超车"

    def is_return_direction_available(self, current_lane, total_lanes, return_direction):
        """检查返回方向是否可用"""
        if return_direction == "RIGHT":
            return current_lane < total_lanes
        else:
            return current_lane > 1

    def check_return_stability(self, vehicle_data):
        """检查返回前的稳定性"""
        # 检查速度稳定性
        if vehicle_data['v_ego_kph'] < 60:
            return True

        # 检查方向盘角度
        if abs(vehicle_data['steering_angle']) > 10:
            print(f"⚠️ 方向盘角度过大({vehicle_data['steering_angle']}°)，等待稳定")
            return False

        # 检查横向加速度
        if abs(vehicle_data['lat_a']) > 0.5:
            print(f"⚠️ 横向加速度过大({vehicle_data['lat_a']}m/s²)，等待稳定")
            return False

        return True

    def check_smart_return_conditions(self, vehicle_data, control_state, config):
        """检查智能返回条件 - 优化版本"""
        # 🎯 基础条件检查
        if not config['shouldReturnToLane']:
            return False

        road_type = config['road_type']
        return_strategy = config['RETURN_STRATEGY'][road_type]

        if not return_strategy['enabled']:
            if control_state['net_lane_changes'] != 0:
                print("🛣️ 普通道路：禁用返回功能，重置净变道次数")
                return False
            return False

        # 🆕 检查30秒返回超时
        if self.check_lane_memory_timeout(control_state):
            control_state['overtakeState'] = "返回超时"
            control_state['overtakeReason'] = "30秒内未完成返回，重置状态"
            return False

        # 🆕 检查原车道记忆是否存在
        if control_state['original_lane_number'] == 0:
            # 如果没有原车道记忆但净变道数不为0，尝试重建
            if control_state['net_lane_changes'] != 0:
                self.start_lane_memory(control_state, config['current_lane_number'])
            else:
                return False

        # 🆕 新增：检查最低车速（与超车相同标准）
        if road_type == 'highway' and vehicle_data['v_ego_kph'] < config['HIGHWAY_MIN_SPEED']:
            control_state['overtakeState'] = "车速过低"
            control_state['overtakeReason'] = f"返回原车道：高速公路车速{vehicle_data['v_ego_kph']}km/h低于最低速度{config['HIGHWAY_MIN_SPEED']}km/h"
            return False

        if road_type == 'normal' and vehicle_data['v_ego_kph'] < config['NORMAL_ROAD_MIN_SPEED']:
            control_state['overtakeState'] = "车速过低"
            control_state['overtakeReason'] = f"返回原车道：普通道路车速{vehicle_data['v_ego_kph']}km/h低于最低速度{config['NORMAL_ROAD_MIN_SPEED']}km/h"
            return False

        # 🆕 新增：检查其他超车条件（除了前车相关条件）
        if not vehicle_data['IsOnroad']:
            control_state['overtakeReason'] = "车辆不在道路上"
            return False

        if not vehicle_data['engaged']:
            control_state['overtakeReason'] = "巡航未激活"
            return False

        if vehicle_data['system_auto_control'] == 1:
            control_state['overtakeReason'] = "OP自动控制中，暂停返回"
            return False

        if control_state['net_lane_changes'] == 0:
            return False

        if not control_state['is_auto_overtake']:
            return False

        if control_state['return_attempts'] >= control_state['max_return_attempts']:
            print(f"⚠️ 达到最大返回尝试次数({control_state['max_return_attempts']})，放弃返回")
            return False

        if control_state['isOvertaking']:
            return False

        # 🎯 确定返回方向
        if control_state['net_lane_changes'] > 0:
            return_direction = "RIGHT"
            check_side = "right"
        else:
            return_direction = "LEFT"
            check_side = "left"

        if not self.is_return_direction_available(config['current_lane_number'], config['lane_count'], return_direction):
            print(f"❌ 返回方向{return_direction}不可用")
            return False

        # 优化部分1：更新目标车辆跟踪
        self.update_target_vehicle_tracking(vehicle_data, control_state)

        # 优化部分2：检查是否完全超越了目标车辆
        if not self.has_completely_overtaken_target(vehicle_data, control_state):
            control_state['overtakeState'] = "正在超越前车"
            
            # 提供详细的超越状态信息
            if control_state['target_vehicle_tracker'] is not None:
                target_distance = control_state['target_vehicle_distance']
                target_speed = control_state['target_vehicle_speed']
                current_relative_speed = vehicle_data['v_ego_kph'] - target_speed
                
                control_state['overtakeReason'] = f"正在超越目标车辆(距离:{target_distance}m, 速度:{target_speed}km/h, 相对:{current_relative_speed}km/h)"
            else:
                control_state['overtakeReason'] = "正在识别目标车辆"
            return False

        # 优化部分3：超越完成后计时
        current_time = time.time() * 1000
        if control_state['overtake_complete_timer'] == 0:
            control_state['overtake_complete_timer'] = current_time
            control_state['overtakeState'] = "已超越前车，等待返回时机"
            control_state['overtakeReason'] = f"等待{control_state['overtake_complete_duration']/1000}秒确认安全返回"
            print(f"⏰ 开始返回计时: {control_state['overtake_complete_duration']/1000}秒")
            return False

        # 检查计时是否完成
        if current_time - control_state['overtake_complete_timer'] < control_state['overtake_complete_duration']:
            remaining = (control_state['overtake_complete_duration'] - (current_time - control_state['overtake_complete_timer'])) / 1000
            control_state['overtakeReason'] = f"确认安全返回，等待{remaining:.1f}秒"
            return False

        # 优化部分4：检查返回效率
        is_efficient, speed_advantage = self.is_return_efficient(vehicle_data, return_direction)
        if not is_efficient:
            control_state['overtakeState'] = "返回效率不足"
            control_state['overtakeReason'] = f"返回车道速度优势不足: +{speed_advantage:.1f}km/h (需要至少+8km/h)"
            # 重置计时器，继续观察
            control_state['overtake_complete_timer'] = current_time
            return False

        # 优化部分5：检查返回安全性
        is_safe, safety_reason = self.is_return_safe(vehicle_data, check_side)
        if not is_safe:
            control_state['overtakeState'] = f"返回{return_direction}不安全"
            control_state['overtakeReason'] = f"安全条件: {safety_reason}"
            # 重置计时器，继续观察
            control_state['overtake_complete_timer'] = current_time
            return False

        # 保持原有的稳定性检查
        if not self.check_return_stability(vehicle_data):
            control_state['overtakeState'] = "稳定行驶中"
            control_state['overtakeReason'] = "等待行驶稳定后再返回"
            # 重置计时器，继续观察
            control_state['overtake_complete_timer'] = current_time
            return False

        # 所有条件满足，可以返回
        control_state['return_conditions_met'] = True
        return True