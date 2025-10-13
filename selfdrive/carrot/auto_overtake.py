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
        # 初始化车辆数据、控制状态和配置
        self.vehicle_data = self._init_vehicle_data()
        self.control_state = self._init_control_state()
        self.config = self._init_config()
        
        # 返回原车道相关状态
        self.return_state = {
            'original_lane': 2,
            'net_lane_changes': 0,
            'return_start_time': 0,
            'is_returning': False,
            'return_timeout': 0
        }

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
            'engaged': False, 'l_front_blind': False, 'r_front_blind': False
        }

    def _init_control_state(self):
        return {
            'current_status': '就绪', 'last_command': '', 'blinker_state': 'none',
            'cruise_active': False, 'isOvertaking': False, 'overtakeState': '等待超车条件',
            'overtakeReason': '分析道路情况中...', 'overtakingCompleted': False,
            'overtakeSuccessCount': 0, 'lastOvertakeDirection': '',
            'lastOvertakeTime': 0, 'lastLaneChangeCommandTime': 0,
            'lane_change_in_progress': False
        }

    def _init_config(self):
        return {
            'road_type': 'highway', 'lane_count': 3, 'preferred_lane': 2,
            'current_lane_number': 2, 'autoOvertakeEnabled': False,
            'shouldReturnToLane': True, 'autoLaneCountEnabled': True,
            'HIGHWAY_MIN_SPEED': 75.0, 'NORMAL_ROAD_MIN_SPEED': 40.0,
            'CRUISE_SPEED_RATIO_THRESHOLD': 0.8, 
            'SAFETY_TIME_GAP': 3.0,
            'MIN_FOLLOW_TIME': 120000,
            'OVERTAKE_COOLDOWN': 8000,
            'RETURN_DELAY': 10000,
            'RETURN_TIMEOUT': 60000,
            'MIN_LANE_WIDTH': 2.5, 'SAFE_LANE_WIDTH': 3.0, 
            'SIDE_LEAD_DISTANCE_MIN': 15.0,
            'SIDE_RELATIVE_SPEED_THRESHOLD': 20, 'CURVATURE_THRESHOLD': 0.02,
            'STEERING_THRESHOLD': 20.0, 'LEAD_RELATIVE_SPEED_THRESHOLD': -5.0,
            'BLINDSPOT_DELAY': 2000
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
                if self.sm.alive['carState']:
                    carState = self.sm['carState']

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

                    if radarState.leadLeft.status:
                        leadLeft = radarState.leadLeft
                        self.vehicle_data.update({
                            'left_lead_distance': int(leadLeft.dRel),
                            'left_lead_speed': int(leadLeft.vLead * 3.6),
                            'left_lead_relative_speed': int(leadLeft.vRel * 3.6)
                        })

                    if radarState.leadRight.status:
                        leadRight = radarState.leadRight
                        self.vehicle_data.update({
                            'right_lead_distance': int(leadRight.dRel),
                            'right_lead_speed': int(leadRight.vLead * 3.6),
                            'right_lead_relative_speed': int(leadRight.vRel * 3.6)
                        })

                self.vehicle_data['desire_speed'] = 90

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
        
        if vd['lead_relative_speed'] <= cfg['LEAD_RELATIVE_SPEED_THRESHOLD']:
            trigger_conditions.append(f"前车相对速度{vd['lead_relative_speed']}km/h")
            
        if vd['lead_distance'] <= dynamic_distance_threshold:
            trigger_conditions.append(f"前车距离{vd['lead_distance']}m≤{dynamic_distance_threshold:.0f}m")
            
        speed_ratio = vd['v_ego_kph'] / vd['v_cruise_kph'] if vd['v_cruise_kph'] > 0 else 1.0
        if speed_ratio <= cfg['CRUISE_SPEED_RATIO_THRESHOLD']:
            trigger_conditions.append(f"巡航速度比例{speed_ratio*100:.0f}%")
            
        if self.is_following and self.follow_start_time > 0:
            follow_duration = now - self.follow_start_time
            if follow_duration >= cfg['MIN_FOLLOW_TIME']:
                trigger_conditions.append(f"长时间跟车{follow_duration/1000:.0f}秒")

        if trigger_conditions:
            cs['overtakeReason'] = f"触发条件: {', '.join(trigger_conditions)}"
            return True
        else:
            status_info = []
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

            if vd['left_blindspot'] or vd['l_front_blind']:
                return False, "盲区有车"
            if vd['left_lead_distance'] > 0 and vd['left_lead_distance'] < cfg['SIDE_LEAD_DISTANCE_MIN']:
                return False, "侧前车距离过近"
            if abs(vd['left_lead_relative_speed']) > cfg['SIDE_RELATIVE_SPEED_THRESHOLD']:
                return False, "侧车相对速度过高"
            return True, "安全"

        elif side == "right":
            if cfg['current_lane_number'] <= 1:
                return False, "已在最右侧车道"
                
            if cfg['road_type'] == 'highway' and cfg['current_lane_number'] == 1:
                return False, "右侧为应急车道，禁止变道"

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
            self.return_state['original_lane'] = self.config['current_lane_number']
            self.return_state['net_lane_changes'] = 0
            print(f"📝 记录原车道: {self.return_state['original_lane']}")

        # 使用原文件的确切变道指令格式
        success = self.send_command("LANECHANGE", direction)
        if success:
            self.control_state['isOvertaking'] = True
            self.control_state['lane_change_in_progress'] = True
            self.control_state['lastOvertakeTime'] = time.time() * 1000
            self.control_state['lastOvertakeDirection'] = direction
            self.control_state['lastLaneChangeCommandTime'] = time.time() * 1000

            if direction == "LEFT":
                self.return_state['net_lane_changes'] += 1
            else:
                self.return_state['net_lane_changes'] -= 1

            self.lane_change_start_time = time.time()
            self.lane_change_direction = direction

            self.is_following = False
            self.follow_start_time = 0

            if direction == "LEFT":
                self.control_state['overtakeState'] = "← 准备向左变道超车"
                self.control_state['current_status'] = "自动左变道"
            else:
                self.control_state['overtakeState'] = "→ 准备向右变道超车"
                self.control_state['current_status'] = "自动右变道"

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

    def try_return_to_original_lane(self):
        rs = self.return_state
        cfg = self.config
        
        if not rs['is_returning']:
            return
            
        now = time.time() * 1000
        
        if now > rs['return_timeout']:
            print("⏰ 返回原车道超时，停止返回")
            rs['is_returning'] = False
            rs['original_lane'] = cfg['current_lane_number']
            rs['net_lane_changes'] = 0
            return
            
        if cfg['current_lane_number'] == rs['original_lane']:
            print("✅ 已成功返回原车道")
            rs['is_returning'] = False
            rs['net_lane_changes'] = 0
            return
            
        if rs['net_lane_changes'] > 0:
            safe, reason = self.check_lane_safety("right")
            if safe:
                success = self.send_command("LANECHANGE", "RIGHT")
                if success:
                    rs['net_lane_changes'] -= 1
                    print(f"↪️ 向右返回原车道，剩余{rs['net_lane_changes']}次变道")
            else:
                print(f"⏳ 右侧车道不安全: {reason}")
        elif rs['net_lane_changes'] < 0:
            safe, reason = self.check_lane_safety("left")
            if safe:
                success = self.send_command("LANECHANGE", "LEFT")
                if success:
                    rs['net_lane_changes'] += 1
                    print(f"↩️ 向左返回原车道，剩余{abs(rs['net_lane_changes'])}次变道")
            else:
                print(f"⏳ 左侧车道不安全: {reason}")

    def start_return_to_original_lane(self):
        if not self.config['shouldReturnToLane']:
            return
            
        rs = self.return_state
        if rs['net_lane_changes'] == 0:
            return
            
        print(f"🔄 开始返回原车道流程，需要{abs(rs['net_lane_changes'])}次变道")
        rs['is_returning'] = True
        rs['return_start_time'] = time.time() * 1000
        rs['return_timeout'] = rs['return_start_time'] + self.config['RETURN_TIMEOUT']

    def manual_overtake(self, lane):
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

                if self.config['autoOvertakeEnabled']:
                    self.perform_auto_overtake()
                    self.check_overtake_completion()
                    
                    if (self.control_state['overtakingCompleted'] and 
                        not self.return_state['is_returning'] and
                        time.time() * 1000 - self.control_state['lastOvertakeTime'] > self.config['RETURN_DELAY']):
                        self.start_return_to_original_lane()
                    
                    if self.return_state['is_returning']:
                        self.try_return_to_original_lane()

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
                if self.return_state['is_returning'] and calculated_lane == self.return_state['original_lane']:
                    self.control_state['overtakingCompleted'] = True
                    self.control_state['overtakeSuccessCount'] += 1
                    self.control_state['overtakeState'] = "✓ 超车完成并返回原车道"
                    self.control_state['overtakeReason'] = "成功完成超车并返回原车道"
                    print("🎉 超车完成并返回原车道")

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
                    'leadRelSpeed': 'LEAD_RELATIVE_SPEED_THRESHOLD'
                }

                for web_key, config_key in param_map.items():
                    if web_key in data:
                        if web_key == 'minFollowTime':
                            controller.config[config_key] = int(data[web_key]) * 60 * 1000
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
                
                detailed_reason_parts.append(f"原车道:{rs['original_lane']} 当前:{cfg['current_lane_number']}")
                
                if controller.is_following and controller.follow_start_time > 0:
                    follow_duration = (time.time() * 1000 - controller.follow_start_time) / 1000
                    if follow_duration >= cfg['MIN_FOLLOW_TIME'] / 1000:
                        detailed_reason_parts.append(f"跟车{follow_duration:.0f}s")
                
                now = time.time() * 1000
                if now - cs['lastOvertakeTime'] < cfg['OVERTAKE_COOLDOWN']:
                    remaining = (cfg['OVERTAKE_COOLDOWN'] - (now - cs['lastOvertakeTime'])) / 1000
                    detailed_reason_parts.append(f"冷却{remaining:.0f}s")
                
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
                    'oc': cs.get('overtakeSuccessCount', 0),
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
                    'right_lane_narrow': right_lane_narrow
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