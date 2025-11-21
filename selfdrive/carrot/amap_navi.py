import json
import time
import threading
import socket
import fcntl
import struct
import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper
from openpilot.common.params import Params
from openpilot.system.hardware import PC

BLINKER_NONE = 0
BLINKER_LEFT = 1
BLINKER_RIGHT = 2
BLINKER_BOTH = 3

class SharedData:
  def __init__(self):
    #=============共享数据（来自amap_navi）=============
    #盲区信号
    self.left_blind = False #摄像头盲区信号
    self.right_blind = False
    self.lidar_lblind = False #雷达盲区信号
    self.lidar_rblind = False
    self.lf_drel = None #雷达左前车距离
    self.lb_drel = None #雷达左后车距离
    self.rf_drel = None #雷达右前车距离
    self.rb_drel = None #雷达右后车距离
    self.lf_xrel = None #雷达左前车距离
    self.lb_xrel = None #雷达左后车距离
    self.rf_xrel = None #雷达右前车距离
    self.rb_xrel = None #雷达右后车距离
    self.lidar_l = False
    self.lidar_r = False
    self.camera_l = False
    self.camera_r = False

    #客户端控制命令
    self.cmd_index = -1
    self.remote_cmd = ""
    self.remote_arg = ""

    self.ext_blinker = BLINKER_NONE # 外挂控制器转向灯状态
    self.ext_state = 0  # 外挂控制器的数量

    #=============共享数据（desire_helper）=============
    self.leftFrontBlind = None
    self.rightFrontBlind = None

    # =============共享数据（carrotMan）=============
    self.roadcate = None
    self.lat_a = None
    self.max_curve = None

    self.showDebugLog = 0

def f1(x):
  return round(float(x), 1)

class AmapNaviServ:
  def __init__(self):
    self.shared_data = SharedData() #new
    self.params = Params()
    self.sm = messaging.SubMaster(['carState', 'modelV2', 'selfdriveState', 'radarState', 'carrotMan'])

    self.broadcast_ip = self.navi_get_broadcast_address() #广播地址
    self.broadcast_port = 4210 #广播端口
    self.listen_port = 4211 #监听地址
    self.local_ip_address = "0.0.0.0" #本地ip地址

    self.clients = {}  # 保存多个客户端

    threading.Thread(target=self.navi_broadcast_info).start()
    threading.Thread(target=self.navi_comm_thread).start()

  def left_blindspot(self):
    return self.shared_data.left_blind or self.shared_data.lidar_lblind
  def right_blindspot(self):
    return self.shared_data.right_blind or self.shared_data.lidar_rblind

  def _capnp_list_to_list(self, capnp_list, max_items=None):
    """将capnp列表转换为Python列表"""
    if capnp_list is None:
      return []
    try:
      result = [float(x) for x in capnp_list]
      if max_items is not None:
          return result[:max_items]
      return result
    except (TypeError, AttributeError):
      return []

  def navi_comm_thread(self):
    blinker_alive = False
    blinker_time = time.time()
    l_blindspot_alive = False
    l_blindspot_time = time.time()
    r_blindspot_alive = False
    r_blindspot_time = time.time()
    lidar_lblind_alive = False
    lidar_lblind_time = time.time()
    lidar_rblind_alive = False
    lidar_rblind_time = time.time()
    while True:
      try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
          sock.settimeout(10)  # 超时10秒
          sock.bind(('0.0.0.0', self.listen_port))
          print("#########navi_comm_thread: UDP thread started...")

          while True:
            try:
              try:
                data, remote_addr = sock.recvfrom(4096)

                if not data:
                  raise ConnectionError("No data received")

                ip, port = remote_addr
                # 修改: 保存多个客户端地址及最后活跃时间
                if not hasattr(self, "clients"):
                  self.clients = {}  # {ip: last_seen_time}

                try:
                  json_obj = json.loads(data.decode())

                  #转向灯模块
                  if "blinker" in json_obj:
                    self.shared_data.ext_blinker = json_obj.get("blinker")
                    if self.shared_data.ext_blinker in ["left", "stockleft"]:
                      self.shared_data.ext_blinker = BLINKER_LEFT
                    elif self.shared_data.ext_blinker in ["right", "stockright"]:
                      self.shared_data.ext_blinker = BLINKER_RIGHT
                    else:
                      self.shared_data.ext_blinker = BLINKER_NONE
                    blinker_alive = True
                    blinker_time = time.time()

                  #客户端命令
                  if "index" in json_obj:
                    self.shared_data.cmd_index = int(json_obj.get("index"))
                  if "cmd" in json_obj:
                    self.shared_data.remote_cmd = json_obj.get("cmd")
                    self.shared_data.remote_arg = json_obj.get("arg")
                    print(f"Command: index={self.shared_data.cmd_index}, cmd={self.shared_data.remote_cmd},arg={self.shared_data.remote_arg}")

                  #响应类型
                  if "resp" in json_obj:
                    resp = json_obj.get("resp")
                    #摄像头盲区信号
                    if resp == "cam_blind":
                      if "left_blind" in json_obj:
                        self.shared_data.left_blind = json_obj.get("left_blind")
                        l_blindspot_alive = True
                        l_blindspot_time = time.time()
                      if "right_blind" in json_obj:
                        self.shared_data.right_blind = json_obj.get("right_blind")
                        r_blindspot_alive = True
                        r_blindspot_time = time.time()

                    #雷达盲区信号和距离
                    if resp == "blindspot":
                      #盲区
                      if "lidar_lblind" in json_obj:
                        self.shared_data.lidar_lblind = json_obj.get("lidar_lblind")
                        lidar_lblind_alive = True
                        lidar_lblind_time = time.time()
                      if "lidar_rblind" in json_obj:
                        self.shared_data.lidar_rblind = json_obj.get("lidar_rblind")
                        lidar_rblind_alive = True
                        lidar_rblind_time = time.time()
                      #距离
                      if "lf_drel" in json_obj:
                        self.shared_data.lf_drel = int(json_obj.get("lf_drel"))
                      else:
                        self.shared_data.lf_drel = None
                      if "lb_drel" in json_obj:
                        self.shared_data.lb_drel = int(json_obj.get("lb_drel"))
                      else:
                        self.shared_data.lb_drel = None
                      if "rf_drel" in json_obj:
                        self.shared_data.rf_drel = int(json_obj.get("rf_drel"))
                      else:
                        self.shared_data.rf_drel = None
                      if "rb_drel" in json_obj:
                        self.shared_data.rb_drel = int(json_obj.get("rb_drel"))
                      else:
                        self.shared_data.rb_drel = None

                      if "lf_xrel" in json_obj:
                        self.shared_data.lf_xrel = int(json_obj.get("lf_xrel"))
                      else:
                        self.shared_data.lf_xrel = None
                      if "lb_xrel" in json_obj:
                        self.shared_data.lb_xrel = int(json_obj.get("lb_xrel"))
                      else:
                        self.shared_data.lb_xrel = None
                      if "rf_xrel" in json_obj:
                        self.shared_data.rf_xrel = int(json_obj.get("rf_xrel"))
                      else:
                        self.shared_data.rf_xrel = None
                      if "rb_xrel" in json_obj:
                        self.shared_data.rb_xrel = int(json_obj.get("rb_xrel"))
                      else:
                        self.shared_data.rb_xrel = None

                  #更新客户端信息
                  old_info = self.clients.get(ip, {})
                  self.clients[ip] = {
                    "last_seen": time.time(),
                    "device": json_obj.get("device", old_info.get("device", "")),
                    "detect_side":int(json_obj.get("detect_side", old_info.get("detect_side", 0))),
                  }

                  if (self.shared_data.showDebugLog & 32) > 0:
                    print(f"receive: {json_obj}")
                except Exception as e:
                  self.shared_data.ext_blinker = BLINKER_NONE
                  if (self.shared_data.showDebugLog & 32) > 0:
                    print(f"navi_comm_thread: json error...: {e}")
                    print(data)
              except TimeoutError:
                if (self.shared_data.showDebugLog & 32) > 0:
                  print("Waiting for data (timeout)...")
              except Exception as e:
                if (self.shared_data.showDebugLog & 32) > 0:
                  print(f"navi_comm_thread: error...: {e}")
                break

              # 修改: 清理超过 10 秒未活跃的客户端
              now = time.time()
              self.clients = {ip: info for ip, info in self.clients.items() if now - info["last_seen"] < 10}

              #超过10秒后重启转向灯和盲区状态
              if blinker_alive and (now - blinker_time) > 10:
                self.shared_data.ext_blinker = BLINKER_NONE
                blinker_alive = False
              if l_blindspot_alive and (now - l_blindspot_time) > 10:
                self.shared_data.left_blind = False
                l_blindspot_alive = False
              if r_blindspot_alive and (now - r_blindspot_time) > 10:
                self.shared_data.right_blind = False
                r_blindspot_alive = False
              if lidar_lblind_alive and (now - lidar_lblind_time) > 10:
                self.shared_data.lidar_lblind = False
                lidar_lblind_alive = False
              if lidar_rblind_alive and (now - lidar_rblind_time) > 10:
                self.shared_data.lidar_rblind = False
                lidar_rblind_alive = False

              if self.clients:
                self.shared_data.ext_state = len(self.clients)
              else:
                self.shared_data.ext_state = 0
                self.shared_data.ext_blinker = BLINKER_NONE

              self.shared_data.ext_blinker = self.shared_data.ext_blinker

            except Exception as e:
              print(f"navi_comm_thread: recv error...: {e}")
              break

          time.sleep(1)
      except Exception as e:
        print(f"Network error, retrying...: {e}")
        time.sleep(2)

  def navi_broadcast_info(self):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    frame = 0
    rk = Ratekeeper(20, print_delay_threshold=None)
    broadcast_cnt = 0

    while True:
      try:
        self.sm.update(0)
        # 修改: 获取当前活跃客户端
        active_clients = list(getattr(self, "clients", {}).keys())

        if frame % 20 == 0 or active_clients:
          try:
            if not PC:
              ip_address = socket.gethostbyname(socket.gethostname())
            else:
              ip_address = self.navi_get_local_ip()
            if ip_address != self.local_ip_address:
              self.local_ip_address = ip_address
              self.clients = {}  # 修改: 本地 IP 变化时清空客户端

            #消息
            navi_msg = None
            navi_dat = None
            lidar_msg = None
            lidar_dat = None

            blinker_msg = self.make_blinker_message()
            blinker_dat = blinker_msg.encode('utf-8')

            broadcast_msg = self.make_broadcast_message()
            broadcast_dat = broadcast_msg.encode('utf-8')

            lidar_l = False
            lidar_r = False
            camera_l = False
            camera_r = False
            if active_clients:
              # 向所有客户端发送
              if self.clients:
                for ip, info in self.clients.items():
                  try:
                    device_type = info.get("device", None)
                    detect_side = info.get("detect_side", None)
                    # 根据 device_type 做判断
                    if device_type == "overtake" or device_type == "navi": #超车或导航
                      if navi_msg is None:
                        navi_msg = self.make_navi_message()
                        navi_dat = navi_msg.encode('utf-8')
                      if navi_dat is not None:
                        sock.sendto(navi_dat, (ip, self.broadcast_port))
                        if (self.shared_data.showDebugLog & 32) > 0:
                          print(f"sendto {ip} (overtake): {navi_dat}")
                    elif device_type == "lidar" or device_type == "camera": #雷达模块
                      if device_type == "lidar":
                        if (detect_side & 1) > 0:
                          lidar_l = True
                        if (detect_side & 2) > 0:
                          lidar_r = True
                      if device_type == "camera":
                        if (detect_side & 1) > 0:
                          camera_l = True
                        if (detect_side & 2) > 0:
                          camera_r = True

                      if lidar_msg is None:
                        lidar_msg = self.make_lidar_message()
                        lidar_dat = lidar_msg.encode('utf-8')
                      if lidar_dat is not None:
                        sock.sendto(lidar_dat, (ip, self.broadcast_port))
                        if (self.shared_data.showDebugLog & 32) > 0:
                          print(f"sendto {ip} (lidar): {lidar_dat}")
                    else: #其他
                      sock.sendto(blinker_dat, (ip, self.broadcast_port))
                      if (self.shared_data.showDebugLog & 32) > 0:
                        print(f"sendto {ip} (blinker): {blinker_dat}")
                  except Exception as e:
                    if (self.shared_data.showDebugLog & 32) > 0:
                      print(f"sendto {ip} failed: {e}")

            self.shared_data.lidar_l = lidar_l
            self.shared_data.lidar_r = lidar_r
            self.shared_data.camera_l = camera_l
            self.shared_data.camera_r = camera_r

            #每2秒广播一次自己的ip和端口
            if frame % 20 == 0:
              if self.broadcast_ip is not None and broadcast_dat is not None:
                self.broadcast_ip = self.navi_get_broadcast_address()
                sock.sendto(broadcast_dat, (self.broadcast_ip, self.broadcast_port))
                broadcast_cnt += 1
                if (self.shared_data.showDebugLog & 32) > 0:
                  print(f"broadcasting: {self.broadcast_ip}:{self.broadcast_port},{broadcast_msg}")

          except Exception as e:
            if (self.shared_data.showDebugLog & 32) > 0:
              print(f"##### navi_broadcast_error...: {e}")
            #traceback.print_exc()

        rk.keep_time()
        frame += 1
      except Exception as e:
        if (self.shared_data.showDebugLog & 32) > 0:
          print(f"navi_broadcast_info error...: {e}")
        #traceback.print_exc()
        time.sleep(1)

  def make_navi_message(self):
    msg = {}
    msg['ip'] = self.local_ip_address
    msg['port'] = self.listen_port
    isOnroad = self.params.get_bool("IsOnroad")
    msg['IsOnroad'] = isOnroad

    if isOnroad:
      #车辆状态
      if self.sm.alive['carState']:# and self.sm.updated['carState']:
        #print("carState alive")
        carState = self.sm['carState']
        v_ego_kph = int(carState.vEgoCluster * 3.6 + 0.5)
        v_cruise_kph = carState.vCruise
        #msg["cruise_speed"] = v_cruise_kph  # 巡航速度
        msg['v_cruise_kph'] = v_cruise_kph  # 巡航速度
        msg['v_ego_kph'] = v_ego_kph  # 当前速度
        #new
        if hasattr(carState, 'vEgo'):
          msg["vego"] = int(carState.vEgo * 3.6)
        if hasattr(carState, 'aEgo'):
          msg["aego"] = round(carState.aEgo,1)
        if hasattr(carState, 'steeringAngleDeg'):
          msg["steer_angle"] = round(carState.steeringAngleDeg,1)
        if hasattr(carState, 'gasPressed'):
          msg["gas_press"] = carState.gasPressed
        if hasattr(carState, 'brakePressed'):
          msg["break_press"] = carState.brakePressed
        if hasattr(carState, 'cruiseState'):
          msg["engaged"] = carState.cruiseState.enabled
        # 盲区检测
        if hasattr(carState, 'leftBlindspot'):
          msg["left_blindspot"] = int(carState.leftBlindspot)
        if hasattr(carState, 'rightBlindspot'):
          msg["right_blindspot"] = int(carState.rightBlindspot)

      # 雷达数据
      if self.sm.alive['radarState']:# and self.sm.updated['radarState']:
        radar_state = self.sm['radarState']
        # 当前车道前车
        if hasattr(radar_state, 'leadOne') and radar_state.leadOne and hasattr(radar_state.leadOne,'status') and radar_state.leadOne.status:
          msg["lead1"] = True
          if hasattr(radar_state.leadOne, 'dRel'):
            msg["drel"] = int(radar_state.leadOne.dRel)
          if hasattr(radar_state.leadOne, 'vLead'):
            msg["vlead"] = int(radar_state.leadOne.vLead * 3.6)
          if hasattr(radar_state.leadOne, 'vRel'):
            msg["vrel"] = int(radar_state.leadOne.vRel * 3.6)
          if hasattr(radar_state.leadOne, 'aRel'):
            msg["lead_accel"] = radar_state.leadOne.aRel
        else:
          msg["lead1"] = False
        # 左侧前车
        if hasattr(radar_state, 'leadLeft') and radar_state.leadLeft and hasattr(radar_state.leadLeft,'status') and radar_state.leadLeft.status:
          msg["l_lead"] = True
          if hasattr(radar_state.leadLeft, 'dRel'):
            msg["l_drel"] = int(radar_state.leadLeft.dRel)
          if hasattr(radar_state.leadLeft, 'vLead'):
            msg["l_vlead"] = int(radar_state.leadLeft.vLead * 3.6)
          if hasattr(radar_state.leadLeft, 'vRel'):
            msg["l_vrel"] = int(radar_state.leadLeft.vRel * 3.6)
        else:
          msg["l_lead"] = False
        # 右侧前车
        if hasattr(radar_state, 'leadRight') and radar_state.leadRight and hasattr(radar_state.leadRight,'status') and radar_state.leadRight.status:
          msg["r_lead"] = True
          if hasattr(radar_state.leadRight, 'dRel'):
            msg["r_drel"] = int(radar_state.leadRight.dRel)
          if hasattr(radar_state.leadRight, 'vLead'):
            msg["r_vlead"] = int(radar_state.leadRight.vLead * 3.6)
          if hasattr(radar_state.leadRight, 'vRel'):
            msg["r_vrel"] = int(radar_state.leadRight.vRel * 3.6)
        else:
          msg["r_lead"] = False

      #前雷达盲区信号
      if self.shared_data.leftFrontBlind is not None:
        msg['l_front_blind'] = self.shared_data.leftFrontBlind
      if self.shared_data.rightFrontBlind is not None:
        msg['r_front_blind'] = self.shared_data.rightFrontBlind

      #雷达和摄像头盲区
      msg['lidar_lblind'] = self.left_blindspot()
      msg['lidar_rblind'] = self.right_blindspot()

      #雷达距离数据
      if self.shared_data.lf_drel is not None:
        msg['lf_drel'] = self.shared_data.lf_drel
      if self.shared_data.lb_drel is not None:
        msg['lb_drel'] = self.shared_data.lb_drel
      if self.shared_data.rf_drel is not None:
        msg['rf_drel'] = self.shared_data.rf_drel
      if self.shared_data.rb_drel is not None:
        msg['rb_drel'] = self.shared_data.rb_drel

      if self.shared_data.lf_xrel is not None:
        msg['lf_xrel'] = self.shared_data.lf_xrel
      if self.shared_data.lb_xrel is not None:
        msg['lb_xrel'] = self.shared_data.lb_xrel
      if self.shared_data.rf_xrel is not None:
        msg['rf_xrel'] = self.shared_data.rf_xrel
      if self.shared_data.rb_xrel is not None:
        msg['rb_xrel'] = self.shared_data.rb_xrel

      #雷达或摄像头是否存在标志
      msg['lidar_l'] = self.shared_data.lidar_l
      msg['lidar_r'] = self.shared_data.lidar_r
      msg['camera_l'] = self.shared_data.camera_l
      msg['camera_r'] = self.shared_data.camera_r

      #来自共享数据
      if self.shared_data.roadcate is not None:
        msg['roadcate'] = self.shared_data.roadcate
      if self.shared_data.lat_a is not None:
        msg['lat_a'] = self.shared_data.lat_a
      if self.shared_data.max_curve is not None:
        msg['max_curve'] = self.shared_data.max_curve

      # carrotMan数据
      if self.sm.alive['carrotMan']:
        carrotMan = self.sm['carrotMan']
        msg["desire_speed"] = int(carrotMan.desiredSpeed)  # 期望速度
        msg['atc_type'] = carrotMan.atcType
        msg['road_name'] = carrotMan.szPosRoadName

    # 来自模型的消息
    if self.sm.alive['modelV2']:
      modelV2 = self.sm['modelV2']
      meta = modelV2.meta

      if hasattr(meta, 'blinker'):
        msg['blinker'] = meta.blinker
      if isOnroad:
        msg['l_lane_width'] = round(meta.laneWidthLeft, 1)
        msg['r_lane_width'] = round(meta.laneWidthRight, 1)
        msg['l_edge_dist'] = round(meta.distanceToRoadEdgeLeft, 1)
        msg['r_edge_dist'] = round(meta.distanceToRoadEdgeRight, 1)
        msg['atc_state'] = meta.laneChangeState.raw

        if self.shared_data.max_curve is None and hasattr(modelV2, 'orientationRate') and len(modelV2.orientationRate.z) > 0:
          orientation_rate_z = self._capnp_list_to_list(modelV2.orientationRate.z)
          if orientation_rate_z:
            # 找到最大方向变化率（表示最大曲率点）
            max_index = max(range(len(orientation_rate_z)), key=lambda i: abs(orientation_rate_z[i]))
            max_orientation_rate = orientation_rate_z[max_index]
            #curvature_value = float(max_orientation_rate)
            #curvature_direction = 1 if max_orientation_rate > 0 else -1
            msg['max_curve'] = f1(max_orientation_rate)

        # 如果字段存在才赋值
        if self.shared_data.leftFrontBlind is None and hasattr(meta, 'leftFrontBlind'):
          msg['l_front_blind'] = meta.leftFrontBlind
        if self.shared_data.rightFrontBlind is None and hasattr(meta, 'rightFrontBlind'):
          msg['r_front_blind'] = meta.rightFrontBlind

    #来自selfdriveState消息
    if self.sm.alive['selfdriveState']:
      selfdrive = self.sm['selfdriveState']
      msg['active'] = True if selfdrive.active else False

      # 若 engaged 不存在或为 False，且 active 为 True，则设置 engaged = True
      if selfdrive.active and not msg.get('engaged', False):
        msg['engaged'] = True

    return json.dumps(msg)

  def make_lidar_message(self):
    msg = {}
    msg['ip'] = self.local_ip_address
    msg['port'] = self.listen_port
    isOnroad = self.params.get_bool("IsOnroad")
    msg['IsOnroad'] = isOnroad

    if isOnroad:
      if self.sm.alive['carState']:
        carState = self.sm['carState']
        v_ego_kph = int(carState.vEgoCluster * 3.6 + 0.5)
        v_cruise_kph = carState.vCruise
        msg['v_cruise_kph'] = v_cruise_kph  # 巡航速度
        msg['v_ego_kph'] = v_ego_kph  # 当前速度

      if self.sm.alive['selfdriveState']:
        selfdrive = self.sm['selfdriveState']
        msg['active'] = "on" if selfdrive.active else "off"

      if self.sm.alive['carrotMan']:
        carrotMan = self.sm['carrotMan']
        msg['atc_type'] = carrotMan.atcType
        msg['road_name'] = carrotMan.szPosRoadName

    return json.dumps(msg)

  def make_blinker_message(self):
    msg = {}
    msg['ip'] = self.local_ip_address
    msg['port'] = self.listen_port

    # 来自模型的消息
    if self.sm.alive['modelV2']:
      meta = self.sm['modelV2'].meta
      if hasattr(meta, 'blinker'):
        msg['blinker'] = meta.blinker

    #来自selfdriveState消息
    if self.sm.alive['selfdriveState']:
      selfdrive = self.sm['selfdriveState']
      msg['active'] = "on" if selfdrive.active else "off"

    return json.dumps(msg)

  def make_broadcast_message(self):
    msg = {}
    msg['ip'] = self.local_ip_address
    msg['port'] = self.listen_port

    return json.dumps(msg)

  def navi_get_broadcast_address(self):
    # 修改为支持PC的多接口检测
    if PC:
        interfaces = ['wlan0', 'eth0', 'enp0s3', 'br0']  # 常见PC接口
        for iface in interfaces:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                    ip = fcntl.ioctl(
                        s.fileno(),
                        0x8919,  # SIOCGIFBRDADDR
                        struct.pack('256s', iface.encode('utf-8')[:15])
                    )[20:24]
                    return socket.inet_ntoa(ip)
            except Exception:
                continue
        return "255.255.255.255"  # 回退地址
    else:
      iface = b'wlan0'
    try:
      with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        ip = fcntl.ioctl(
          s.fileno(),
          0x8919,
          struct.pack('256s', iface)
        )[20:24]
        return socket.inet_ntoa(ip)
    except (OSError, Exception):
      return None

  def navi_get_local_ip(self):
      try:
          # 외부 서버와의 연결을 통해 로컬 IP 확인
          with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
              s.connect(("8.8.8.8", 80))  # Google DNS로 연결 시도
              return s.getsockname()[0]
      except Exception as e:
          return f"Error: {e}"

def main():
  amap_navi = AmapNaviServ()
  #amap_navi.navi_comm_thread()

if __name__ == "__main__":
  main()
