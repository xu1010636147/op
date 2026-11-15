#!/usr/bin/env python3
import threading
import os
from collections import namedtuple

from msgq.visionipc import VisionIpcServer, VisionStreamType
from cereal import messaging

from openpilot.tools.webcam.camera import CameraMJPG
from openpilot.common.realtime import Ratekeeper

WIDE_CAM = os.getenv("WIDE_CAM")
CameraType = namedtuple("CameraType", ["msg_name", "stream_type", "cam_id"])

# 相机分辨率。必须在 create_buffers() 之前确定，而 create_buffers() 必须先于
# start_listener()，所以这里写成常量而不是从设备读回 —— 否则 camerad 的启动
# 会被绑在"相机此刻已枚举完成"上，设备刚重枚举的那一瞬间就起不来。
# 改这里等于改 visionipc 的 buffer 尺寸，两路相机必须一致。
CAM_W, CAM_H = 1920, 1080

def resolve_cam_device(name):
  # ROAD_CAM / WIDE_CAM 可写视频编号（"0" → /dev/video0），也可写 by-path 名
  # （"pci-…" → /dev/v4l/by-path/pci-…）。by-path 与机器插槽绑定，比 /dev/videoN 稳定，
  # 但换机器或改接线后会失效，需用下面命令重新获取：
  #   ls /dev/v4l/by-path/ | grep usb-0
  if name.startswith("pci-"):
    return f"/dev/v4l/by-path/{name}"
  return f"/dev/video{name}"

CAMERAS = [
  CameraType("roadCameraState", VisionStreamType.VISION_STREAM_ROAD, os.getenv("ROAD_CAM", "0")),
  # CameraType("driverCameraState", VisionStreamType.VISION_STREAM_DRIVER, os.getenv("DRIVER_CAM", "2")),
]
if WIDE_CAM:
  CAMERAS.append(CameraType("wideRoadCameraState", VisionStreamType.VISION_STREAM_WIDE_ROAD, WIDE_CAM))

class Camerad:
  def __init__(self):
    self.pm = messaging.PubMaster([c.msg_name for c in CAMERAS])
    self.vipc_server = VisionIpcServer("camerad")

    self.cameras = []
    for c in CAMERAS:
      cam_device = resolve_cam_device(c.cam_id)
      print(f"opening {c.msg_name} at {cam_device}")
      cam = CameraMJPG(c.msg_name, c.stream_type, cam_device, CAM_W, CAM_H)
      self.cameras.append(cam)
      self.vipc_server.create_buffers(c.stream_type, 20, cam.W, cam.H)

    self.vipc_server.start_listener()

  def _send_yuv(self, yuv, frame_id, pub_type, yuv_type):
    eof = int(frame_id * 0.05 * 1e9)
    self.vipc_server.send(yuv_type, yuv, frame_id, eof, eof)
    dat = messaging.new_message(pub_type, valid=True)
    msg = {
      "frameId": frame_id,
      "transform": [1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0]
    }
    setattr(dat, pub_type, msg)
    self.pm.send(pub_type, dat)

  def camera_runner(self, cam):
    rk = Ratekeeper(20, None)
    for yuv in cam.read_frames():
      self._send_yuv(yuv, cam.cur_frame_id, cam.cam_type_state, cam.stream_type)
      cam.cur_frame_id += 1
      rk.keep_time()

  def run(self):
    threads = []
    for cam in self.cameras:
      cam_thread = threading.Thread(target=self.camera_runner, args=(cam,))
      cam_thread.start()
      threads.append(cam_thread)

    for t in threads:
      t.join()


def main():
  camerad = Camerad()
  camerad.run()


if __name__ == "__main__":
  main()
