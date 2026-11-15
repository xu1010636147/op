#!/usr/bin/env python3
"""
USB 外接 IMU 接收进程（ESP32-S3 + LSM6DS3）

从 USB CDC 串口读取 16 字节定长帧（协议见 system/sensord/esp32_usb_imu/README.md），
转成标准的 accelerometer / gyroscope cereal 消息后发布，供原版 locationd 的 EKF 消费。

为什么不需要改 locationd：
  source 用 SensorSource.lsm6ds3（cereal/log.capnp:405）。原版 locationd 只在
  selfdrive/locationd/locationd.cc:252-255 过滤 BMX055 这一个 source，对 lsm6ds3
  放行。所以 locationd.cc / locationd.h 一行都不用动。

为什么轴映射放在这里而不是固件里：
  模块安装朝向必须上车实测、反复调整。放在 Python 侧改一个常量重启进程即可，
  不必每次重烧 ESP32。

独立调试：python3 system/sensord/sensord_usbimu.py --probe
"""

import argparse
import math
import os
import struct
import time

import serial

import cereal.messaging as messaging
from cereal import log
from openpilot.common.realtime import config_realtime_process
from openpilot.common.swaglog import cloudlog

# ============================ 帧协议 ============================
FRAME_HEADER = b'\xaa\x55'
FRAME_SIZE = 16
CHECKSUM_OFFSET = 15

DEFAULT_DEVICE = "/dev/ttyACM0"
DEFAULT_BAUD = 115200   # USB CDC 忽略波特率，仅为兼容将来的 USB-TTL 场景
READ_CHUNK = FRAME_SIZE * 16

# ============================ 量程换算 ============================
# 必须与固件的 CTRL1_XL / CTRL2_G 配置一致（esp32_usb_imu/main.py）
ACCEL_FULL_SCALE_G = 4.0      # ±4g
GYRO_FULL_SCALE_DPS = 500.0   # ±500dps

G_TO_MS2 = 9.80665
ACCEL_SCALE = ACCEL_FULL_SCALE_G * G_TO_MS2 / 32768.0        # raw -> m/s^2
GYRO_SCALE = GYRO_FULL_SCALE_DPS * math.pi / 180.0 / 32768.0  # raw -> rad/s

# ============ 传感器消息字段（system/sensord/sensors/constants.h）============
SENSOR_ACCELEROMETER = 1
SENSOR_GYRO_UNCALIBRATED = 5
SENSOR_TYPE_ACCELEROMETER = 1
SENSOR_TYPE_GYROSCOPE_UNCALIBRATED = 16


def map_axes(x, y, z):
  """芯片原始轴序 -> 发布给 locationd 的轴序。

  对应安装姿态：**模块平放，芯片 +X 朝向车头**（注意实测 VIN 端 = 芯片 -X，
  所以这一姿态等价于 VIN 端朝车尾）。

  朝向由重力实测确定：
    模块平放                     -> 重力落在芯片 -Z，即芯片 +Z 轴朝下
    把 VIN 端抬起 30 度          -> 变化出现在芯片 X 轴，即 VIN 端沿芯片 X 轴方向
  按上面选定的安装朝向，车体系（X 前 / Y 右 / Z 下）与芯片轴的对应为：
    车辆 X = 芯片 +X,  车辆 Y = 芯片 +Y,  车辆 Z = 芯片 +Z

  locationd 期望 v = (-d_z, -d_y, -d_x)，d 为车体系下的量（locationd.cc:259,294），
  代入即得下面的映射。用平放静止可验证：芯片 (0, 0, -9.8) -> v = (9.8, 0, 0)，
  第一分量恰为重力，与 locationd 的约定吻合。

  另注：绕垂直轴的朝向（哪一端朝车头）无法由重力观测，是按安装约定选定的，
  需实车直线行驶时看横摆分量方向来最终确认；不同就翻对应项的符号。

  装法改变时必须重新标定，要改的就是这一个函数。
  """
  return -z, -y, -x


def extract_frames(buf):
  """从缓冲区就地提取所有完整且校验通过的帧。

  buf 为 bytearray，被就地消费：已取走的字节会被移除。
  返回本次提取出的帧（bytes 列表）。
  """
  frames = []
  while True:
    idx = buf.find(FRAME_HEADER)
    if idx < 0:
      # 整个缓冲区没有帧头，只保留最后 1 字节——它可能是被截断的帧头前半
      if len(buf) > 1:
        del buf[:len(buf) - 1]
      return frames

    if idx > 0:
      del buf[:idx]   # 丢弃帧头之前的噪声

    if len(buf) < FRAME_SIZE:
      return frames   # 数据还没收齐，等下一批

    if (sum(buf[:CHECKSUM_OFFSET]) & 0xFF) != buf[CHECKSUM_OFFSET]:
      # 校验失败说明这个帧头是误匹配，只丢 1 字节让后续帧头重新对齐
      del buf[:1]
      continue

    frames.append(bytes(buf[:FRAME_SIZE]))
    del buf[:FRAME_SIZE]


def decode_frame(frame):
  """解出一帧的序号与六轴原始值（芯片原始轴序，未换算）。"""
  return struct.unpack_from('<Bhhhhhh', frame, 2)


def build_messages(seq, ax, ay, az, gx, gy, gz, ts_ns):
  """把一帧原始数据转成 accelerometer / gyroscope 两条 cereal 消息。

  locationd 会把 v 内部做 (-v[2], -v[1], -v[0]) 变换（locationd.cc:259, 294），
  所以这里按 sensord 的约定填轴序即可，不要预先做那步变换。
  """
  accel = [v * ACCEL_SCALE for v in map_axes(ax, ay, az)]
  gyro = [v * GYRO_SCALE for v in map_axes(gx, gy, gz)]

  accel_msg = messaging.new_message('accelerometer', valid=True)
  accel_msg.accelerometer.timestamp = ts_ns
  accel_msg.accelerometer.source = log.SensorEventData.SensorSource.lsm6ds3
  accel_msg.accelerometer.sensor = SENSOR_ACCELEROMETER
  accel_msg.accelerometer.type = SENSOR_TYPE_ACCELEROMETER
  accel_msg.accelerometer.version = 1
  # acceleration 是 capnp union 成员，必须先 init 才能赋值
  acceleration = accel_msg.accelerometer.init('acceleration')
  acceleration.v = accel
  acceleration.status = 0

  gyro_msg = messaging.new_message('gyroscope', valid=True)
  gyro_msg.gyroscope.timestamp = ts_ns
  gyro_msg.gyroscope.source = log.SensorEventData.SensorSource.lsm6ds3
  gyro_msg.gyroscope.sensor = SENSOR_GYRO_UNCALIBRATED
  gyro_msg.gyroscope.type = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED
  gyro_msg.gyroscope.version = 2
  # 注意必须是 gyroUncalibrated 而不是 gyro——locationd 只认前者（locationd.cc:258）
  gyro_uncalibrated = gyro_msg.gyroscope.init('gyroUncalibrated')
  gyro_uncalibrated.v = gyro
  gyro_uncalibrated.status = 0

  return accel_msg, gyro_msg


def open_serial(device, baud):
  try:
    return serial.Serial(device, baud, timeout=0.005, exclusive=True)
  except serial.SerialException as e:
    cloudlog.error(f"无法打开串口 {device}: {e}（检查设备是否存在，以及当前用户是否在 dialout 组）")
    return None


def probe_report(stat, now):
  """--probe 模式下的周期性统计输出。"""
  elapsed = now - stat['last_report']
  if elapsed < 1.0:
    return
  rate = stat['count'] / elapsed
  accel_norm = math.sqrt(sum(v * v for v in stat['accel']))
  cloudlog.info("[probe] %.1f Hz, 帧总数 %d, 丢帧 %d, accel=(%.3f, %.3f, %.3f) |a|=%.3f m/s^2, gyro=(%.4f, %.4f, %.4f) rad/s"
                % (rate, stat['total'], stat['dropped'],
                   stat['accel'][0], stat['accel'][1], stat['accel'][2], accel_norm,
                   stat['gyro'][0], stat['gyro'][1], stat['gyro'][2]))
  stat['count'] = 0
  stat['last_report'] = now


def main():
  parser = argparse.ArgumentParser(description="USB 外接 IMU 接收进程")
  parser.add_argument("--device", default=os.getenv("USB_IMU_DEV", DEFAULT_DEVICE),
                      help=f"串口设备路径，默认 {DEFAULT_DEVICE}（亦可用环境变量 USB_IMU_DEV 指定）")
  parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="波特率，USB CDC 下无效")
  parser.add_argument("--probe", action="store_true", help="额外打印实时帧率、丢帧数与六轴读数")
  args = parser.parse_args()

  if not args.probe:
    config_realtime_process([0, 1, 2, 3], 5)

  ser = open_serial(args.device, args.baud)
  if ser is None:
    return

  cloudlog.info(f"外接 IMU 已连接: {args.device}, source=lsm6ds3, 轴映射=y,-x,z")

  pm = messaging.PubMaster(['accelerometer', 'gyroscope'])
  buf = bytearray()

  stat = {'count': 0, 'total': 0, 'dropped': 0, 'last_report': time.monotonic(),
          'accel': [0.0, 0.0, 0.0], 'gyro': [0.0, 0.0, 0.0]}
  last_seq = None

  while True:
    data = ser.read(READ_CHUNK)
    if not data:
      continue

    buf.extend(data)
    for frame in extract_frames(buf):
      seq, ax, ay, az, gx, gy, gz = decode_frame(frame)

      # 序号不连续说明中间丢过帧，仅用于诊断（USB 传输本身有 CRC 保护）
      if last_seq is not None and seq != (last_seq + 1) & 0xFF:
        stat['dropped'] += (seq - last_seq - 1) & 0xFF
      last_seq = seq

      # 时间戳用接收时刻：与 new_message 自动填入的 logMonoTime 同源（均为 monotonic），
      # 差值即 USB 传输延迟（约 1ms），远低于 locationd.cc:243 的 100ms 阈值
      ts_ns = int(time.monotonic() * 1e9)
      accel_msg, gyro_msg = build_messages(seq, ax, ay, az, gx, gy, gz, ts_ns)
      pm.send('accelerometer', accel_msg)
      pm.send('gyroscope', gyro_msg)

      stat['count'] += 1
      stat['total'] += 1
      stat['accel'] = list(accel_msg.accelerometer.acceleration.v)
      stat['gyro'] = list(gyro_msg.gyroscope.gyroUncalibrated.v)

    if args.probe:
      probe_report(stat, time.monotonic())

    # 串口静默超过 1 秒视为传感器掉线，打印一次便于排查
    if time.monotonic() - stat['last_report'] > 5.0 and stat['count'] == 0:
      cloudlog.error("外接 IMU 已超过 5 秒没有数据，检查 ESP32 固件是否在运行、USB 线是否插好")
      stat['last_report'] = time.monotonic()


if __name__ == "__main__":
  main()
