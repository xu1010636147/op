"""
LSM6DS3 外接 IMU 固件（YD-ESP32-S3 / MicroPython）

采样 LSM6DS3 六轴数据，以 16 字节定长帧从 UART0 输出，由 openpilot 侧的
system/sensord/sensord_usbimu.py 接收并转成 accelerometer / gyroscope cereal 消息。
帧格式定义见同目录 README.md。

坐标约定：本固件只输出芯片原始轴序 (x, y, z)，不做任何轴映射与单位换算。
          轴映射放在 openpilot 侧，上车实测时改一个 Python 常量即可，无需重烧固件。

用法：用 Thonny 或 mpremote 把本文件保存到开发板的 main.py，复位后自动运行。
"""

import sys
import time

from machine import I2C, Pin

# ============================ 硬件配置 ============================
# 按实际接线修改。ESP32-S3 的 I2C0 默认落在 GPIO8(SDA) / GPIO9(SCL)。
I2C_ID = 0
I2C_SCL_PIN = 9
I2C_SDA_PIN = 8
I2C_FREQ = 400_000

# LSM6DS3 的 7 位地址由模块上 SA0 焊盘决定：接地 = 0x6A，接 VDD = 0x6B。
# 启动时按扫描结果自动选择，两种接线都能工作。
CANDIDATE_ADDRS = (0x6A, 0x6B)

# ========================= LSM6DS3 寄存器 =========================
# 定义与 system/sensord/sensors/lsm6ds3_accel.h、lsm6ds3_gyro.h 保持一致
REG_WHO_AM_I = 0x0F
REG_CTRL1_XL = 0x10   # 加速度计 ODR / 量程
REG_CTRL2_G = 0x11    # 陀螺仪 ODR / 量程
REG_CTRL3_C = 0x12    # 块更新 / 寄存器自增
REG_STATUS = 0x1E     # 数据就绪标志
REG_OUTX_L_G = 0x22   # 陀螺仪起址，连续 12 字节 = gyro xyz + accel xyz

# WHO_AM_I 允许值：0x69 = LSM6DS3，0x6A = LSM6DS3TR-C（同一颗芯片的卷带封装版）
VALID_CHIP_IDS = (0x69, 0x6A)

# CTRL1_XL = ODR 104Hz(bits[7:4]=0b0100) | FS ±4g(bits[3:2]=0b10)
# 满量程灵敏度 32768 / 4g = 8192 LSB/g
CTRL1_XL_104HZ_4G = 0x48

# CTRL2_G = ODR 104Hz(bits[7:4]=0b0100) | FS ±500dps(bits[3:2]=0b01)
# 满量程灵敏度 32768 / 500dps = 65.536 LSB/(°/s)
CTRL2_G_104HZ_500DPS = 0x44

# CTRL3_C = BDU(bit6) | IF_INC(bit2)
# BDU 保证 12 字节输出寄存器在一次读取期间不被刷新，IF_INC 让多字节读地址自增
CTRL3_C_BDU_IFINC = 0x44

STATUS_XLDA = 0x01    # 加速度计新数据就绪


# ============================ 帧协议 ============================
FRAME_HEADER_0 = 0xAA
FRAME_HEADER_1 = 0x55
FRAME_SIZE = 16       # 2 帧头 + 1 序号 + 6 加速度 + 6 角速度 + 1 校验和


def fatal(msg):
  """配置失败时停在原地并打印原因。此时还没进入二进制数据流，主机端能直接看到文本。"""
  print("FATAL:", msg)
  while True:
    time.sleep_ms(1000)


def build_frame(seq, raw):
  """把一次 I2C 读到的 12 字节原始数据打包成协议帧。

  raw 布局为 gyro x,y,z + accel x,y,z（各 int16 小端），帧内调整为
  accel 在前、gyro 在后，与 README 的协议表一致。
  """
  frame = bytearray(FRAME_SIZE)
  frame[0] = FRAME_HEADER_0
  frame[1] = FRAME_HEADER_1
  frame[2] = seq
  frame[3:9] = raw[6:12]    # accel x, y, z
  frame[9:15] = raw[0:6]    # gyro x, y, z
  frame[15] = sum(frame[:15]) & 0xFF
  return frame


def main():
  i2c = I2C(I2C_ID, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=I2C_FREQ)

  found = i2c.scan()
  print("I2C scan:", [hex(a) for a in found])

  addr = next((a for a in CANDIDATE_ADDRS if a in found), None)
  if addr is None:
    fatal("未找到 LSM6DS3（扫描结果 %s）。检查供电、SCL/SDA 是否接反、以及 CS 是否接 3V3" %
          [hex(a) for a in found])
  print("使用 I2C 地址: 0x%02X" % addr)

  chip_id = i2c.readfrom_mem(addr, REG_WHO_AM_I, 1)[0]
  print("WHO_AM_I = 0x%02X" % chip_id)
  if chip_id not in VALID_CHIP_IDS:
    fatal("WHO_AM_I 异常 0x%02X，期望 %s" % (chip_id, [hex(i) for i in VALID_CHIP_IDS]))

  i2c.writeto_mem(addr, REG_CTRL3_C, bytes([CTRL3_C_BDU_IFINC]))
  i2c.writeto_mem(addr, REG_CTRL1_XL, bytes([CTRL1_XL_104HZ_4G]))
  i2c.writeto_mem(addr, REG_CTRL2_G, bytes([CTRL2_G_104HZ_500DPS]))

  # 等第一个采样周期稳定
  time.sleep_ms(50)
  print("LSM6DS3 ready, streaming 104Hz")

  # MicroPython 的 stdout 绑定在 UART0（GPIO43/44），即板上的 COM 口。
  # 取 .buffer 绕过文本层，直接写原始字节。
  out = getattr(sys.stdout, "buffer", sys.stdout)

  seq = 0
  while True:
    # 轮询数据就绪标志，采样节奏自动跟随芯片 ODR，不依赖 MicroPython 的 sleep 精度
    if not (i2c.readfrom_mem(addr, REG_STATUS, 1)[0] & STATUS_XLDA):
      continue

    # 一次读完 12 字节，保证加速度与角速度出自同一采样时刻
    raw = i2c.readfrom_mem(addr, REG_OUTX_L_G, 12)

    out.write(build_frame(seq, raw))
    seq = (seq + 1) & 0xFF


if __name__ == "__main__":
  main()
