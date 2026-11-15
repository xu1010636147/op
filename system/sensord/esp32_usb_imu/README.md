# LSM6DS3 + ESP32-S3 外接 IMU

给 PC 平台（x86_64 Linux）的 openpilot 补上缺失的 IMU 数据源。

PC 平台没有板载 IMU，且 `sensord` 被 `enabled=not PC` 禁用（`system/manager/process_config.py`），
而 locationd 的 EKF 完全被 `accelerometer` + `gyroscope` 两条消息门控。不接这个模块时，
`liveLocationKalman.inputsOK` 恒为 false。

```
LSM6DS3 ──I2C──> ESP32-S3 ──UART0──> CH343 ──USB──> sensord_usbimu.py ──cereal──> locationd (原版, 零改动)
```

## 硬件接线

板子上的 **COM 口**（CH343 USB-UART 桥）与 **USB 口**（ESP32-S3 原生 USB）都能在 Linux 下
枚举成 `/dev/ttyACM0`，但本方案走 **COM 口**：

- UART 是硬件外设，时序确定，比原生 USB-CDC 更适合 104 Hz 连续数据流
- 原生 USB 在部分 MicroPython 版本上会掉线重枚举
- MicroPython 的 REPL 就在 UART0 上（即 COM 口），烧录与数据共用这一个口

| LSM6DS3 模块 | 接到 ESP32-S3 | 说明 |
|---|---|---|
| VIN 或 3V3 | 3V3 | 供电 |
| GND | GND | 共地 |
| SCL | GPIO9 | I2C 时钟 |
| SDA | GPIO8 | I2C 数据 |
| **CS** | **3V3** | **必须拉高才进入 I2C 模式，这是最常见的接线错误** |
| SA0 | GND 或 3V3 | 决定地址：接地 `0x6A`，接 3V3 `0x6B`。固件会自动识别两者 |
| INT1 / INT2 | 不接 | 本固件用轮询数据就绪标志，不用中断 |
| SCX / SDX / OCS | 不接 | 辅助 SPI 接口，I2C 模式下不用 |

引脚按实际情况改 `main.py` 顶部的 `I2C_SCL_PIN` / `I2C_SDA_PIN`。ESP32-S3 的 I2C0
默认落在 GPIO8(SDA) / GPIO9(SCL)。

## 首次准备：串口权限

CH343 的 VID:PID 是 `1a86:55d3`，默认设备节点属 `root:dialout` 且权限 `660`，普通用户打不开。
加一条 udev 规则即可（一次配置，重插和重启都有效）：

```bash
sudo tee /etc/udev/rules.d/99-esp32-s3-imu.rules > /dev/null <<'EOF'
# ESP32-S3 外接 IMU（沁恒 CH343 USB-UART 桥）
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", MODE="0666"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty
```

替代做法是 `sudo usermod -a -G dialout $LOGNAME`，但必须注销重新登录才生效。

## 烧录 MicroPython 固件

工具装在独立环境里，不污染 openpilot 的 venv：

```bash
uv tool install esptool mpremote        # 可执行文件落在 ~/.local/bin
```

下载固件并烧录（标准变体即可，不需要 PSRAM——本项目固件只用几 KB 内存）：

```bash
ESPTOOL=~/.local/bin/esptool
PORT=/dev/ttyACM0

curl -sSL -o /tmp/micropython-s3.bin \
  https://micropython.org/resources/firmware/ESP32_GENERIC_S3-20260824-v1.29.0.bin

$ESPTOOL --port $PORT flash-id                    # 先确认芯片：应为 ESP32-S3 / 16MB flash
$ESPTOOL --port $PORT erase-flash
$ESPTOOL --port $PORT --baud 460800 write-flash 0x0 /tmp/micropython-s3.bin
```

> `esptool v5` 的子命令用连字符（`erase-flash` / `write-flash`），与 v4 的下划线写法不同。

## 上传固件并运行

```bash
~/.local/bin/mpremote connect /dev/ttyACM0 fs cp \
  system/sensord/esp32_usb_imu/main.py :main.py
```

上传后复位板子（或在 REPL 里 `Ctrl-D`），启动自检会打印：

```
I2C scan: ['0x6b']
使用 I2C 地址: 0x6B
WHO_AM_I = 0x69
LSM6DS3 ready, streaming 104Hz
```

一旦出现最后一行，串口就开始输出二进制数据流，此时**普通串口终端会显示乱码，这是正常的**。
用 Thonny 连同一个口会打断数据流，调试完记得断开。

启动自检失败时固件会停在 `FATAL: ...` 并打印原因，不会进入数据流。

## 协议

16 字节定长帧，多字节字段为小端。

| 偏移 | 长度 | 内容 |
|---|---|---|
| 0 | 2 | 帧头 `0xAA 0x55` |
| 2 | 1 | 序号 seq，0–255 循环 |
| 3 | 6 | 加速度 x,y,z — `int16`，原始值 |
| 9 | 6 | 角速度 x,y,z — `int16`，原始值 |
| 15 | 1 | 校验和 = 前 15 字节之和 `& 0xFF` |

原始值换算（量程由 `main.py` 的 `CTRL1_XL_104HZ_4G` / `CTRL2_G_104HZ_500DPS` 决定）：

- 加速度：`raw / 8192 = g`，即 `raw / 32768 × 4 × 9.80665 = m/s²`
- 角速度：`raw / 65.536 = °/s`，即 `raw / 32768 × 500 × π/180 = rad/s`

帧数据是**芯片原始轴序**，不做任何映射——轴映射由 openpilot 侧的
`system/sensord/sensord_usbimu.py` 里 `map_axes()` 负责，方便上车实测时快速调整，
不必重烧固件。

## openpilot 侧使用

```bash
# 台架调试：直接看帧率、丢帧数和六轴读数，不启动整套 openpilot
python3 system/sensord/sensord_usbimu.py --probe

# 完整运行
USB_IMU=1 system/manager/manager.py

# 设备不在默认路径时
USB_IMU=1 USB_IMU_DEV=/dev/ttyUSB0 system/manager/manager.py
```

开机自启用 `./config_autostart_usbimu.sh`（卸载用 `./remove_autostart_usbimu.sh`）。

## 故障排查

| 现象 | 原因与处理 |
|---|---|
| `I2C scan: []` + `未找到 LSM6DS3` | 接线问题。重点查 **CS 是否接 3V3**、SDA/SCL 是否接反、模块是否供电 |
| `WHO_AM_I 异常` | 地址扫到了但芯片不对，可能扫到总线上其它设备 |
| `无法打开串口 /dev/ttyACM0` | 设备不存在（换 USB 口 / 换根数据线，有些线只供电不传数据），或权限没配（见上文 udev 规则） |
| 打开串口报「设备忙」 | Thonny 或其它终端占着，先断开 |
| 串口能打开但一直无数据 | 板子没在跑固件。复位看有没有启动自检输出；没有则重新上传 `main.py` |
| 完全无输出、连 ROM 日志都没有 | 板子可能卡在 PSRAM 初始化（见下）或未上电 |

## 已知问题

**`quad_psram: PSRAM chip is not connected, or wrong PSRAM line mode`**

本板（YD-ESP32-S3）的 8 MB PSRAM 是 Octal 类型，而 MicroPython 的标准
`ESP32_GENERIC_S3` 固件按 Quad 模式初始化 PSRAM，所以每次启动都会报这几行错：

```
E (307) quad_psram: PSRAM chip is not connected, or wrong PSRAM line mode
E (307) esp_psram: PSRAM enabled but initialization failed. Bailing out.
E cpu_start: Failed to init external RAM; continuing without it.
```

**不影响功能**——注意最后一行 `continuing without it`，芯片继续启动，PSRAM 只是不可用。
本固件只用几 KB 内存，用不到那 8 MB。可以忽略这些启动噪音。

## 配置速查

| 参数 | 位置 | 当前值 |
|---|---|---|
| 采样率 ODR | `main.py` 的 `CTRL1_XL_104HZ_4G` / `CTRL2_G_104HZ_500DPS` | 104 Hz |
| 加速度量程 | 同上（`CTRL1_XL` 的 FS 位） | ±4 g |
| 角速度量程 | 同上（`CTRL2_G` 的 FS 位） | ±500 °/s |
| I2C 地址 | `main.py` 的 `CANDIDATE_ADDRS` | 自动识别 `0x6A`/`0x6B` |
| 轴映射 | `sensord_usbimu.py` 的 `map_axes()` | `(y, -x, z)` |
| 串口设备 | 环境变量 `USB_IMU_DEV` | `/dev/ttyACM0` |

改量程时必须**同时**改 `main.py` 的寄存器常量和 `sensord_usbimu.py` 的
`ACCEL_FULL_SCALE_G` / `GYRO_FULL_SCALE_DPS`，否则读数会整体缩放错误。
