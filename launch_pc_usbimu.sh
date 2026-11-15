#!/bin/bash
# 重新加载 udev 规则，让 panda 等 USB 设备拿到正确的访问权限
# 注意：不要用无参数的 `udevadm trigger`——它会对全系统所有设备发 change 事件，
#       造成 udev 事件风暴（mtp-probe 被反复拉起）。这里只定向触发 USB 子系统。
#       用 --action=change 重新应用规则即可，不会让设备重新枚举。
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb --action=change
# 运行op（USB_IMU=1 启用外接 IMU：ESP32-S3 + LSM6DS3，默认设备 /dev/ttyACM0）
# 双摄：ROAD_CAM/WIDE_CAM 取 /dev/v4l/by-path/ 下的 by-path 名（比 /dev/videoN 稳定）。
# 换机器或改接线后用 `ls /dev/v4l/by-path/ | grep usb-0` 重新获取路径。
# 两路都为 0bda:5161 RGB Camera，型号相同，只能靠画面区分（详见已抓帧比对）：
#   65:00.3-usb-0:2 → 前视，视野窄、畸变小、车牌清晰
#   67:00.4-usb-0:1 → 广角，视野宽、边缘畸变明显
cd /data/openpilot &&
source .venv/bin/activate &&
QT_QPA_PLATFORM=xcb USE_WEBCAM=1 ROAD_CAM=pci-0000:65:00.3-usb-0:2:1.0-video-index0 WIDE_CAM=pci-0000:67:00.4-usb-0:1:1.0-video-index0 NO_DM=0 USB_IMU=1 system/manager/manager.py
