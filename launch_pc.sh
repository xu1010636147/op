#!/bin/bash
# 重新加载 udev 规则并触发设备扫描
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo chmod 777 /dev/bus/usb/*

cd /data/openpilot && 
source .venv/bin/activate && 

# 摄像头配置 (camerad 使用)
export USE_WEBCAM=1
export ROAD_CAM=0
export WIDE_CAM=1
export NO_DM=1                               # 禁用驾驶员摄像头

# 启动主管理器
system/manager/manager.py
