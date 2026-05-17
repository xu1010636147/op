#!/bin/bash
cd /data/openpilot/ && 
source .venv/bin/activate && 

# camerad (C++ 摄像头采集服务)
# 环境变量说明:
# USE_WEBCAM=1        - 使用 USB 摄像头
# ROAD_CAM=0          - 道路摄像头设备编号 /dev/video0
# DRIVER_CAM=1        - 驾驶员摄像头 /dev/video1 (可选)
# CAMERAD_DEBUG=1     - 调试模式 (可选)
export USE_WEBCAM=1
export ROAD_CAM=0

exec ./system/camerad/camerad
