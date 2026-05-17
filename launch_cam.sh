#!/bin/bash
cd /data/openpilot/ && 
source .venv/bin/activate && 

# ========================================
# camerad 摄像头服务启动脚本
# 支持: 道路摄像头 + 驾驶员摄像头 (双摄)
# ========================================

# --- 通用设置 ---
export USE_WEBCAM=${USE_WEBCAM:-1}           # 1=USB摄像头, 0=MIPI摄像头
export CAMERAD_DEBUG=${CAMERAD_DEBUG:-1}     # 1=开启调试日志

# --- 道路摄像头 (ROAD_CAM) ---
export ROAD_CAM=${ROAD_CAM:-0}               # /dev/video0
# export ROAD_CAM_WIDTH=1920                  # 可选: 自定义分辨率
# export ROAD_CAM_HEIGHT=1080

# --- 驾驶员摄像头 (DRIVER_CAM) - 双摄模式 ---
# 设置以下环境变量启用第二个摄像头:
export DRIVER_CAM=${DRIVER_CAM:-1}           # /dev/video1
# export DRIVER_CAM_USE_WEBCAM=1             # 可选: 独立设置摄像头类型
# export DRIVER_CAM_WIDTH=640
# export DRIVER_CAM_HEIGHT=480

# --- 启动 camerad ---
echo "=============================="
echo " C AM E R A D   S T A R T"
echo "=============================="
echo "USE_WEBCAM:  $USE_WEBCAM"
echo "ROAD_CAM:    /dev/video${ROAD_CAM}"
echo "DRIVER_CAM:  /dev/video${DRIVER_CAM:-未启用}"
echo "=============================="

exec ./system/camerad/camerad
