#!/bin/bash
cd /data/openpilot/ && 
source .venv/bin/activate && 

# ========================================
# camerad 摄像头服务启动脚本
# 双摄像头: HF500_8mm (45°) + HF500_1.8mm (170°)
# ========================================

# --- 通用设置 ---
export USE_WEBCAM=1                           # 1=USB摄像头
export CAMERAD_DEBUG=1                        # 调试日志

# --- 道路摄像头: HF500_8mm (45°, 无畸变) ---
# fcam, 用于车道保持和物体检测
export ROAD_CAM=0                             # /dev/video0
export ROAD_CAM_WIDTH=1920
export ROAD_CAM_HEIGHT=1080
export ROAD_CAM_FRAMERATE=20

# --- 广角摄像头: HF500_1.8mm (170°鱼眼) ---
# ecam, 用于盲区检测和近距离感知
export WIDE_CAM=1                             # /dev/video1
export WIDE_ROAD_CAM_WIDTH=1920
export WIDE_ROAD_CAM_HEIGHT=1080
export WIDE_ROAD_CAM_FRAMERATE=20

# --- 启动 ---
echo "=============================="
echo " C A M E R A D   S T A R T"
echo "=============================="
echo "模式:      USB 摄像头"
echo "ROAD:      HF500_8mm  (45°)  1920x1080"
echo "WIDE:      HF500_1.8mm (170°) 1920x1080"
echo "=============================="

exec ./system/camerad/camerad
