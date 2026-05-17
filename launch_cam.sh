#!/bin/bash
cd /data/openpilot/ && 
source .venv/bin/activate && 

# ========================================
# camerad 摄像头服务启动脚本
# 双摄像头: HF500_8mm (45°) + HF500_1.8mm (170°)
# ========================================

# --- 通用设置 ---
export USE_WEBCAM=0                           # 0=MIPI摄像头
export CAMERAD_DEBUG=1                        # 调试日志

# --- 道路摄像头: HF500_8mm (45°, 无畸变) ---
# 主视野，用于车道保持和物体检测
export ROAD_CAM=0                             # /dev/video0
export ROAD_CAM_WIDTH=1920
export ROAD_CAM_HEIGHT=1080
export ROAD_CAM_FRAMERATE=20

# --- 广角摄像头: HF500_1.8mm (170°) ---
# 宽视野，用于盲区检测和近距离感知
export WIDE_CAM=1                            # /dev/video1 (代码中用 WIDE_CAM)
# export WIDE_ROAD_CAM_WIDTH=1920
# export WIDE_ROAD_CAM_HEIGHT=1080
# export WIDE_ROAD_CAM_FRAMERATE=20

# --- 驾驶员摄像头 (可选) ---
# 如果你的设备有第三颗摄像头拍驾驶员:
# export DRIVER_CAM=2                         # /dev/video2

# --- 启动 ---
echo "=============================="
echo " C A M E R A D   S T A R T"
echo "=============================="
echo "模式:      MIPI 摄像头"
echo "ROAD:      HF500_8mm  (45°)"
echo "WIDE:      HF500_1.8mm (170°)  ← 新增"
echo "=============================="

exec ./system/camerad/camerad
