#!/bin/bash
cd /home/xuqi/openpilot &&
source op_yolo_venv/bin/activate &&
cd /home/xuqi/openpilot &&
export PYTHONPATH=/home/xuqi/openpilot
python3 lane.py

