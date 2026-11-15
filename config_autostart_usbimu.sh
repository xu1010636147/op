#!/bin/bash
# 一键配置：桌面快捷方式 + 开机自启动（双摄 + 外接 IMU 方案）
#
# 设计要点：
#   - 幂等，可重复执行：不会重复追加 /etc/sudoers
#   - 快捷方式与自启项都直接指向仓库内的脚本，不在 /home 下留副本，
#     避免"改了仓库但副本没更新"的不一致
#   - 不再往 /home 复制 launch 脚本（Exec 直接用仓库绝对路径）
set -e

REPO=/data/openpilot

# ---- 桌面快捷方式 ----
sudo install -m 644 "$REPO/CarrotPilot.desktop" /usr/share/applications/CarrotPilot.desktop

# ---- 开机自启动 ----
mkdir -p "$HOME/.config/autostart"
install -m 755 "$REPO/autostart_launch_pc_usbimu.sh.desktop" \
    "$HOME/.config/autostart/autostart_launch_pc_usbimu.sh.desktop"

# ---- 免密 sudo（幂等）----
# launch_pc_usbimu.sh 开头的 udevadm 需要免密 sudo
if sudo grep -qE "^${LOGNAME:-$USER}[[:space:]]+ALL=\(ALL\)[[:space:]]+NOPASSWD:" /etc/sudoers; then
    echo "sudo 免密配置已存在，跳过"
else
    echo "${LOGNAME:-$USER} ALL=(ALL) NOPASSWD: ALL" | sudo tee -a /etc/sudoers > /dev/null
    echo "已添加 sudo 免密配置"
fi

# ---- 串口访问兜底 ----
# /dev/ttyACM0 已由 98-esp32-s3-imu.rules 设为 0666，正常情况下不需要这个组；
# 保留是为了换用其它 USB 桥（设备名/VID 不匹配规则时）仍能访问串口。
sudo usermod -a -G dialout "${LOGNAME:-$USER}"

echo "配置完成：桌面快捷方式 CarrotPilot + 开机自启动（双摄 + 外接 IMU）"
