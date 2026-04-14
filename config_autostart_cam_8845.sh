#!/bin/bash

#增加启动脚本
echo "copy launch_cam_8845.sh"
cp launch_cam_8845.sh /home/xuqi/openpilot/
echo "chmod +x launch_cam_8845.sh"
chmod +x /home/xuqi/openpilot/launch_cam_8845.sh
#增加开机启动脚本
if [ ! -d /home/xuqi/openpilot/.config/autostart ]; then
    echo "mkdir .config/autostart"
    mkdir -p /home/xuqi/openpilot/.config/autostart
fi
echo "copy autostart_launch_cam_8845.sh.desktop"
cp autostart_launch_cam_8845.sh.desktop /home/xuqi/openpilot/.config/autostart/
chmod +x /home/xuqi/openpilot/.config/autostart/autostart_launch_cam_8845.sh.desktop
#去掉sudo密码认证
echo "$LOGNAME ALL=NOPASSWD: ALL" |sudo tee -a /etc/sudoers
echo "config autostart end"
