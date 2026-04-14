#!/bin/bash

echo "copy logo.ico"
cp logo.ico /home/xuqi/openpilot/
echo "copy Camera.desktop"
sudo cp Camera.desktop /usr/share/applications/
echo "chmod +x Camera.desktop"
sudo chmod +x /usr/share/applications/Camera.desktop
#增加启动脚本
echo "copy launch_cam.sh"
cp launch_cam.sh /home/xuqi/openpilot/
echo "chmod +x launch_cam.sh"
chmod +x /home/xuqi/openpilot/launch_cam.sh
#增加开机启动脚本
if [ ! -d /home/xuqi/openpilot/.config/autostart ]; then
    echo "mkdir .config/autostart"
    mkdir -p /home/xuqi/openpilot/.config/autostart
fi
echo "copy autostart_launch_cam.sh.desktop"
cp autostart_launch_cam.sh.desktop /home/xuqi/openpilot/.config/autostart/
chmod +x /home/xuqi/openpilot/.config/autostart/autostart_launch_cam.sh.desktop
#去掉sudo密码认证
echo "$LOGNAME ALL=NOPASSWD: ALL" |sudo tee -a /etc/sudoers
echo "config autostart end"
