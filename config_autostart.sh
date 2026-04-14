#!/bin/bash

echo "copy logo.ico"
cp logo.ico /home/xuqi/openpilot/
echo "copy sunnypilot.desktop"
sudo cp sunnypilot.desktop /usr/share/applications/
echo "chmod +x sunnypilot.desktop"
sudo chmod +x /usr/share/applications/sunnypilot.desktop
#增加启动脚本
echo "copy launch_pc.sh"
cp launch_pc.sh /home/xuqi/openpilot/
echo "chmod +x launch_pc.sh"
chmod +x /home/xuqi/openpilot/launch_pc.sh
#增加开机启动脚本
if [ ! -d /home/xuqi/openpilot/.config/autostart ]; then
    echo "mkdir .config/autostart"
    mkdir -p /home/xuqi/openpilot/.config/autostart
fi
echo "copy autostart_launch_pc.sh.desktop"
cp autostart_launch_pc.sh.desktop /home/xuqi/openpilot/.config/autostart/
chmod +x /home/xuqi/openpilot/.config/autostart/autostart_launch_pc.sh.desktop
#去掉sudo密码认证
echo "$LOGNAME ALL=NOPASSWD: ALL" |sudo tee -a /etc/sudoers
echo "config autostart end"
