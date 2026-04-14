#!/bin/bash

echo "copy logo.ico"
cp logo.ico /home/xuqi/openpilot/
echo "copy PandaSrv.desktop"
sudo cp PandaSrv.desktop /usr/share/applications/
echo "chmod +x PandaSrv.desktop"
sudo chmod +x /usr/share/applications/PandaSrv.desktop
#增加启动脚本
echo "copy stoppandasrv.sh"
cp stoppandasrv.sh /home/xuqi/openpilot/
echo "chmod +x stoppandasrv.sh"
chmod +x /home/xuqi/openpilot/stoppandasrv.sh
#去掉sudo密码认证
echo "$LOGNAME ALL=NOPASSWD: ALL" |sudo tee -a /etc/sudoers
echo "config end"
