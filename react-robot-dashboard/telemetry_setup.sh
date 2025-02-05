#!/bin/bash

echo START: sudo apt update
yes | sudo apt update
echo END: sudo apt update

echo START: sudo apt upgrade
yes | sudo apt upgrade
echo END: sudo apt upgrade

echo START: sudo apt install npm
yes | sudo apt install npm
echo END: sudo apt install npm

#echo START: sudo npm install -g serve
#yes | sudo npm install -g serve
#echo END: sudo npm install -g serve
echo SKIPPING sudo npm install -g serve

echo SET UP DIRECTORIES
sudo mkdir /WS
sudo chown -R pi /WS
cd /WS

echo CLONE REPO
git clone https://github.com/legokor/TanfolyamRobot.git
git config --global --add safe.directory /WS/TanfolyamRobot
cd TanfolyamRobot
git checkout telemetry_page_rpi

echo START NPM
cd /WS/TanfolyamRobot/react-robot-dashboard/my-robot-server && npm install
cd /WS/TanfolyamRobot/react-robot-dashboard/robot-dashboard && npm install
chmod +x /WS/TanfolyamRobot/react-robot-dashboard/start.sh

echo START SERVICE
sudo cp /WS/TanfolyamRobot/react-robot-dashboard/tanfrobot-telemetry.service /etc/systemd/system/tanfrobot-telemetry.service
sudo systemctl daemon-reload
sudo systemctl enable tanfrobot-telemetry.service
sudo systemctl start tanfrobot-telemetry.service
systemctl status tanfrobot-telemetry.service