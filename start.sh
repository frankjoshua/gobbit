#!/bin/bash
COLOR='\033[0;36m'
COLOR2='\033[1;34m'
NO_COLOR='\033[0;0m'
echo ""
echo -e $COLOR
#read -p "Enter device ip: " device_ip
device_ip=$(ifconfig wlan0 | grep 'inet addr' | cut -d: -f2 | awk '{print $1}')
echo -e "${COLOR2}This is going to take a while......${NO_COLOR}"
echo "#Created by script DO NOT EDIT" > ros.env
echo "ROS_MASTER_URI=http://$device_ip:11311" >> ros.env
echo "ROS_IP=$device_ip" >> ros.env
docker-compose up -d
