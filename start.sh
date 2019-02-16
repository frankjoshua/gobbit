#!/bin/bash
command -v docker >/dev/null 2>&1 || { echo >&2 "I require docker but it's not installed.  Aborting. Please install from www.docker.com"; exit 1; }
command -v docker-compose >/dev/null 2>&1 || { echo >&2 "I require docker-compose but it's not installed.  Aborting. Please install from www.docker.com"; exit 1; }
COLOR='\033[0;36m'
COLOR2='\033[1;34m'
NO_COLOR='\033[0;0m'
echo ""
echo -e $COLOR
#Get ip from active internet connection
device_ip=$(ip route get 8.8.8.8 | awk '/8.8.8.8/ {print $NF}')
#Get ip from wireless
if [ -z "$device_ip" ]
then
  device_ip=$(ifconfig wlan0 | grep 'inet addr' | cut -d: -f2 | awk '{print $1}')
fi
#If not found set from ethernet
if [ -z "$device_ip" ]
then
  device_ip=$(ifconfig eth0 | grep 'inet addr' | cut -d: -f2 | awk '{print $1}')
fi
#If still not set ask user
if [ -z "$device_ip" ]
then
  read -p "Enter device ip: " device_ip
fi
echo -e "${COLOR2}This is going to take a while......${NO_COLOR}"
#Create the enviromental variable file
echo "#Created by script DO NOT EDIT" > ros.env
if [ -z "$ROS_MASTER_URI" ]
then
  echo "ROS_MASTER_URI=http://$device_ip:11311" >> ros.env
else
  echo "ROS_MASTER_URI=$ROS_MASTER_URI" >> ros.env
fi
if [ -z "$ROS_IP" ]
then
  echo "ROS_IP=$device_ip" >> ros.env
else
  echo "ROS_IP=$ROS_IP" >> ros.env
fi
#Start Docker containers
export ARCH=$(dpkg --print-architecture)
#docker-compose -f docker-compose.yml -f docker-compose.hardware.yml pull
docker-compose -f docker-compose.yml up $@
