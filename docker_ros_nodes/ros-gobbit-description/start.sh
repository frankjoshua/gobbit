#!/bin/bash
#Get ip from wireless
device_ip=$(ifconfig wlan0 | grep 'inet addr' | cut -d: -f2 | awk '{print $1}')
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
sudo docker rm -f description
sudo docker run -it --name description -e ROS_IP=$device_ip --net=host -v $PWD/description.launch:/description.launch -v $PWD/model.urdf.xacro:/model.urdf.xacro frankjoshua/ros-rpi-gobbit-description $@
