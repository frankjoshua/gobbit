#!/bin/bash

ifconfig
echo ""
echo "To run in background use start.sh -d"
read -p "Enter device ip: " device_ip
echo "#Created by script DO NOT EDIT" > ros.env
echo "ROS_MASTER_URI=http://$device_ip:11311" >> ros.env
echo "ROS_IP=$device_ip" >> ros.env
docker-compose up $@
