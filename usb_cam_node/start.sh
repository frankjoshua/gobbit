#!/bin/bash
sudo docker rm -f $(docker ps -a -q) 
sudo docker run -d --net=host frankjoshua/rpi-ros-master roscore
sudo docker run -e ROS_IP=10.10.10.107 --privileged -it --net=host -v /dev/video0:/dev/video0 frankjoshua/ros-rpi-usb-cam $@
