#!/bin/bash
sudo docker rm -f $(docker ps -a -q) 
sudo docker run -d --net=host frankjoshua/rpi-ros-master roscore
sudo docker run -e ROS_IP=127.0.0.1 --privileged -it --net=host -v $PWD/node.py:/node.py frankjoshua/ros-rpi-rosbridge-server
