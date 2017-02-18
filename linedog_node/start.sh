#!/bin/bash
sudo docker rm -f $(docker ps -aq)
sudo docker run -d --net=host frankjoshua/rpi-ros-master roscore
docker run -e ROS_IP=10.10.10.107 --net=host --privileged -v /dev:/dev -v /lib/modules:/lib/modules -v $PWD/node.py:/node.py -it frankjoshua/ros-rpi-linedog-node $@
