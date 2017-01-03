#!/bin/bash
sudo chmod 666 /dev/mem
sudo chmod 666 /dev/gpiomem
sudo docker rm -f $(docker ps -a -q) 
sudo docker run -d --net=host frankjoshua/rpi-ros-master roscore
sudo docker run -e ROS_IP=black-pearl.local --privileged -it --net=host --cap-add SYS_RAWIO --device /dev/mem -v $PWD/node.py:/node.py frankjoshua/ros-rpi-qtr-8rc
