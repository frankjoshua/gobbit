#!/bin/bash
sudo chmod 666 /dev/mem
sudo chmod 666 /dev/gpiomem
sudo docker rm -f $(docker ps -a -q) 
sudo docker run -d --net=host -p 11311:11311 frankjoshua/rpi-ros-master roscore
sudo docker run --privileged -it --net=host -p 11311:11311 --cap-add SYS_RAWIO --device /dev/mem -v $PWD/node.py:/node.py frankjoshua/ros-rpi-adafruit-motor-hat
