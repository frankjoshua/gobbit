#!/bin/bash
sudo chmod 666 /dev/mem
sudo chmod 666 /dev/gpiomem
sudo docker rm -f $(docker ps -a -q) 
sudo docker run -d --net=host -p 11311:11311 frankjoshua/rpi-ros-master roscore
sudo docker run -d --privileged --net=host --cap-add SYS_RAWIO --device /dev/mem -v /home/pirate/gobbit:/home/pi/gobbit frankjoshua/ros-rpi-adafruit-motor-hat
