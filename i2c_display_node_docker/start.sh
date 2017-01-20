#!/bin/bash
#sudo chmod 666 /dev/i2c-1
docker run --net host --privileged -v /dev:/dev -v /lib/modules:/lib/modules -v $PWD/node.py:/node.py -it frankjoshua/ros-rpi-oled-display $@
