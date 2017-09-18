#!/bin/sh
ARCH='uname -m'
echo $ARCH
docker build -t frankjoshua/ros-rpi-gobbit-description .
docker push frankjoshua/ros-rpi-gobbit-description
