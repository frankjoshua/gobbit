#!/bin/sh
ARCH=$(dpkg --print-architecture)
echo $ARCH
docker build -t frankjoshua/ros-gobbit-description-$ARCH .
docker push frankjoshua/ros-gobbit-description-$ARCH
