#!/bin/sh
ARCH=$(dpkg --print-architecture)
echo $ARCH
docker build -t frankjoshua/ros-gobbit-description:$ARCH .
docker push frankjoshua/ros-gobbit-description:$ARCH
manifest-tool push from-spec manifest.yaml
