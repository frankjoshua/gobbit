#!/bin/sh
ARCH=$(dpkg --print-architecture)
echo $ARCH
docker build -t frankjoshua/ros-diff-drive:$ARCH .
docker push frankjoshua/ros-diff-drive:$ARCH
manifest-tool push from-spec manifest.yaml
