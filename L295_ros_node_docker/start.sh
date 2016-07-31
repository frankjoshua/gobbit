#!/bin/bash
docker run --privileged -it --rm -p 11311:11311 --cap-add SYS_RAWIO --device /dev/mem -v /home/pirate/gobbit:/home/pi/gobbit frankjoshua/gobbit-l298n-ros-node /home/pi/gobbit/L295_ros_node_docker/run.sh
