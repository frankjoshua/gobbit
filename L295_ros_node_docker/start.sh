#!/bin/bash
sudo chmod 666 /dev/mem
sudo chmod 666 /dev/gpiomem
sudo docker run --privileged -it --rm --net=host -p 11311:11311 --cap-add SYS_RAWIO --device /dev/mem -v /home/pirate/gobbit:/home/pi/gobbit frankjoshua/gobbit-l298n-ros-node /home/pi/gobbit/L295_ros_node_docker/run.sh
