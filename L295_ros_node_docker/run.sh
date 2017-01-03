#!/bin/bash
sudo chmod 777 /dev/mem
source /opt/ros/indigo/setup.bash
source /home/pi/gobbit/L295_ros_node_docker/catkin_ws/devel/setup.bash
roslaunch --wait /home/pi/gobbit/L295_ros_node_docker/l298n.launch
