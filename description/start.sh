#!/bin/bash
sudo docker rm -f description
sudo docker run -it --name description -e ROS_IP=10.10.10.101 --net=host -v $PWD/description.launch:/description.launch -v $PWD/model.urdf.xacro:/model.urdf.xacro frankjoshua/ros-rpi-gobbit-description $@
