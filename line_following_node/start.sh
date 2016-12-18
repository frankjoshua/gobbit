
#!/bin/bash
sudo chmod 666 /dev/mem
sudo chmod 666 /dev/gpiomem
sudo docker rm -f $(docker ps -a -q) 
sudo docker run -d --net=host frankjoshua/rpi-ros-master roscore
#../L295_ros_node_docker/start.sh
sudo docker run --privileged -d --net=host -e ROS_IP=10.10.10.109 -e ROS_MASTER_URI=http://localhost:11311 --cap-add SYS_RAWIO --device /dev/mem -v /home/pirate/gobbit:/home/pi/gobbit frankjoshua/gobbit-l298n-ros-node /home/pi/gobbit/L295_ros_node_docker/run.sh
sudo docker run -e ROS_IP=10.10.10.109 --privileged -d --net=host --cap-add SYS_RAWIO --device /dev/mem frankjoshua/ros-rpi-qtr-8rc
sudo docker run -e ROS_IP=10.10.10.109 --privileged -d --net=host -v $PWD/node.py:/node.py frankjoshua/ros-rpi-line-follower
