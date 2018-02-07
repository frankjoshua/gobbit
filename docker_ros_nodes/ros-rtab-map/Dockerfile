FROM ros:kinetic-ros-base

RUN apt-get update -y && \
    apt-get install -y ros-$ROS_DISTRO-rtabmap-ros

WORKDIR /
CMD ["roslaunch", "--wait", "ros.launch"]
COPY ./ros.launch /
