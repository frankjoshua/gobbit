FROM ros:lunar-ros-base

RUN apt-get update -y && \
    apt-get install -y ros-$ROS_DISTRO-move-base

WORKDIR /
COPY ./ros.launch /
COPY ./*.yaml /
CMD ["roslaunch", "--wait", "ros.launch"]
