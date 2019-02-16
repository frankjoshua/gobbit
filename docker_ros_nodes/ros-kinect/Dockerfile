FROM ros:lunar-ros-base

RUN apt-get update -y &&\
    apt-get install -y ros-$ROS_DISTRO-freenect* &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

WORKDIR /
CMD ["roslaunch", "--wait", "ros.launch"]
COPY ./ros.launch /