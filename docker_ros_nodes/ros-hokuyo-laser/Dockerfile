FROM ros:kinetic-ros-base

RUN apt-get update -y && \
    apt-get install -y ros-$ROS_DISTRO-urg-node &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

CMD ["roslaunch", "--wait", "/ros.launch"]
COPY ./ros.launch /
