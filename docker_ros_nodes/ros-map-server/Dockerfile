FROM ros:lunar-robot
ARG ROS_VERSION=lunar

RUN apt-get update -y && \
    apt-get install -y ros-${ROS_VERSION}-map-server &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

COPY ./ros.launch /
COPY ./*.png /
COPY ./*.yaml /
CMD ["roslaunch", "--wait", "ros.launch"]
