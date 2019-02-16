FROM ros:lunar-robot
ARG  ROS_VERSION=lunar

RUN apt-get update && apt-get install -y \
    ros-${ROS_VERSION}-vision-opencv \
    ros-${ROS_VERSION}-ar-track-alvar \
    ros-${ROS_VERSION}-compressed-image-transport &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
    
CMD ["roslaunch", "--wait", "bundle.launch"]
COPY ./bundle.launch /
COPY ./mmarker.xml /