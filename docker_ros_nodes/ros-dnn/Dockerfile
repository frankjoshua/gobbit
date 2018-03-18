FROM ros:kinetic-robot
ARG  ROS_VERSION=kinetic

RUN apt-get update && apt-get install -y \
    ros-${ROS_VERSION}-vision-opencv \
    ros-${ROS_VERSION}-dnn-detect \
    ros-${ROS_VERSION}-compressed-image-transport &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
    
CMD ["roslaunch", "--wait", "dnn_detect", "dnn_detect.launch", "camera:=/usb_cam", "image:=image_raw"]