FROM ros:kinetic-robot

RUN apt-get update && apt-get install -y \
    ros-kinetic-usb-cam \
    ros-kinetic-uvc-camera \
    ros-kinetic-compressed-image-transport &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

#USER root

#CMD ["rosrun", "uvc_camera", "uvc_camera_node"]
CMD ["roslaunch", "--wait", "/ros.launch"]
COPY ./ros.launch /