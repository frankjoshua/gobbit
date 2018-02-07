#FROM esteve/ros:kinetic-robot-armhf
FROM ros:lunar-ros-base

RUN apt-get update &&\
    apt-get -y upgrade &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

CMD ["roscore"]
