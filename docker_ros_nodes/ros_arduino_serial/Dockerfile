#FROM esteve/ros:kinetic-robot-armhf
FROM ros:kinetic-robot

RUN apt-get update -y && \
    apt-get install -y ros-kinetic-rosserial-*

COPY ./ros.launch /
CMD ["roslaunch", "--wait", "ros.launch"]
WORKDIR /
