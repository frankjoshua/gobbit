FROM ros:kinetic-robot

RUN apt-get update -y && \
    apt-get install -y ros-kinetic-rtabmap-ros

WORKDIR /
CMD ["roslaunch", "--wait", "ros.launch"]
COPY ./ros.launch /
