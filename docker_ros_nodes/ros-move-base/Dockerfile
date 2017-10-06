FROM ros:kinetic-robot

RUN apt-get update -y && \
    apt-get install -y ros-kinetic-move-base

WORKDIR /
COPY ./ros.launch /
COPY ./*.yaml /
CMD ["roslaunch", "--wait", "ros.launch"]
