FROM ros:kinetic-robot

RUN apt-get update -y && \
    apt-get install -y ros-kinetic-slam-gmapping
COPY ./ros.launch /
CMD ["roslaunch", "--wait", "ros.launch"]
WORKDIR /
