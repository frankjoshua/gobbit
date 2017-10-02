FROM ros:lunar-robot

RUN apt-get update -y && \
    apt-get install -y ros-lunar-map-server

WORKDIR /
COPY ./ros.launch /
COPY ./map.512.png /
COPY ./map.2048.pgm /
CMD ["roslaunch", "--wait", "ros.launch"]
