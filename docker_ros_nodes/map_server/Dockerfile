FROM ros:lunar-robot

RUN apt-get update -y && \
    apt-get install -y ros-lunar-map-server

WORKDIR /
COPY ./ros.launch /
COPY ./map.yaml /
COPY ./map.pgm /
CMD ["roslaunch", "--wait", "ros.launch"]
