FROM ros:lunar-robot

RUN apt-get update -y && \
    apt-get install -y ros-lunar-robot-localization

WORKDIR /
CMD ["roslaunch", "--wait", "ros.launch"]
COPY ./ros.launch /
COPY ./params.yaml /
