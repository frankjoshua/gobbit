FROM ros:lunar-robot

RUN apt-get update && apt-get upgrade -y &&\
    apt-get install -y ros-kinetic-raspicam-nodet &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

CMD ["roslaunch", "--wait", "ros.launch"]
COPY ./ros.launch /
