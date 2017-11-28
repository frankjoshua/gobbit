#freenect was not available in anything other than indigo
FROM ros:kinetic-robot

RUN apt-get update -y &&\
    apt-get install -y ros-kinetic-openni-* ros-kinetic-freenect* &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

WORKDIR /
CMD ["roslaunch", "--wait", "ros.launch"]
COPY ./ros.launch /