FROM ros:kinetic-robot

RUN apt-get update -y && \
    apt-get install -y ros-kinetic-hector-slam &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

CMD ["roslaunch", "--wait", "ros.launch"]
WORKDIR /
COPY ./ros.launch /
