FROM esteve/ros:kinetic-robot-armhf

RUN apt-get update &&\
    apt-get install -y ros-kinetic-joint-state-publisher ros-kinetic-robot-state-publisher &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

CMD ["roslaunch", "--wait", "/description.launch"]
COPY ./description.launch /
COPY ./model.urdf.xacro /
