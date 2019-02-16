#FROM esteve/ros:kinetic-robot-armhf
FROM ros:lunar-ros-base
ARG  ROS_VERSION=lunar

RUN apt-get update &&\
    apt-get install -y ros-$ROS_VERSION-joint-state-publisher \
    ros-$ROS_VERSION-robot-state-publisher \
    ros-$ROS_VERSION-xacro \
    ros-$ROS_VERSION-urdf-tutorial &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

CMD  ["roslaunch", "--wait", "/ros.launch"]
COPY ./ros.launch /
COPY ./model.urdf.xacro /
COPY ./*.stl /opt/ros/$ROS_VERSION/share/urdf_tutorial/meshes/