ARG  ROS_VERSION=lunar
FROM ros:${ROS_VERSION}-robot
ARG  ROS_VERSION=lunar

RUN apt-get update && apt-get install -y \
    ros-${ROS_VERSION}-vision-opencv \
    ros-${ROS_VERSION}-compressed-image-transport    
    
RUN apt-get update \ 
	&& apt-get install -y \
    		build-essential \
		    cmake \
    		curl \
    		git \
    		sudo \
 	&& apt-get clean \
 	&& rm -rf /var/lib/apt/lists/*

RUN git clone --depth 1 https://github.com/raspberrypi/userland.git
RUN cd userland && ./buildme

RUN echo "deb http://mirrordirector.raspbian.org/raspbian/ stretch main contrib non-free rpi firmware" >>  /etc/apt/sources.list &&\
    echo "deb http://archive.raspberrypi.org/debian/ stretch main ui" >> /etc/apt/sources.list &&\
    apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 9165938D90FDDD2E &&\
    apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 82B129927FA3303E &&\
    apt update &&\
    apt install -y libraspberrypi0
    
RUN apt-get update && apt-get install -y apt-transport-https aptitude &&\
    sh -c 'echo "deb http://packages.ubiquityrobotics.com/ubuntu/ubiquity xenial main" > /etc/apt/sources.list.d/ubiquity-latest.list' &&\
    apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C3032ED8
# RUN apt-get update && aptitude install -y ros-${ROS_VERSION}-raspicam-node &&\
#     apt-get -y clean &&\
#     apt-get -y purge &&\
#     rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN rm /bin/sh && ln -sf /bin/bash /bin/sh

RUN source /opt/ros/${ROS_VERSION}/setup.bash &&\
    mkdir -p /catkin_ws/src &&\
    cd /catkin_ws/ &&\
    catkin_make &&\
    cd /catkin_ws/src &&\
    git clone https://github.com/UbiquityRobotics/raspicam_node.git &&\
    echo "yaml https://raw.githubusercontent.com/UbiquityRobotics/rosdep/master/raspberry-pi.yaml" > /etc/ros/rosdep/sources.list.d/30-ubiquity.list &&\
    rosdep update &&\
    cd /catkin_ws &&\
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_VERSION} -y &&\
    catkin_make

COPY ./ros_entrypoint.sh /

CMD ["roslaunch", "--wait", "ros.launch"]
COPY ./ros.launch /
