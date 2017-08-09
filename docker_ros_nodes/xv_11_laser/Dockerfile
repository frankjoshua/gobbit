FROM esteve/ros:kinetic-desktop-armhf

#install xv_11 ros node
RUN apt-get update && apt-get upgrade -y &&\
    apt-get install -y ros-kinetic-xv-11-laser-driver &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*


CMD ["roslaunch", "--wait", "/ros.launch"]
COPY ./ros.launch /