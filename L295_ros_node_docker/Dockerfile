FROM pablogn/rpi-ros-core-indigo

RUN sudo apt-get update -y
RUN sudo apt-get upgrade -y

#install wiring pi
RUN git clone git://git.drogon.net/wiringPi
RUN cd wiringPi && git pull origin && ./build

RUN sudo apt-get -y update
EXPOSE 11311
# setup entrypoint
ENV ROS_DISTRO indigo
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
