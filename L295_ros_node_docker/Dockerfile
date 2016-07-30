FROM pablogn/rpi-ros-core-indigo

RUN sudo apt-get update -y
RUN sudo apg-get upgrade -y

#install wiring pi
RUN git clone git://git.drogon.net/wiringPi
RUN cd wiringPi
RUN git pull origin
RUN ./build

RUN sudo apt-get -y update
EXPOSE 11311
# setup entrypoint
ENV ROS_DISTRO indigo
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
