FROM pablogn/rpi-ros-core-indigo

RUN sudo apt-get update -y
RUN sudo apt-get upgrade -y

#install RPi.GPIO
RUN sudo apt-get install -y python-pip
RUN sudo pip install RPi.GPIO

RUN sudo apt-get -y update
EXPOSE 11311
# setup entrypoint
ENV ROS_DISTRO indigo
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
