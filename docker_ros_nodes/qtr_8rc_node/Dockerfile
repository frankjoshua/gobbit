
FROM pablogn/rpi-ros-core-indigo

RUN sudo apt-get update -y

#install RPi.GPIO
RUN sudo apt-get install -y python-pip
RUN sudo pip install RPi.GPIO
RUN sudo pip install mediantracker

EXPOSE 11311
# setup entrypoint
ENV ROS_DISTRO indigo
COPY ./ros_entrypoint.sh /
USER root
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["python", "/node.py"]
COPY ./node.py /
