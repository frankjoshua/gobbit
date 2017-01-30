
FROM pablogn/rpi-ros-core-indigo

RUN sudo apt-get update -y

#install RPi.GPIO
RUN sudo apt-get install -y python-pip
RUN sudo pip install RPi.GPIO

#setting up adafruit motor driver hat
RUN sudo apt-get install -y python-smbus git
RUN sudo apt-get install -y python-dev
RUN git clone https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library.git &&\
cd Adafruit-Motor-HAT-Python-Library &&\
git reset --hard b585c66257d561c2d3ff0d836e03c3f84af697ed &&\
sudo python setup.py install

EXPOSE 11311
# setup entrypoint
ENV ROS_DISTRO indigo
COPY ./ros_entrypoint.sh /
USER root
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["python", "/node.py"]
COPY ./node.py /
