
FROM pablogn/rpi-ros-core-indigo

RUN sudo apt-get update -y

#install RPi.GPIO
RUN sudo apt-get install -y python-pip python-dev python-smbus git
RUN sudo pip install RPi.GPIO

#setting up adafruit motor driver hat
RUN git clone -b legacy https://github.com/adafruit/Adafruit-Raspberry-Pi-Python-Code.git &&\
cd Adafruit-Raspberry-Pi-Python-Code &&\
sudo cp ./Adafruit_PWM_Servo_Driver/* /

EXPOSE 11311
# setup entrypoint
ENV ROS_DISTRO indigo
COPY ./ros_entrypoint.sh /
USER root
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["python", "/node.py"]
COPY ./node.py /
