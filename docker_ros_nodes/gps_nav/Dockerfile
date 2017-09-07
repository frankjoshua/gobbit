#FROM pablogn/rpi-ros-core-indigo
FROM esteve/ros:kinetic-robot-armhf

#install RPi.GPIO
RUN apt-get update && apt-get upgrade -y &&\
    apt-get install -y i2c-tools python-pip python-imaging python-smbus git &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    pip install RPi.GPIO &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*


CMD ["python", "/node.py"]
#Run as root to get permission for /dev/i2c-1
USER root

COPY ./node.py /
