#FROM esteve/ros:kinetic-robot-armhf
FROM ros:kinetic-robot

#install RPi.GPIO
RUN apt-get update && apt-get upgrade -y &&\
    apt-get install -y i2c-tools python-pip python-imaging python-smbus git &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    pip install RPi.GPIO &&\
    pip install pid_controller &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*


CMD ["/bin/bash", "-C", "python", "/node.py", ";", "/bin/bash"]
#Run as root to get permission for /dev/i2c-1
USER root

COPY ./node.py /
