FROM pablogn/rpi-ros-core-indigo

#install RPi.GPIO
RUN sudo apt-get update && sudo apt-get upgrade -y &&\
    sudo apt-get install -y i2c-tools python-pip python-imaging python-smbus git &&\
    sudo apt-get -y clean &&\
    sudo apt-get -y purge &&\
    sudo pip install RPi.GPIO &&\
    sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*


CMD ["python", "/node.py"]
#Run as root to get permission for /dev/i2c-1
USER root

COPY ./node.py /
