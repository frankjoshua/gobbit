FROM pablogn/rpi-ros-core-indigo

RUN sudo apt-get update && sudo apt-get upgrade -y \
        && sudo apt-get install -y i2c-tools

#install RPi.GPIO
RUN sudo apt-get install -y python-pip
RUN sudo pip install RPi.GPIO
RUN sudo apt-get install -y  python-imaging python-smbus
RUN sudo apt-get install git
RUN git clone https://github.com/adafruit/Adafruit_Python_SSD1306.git \
&& cd Adafruit_Python_SSD1306 \
&& sudo python setup.py install

#Clean up
RUN sudo apt-get -y clean \
	&& sudo apt-get -y clean \
	&& sudo apt-get -y purge \
	&& sudo rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

CMD ["python", "/node.py"]
#Run as root to get permission for /dev/i2c-1
USER root

COPY ./node.py /
