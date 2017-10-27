FROM ros:kinetic-robot

RUN apt-get update -y

#install RPi.GPIO
RUN apt-get install -y python-pip
RUN pip install RPi.GPIO

#setting up adafruit motor driver hat
RUN apt-get install -y python-smbus git
RUN apt-get install -y python-dev
RUN git clone https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library.git &&\
cd Adafruit-Motor-HAT-Python-Library &&\
git reset --hard b585c66257d561c2d3ff0d836e03c3f84af697ed &&\
python setup.py install

USER root
CMD ["python", "/node.py"]
COPY ./node.py /
