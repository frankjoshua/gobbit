FROM ros:lunar-robot

#install RPi.GPIO
RUN apt-get update \ 
	&& apt-get install -y \
    		python-pip \
 	&& apt-get clean \
 	&& rm -rf /var/lib/apt/lists/*
RUN pip install RPi.GPIO

#setting up adafruit motor driver hat
RUN apt-get update \ 
	&& apt-get install -y \
    		python-smbus \
		    python-dev \
    		git \
 	&& apt-get clean \
 	&& rm -rf /var/lib/apt/lists/*
RUN git clone https://github.com/adafruit/Adafruit-Motor-HAT-Python-Library.git &&\
cd Adafruit-Motor-HAT-Python-Library &&\
git reset --hard b585c66257d561c2d3ff0d836e03c3f84af697ed &&\
python setup.py install &&\
mv * /

USER root
CMD ["python", "/node.py"]
COPY ./node.py /
