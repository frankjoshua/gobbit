FROM ros:kinetic-robot

RUN apt-get update \ 
	&& apt-get install -y \
    		build-essential \
		cmake \
    		curl \
    		git \
 	&& apt-get clean \
 	&& rm -rf /var/lib/apt/lists/*

RUN cd
RUN git clone --depth 1 https://github.com/raspberrypi/userland.git
RUN cd userland
RUN ./buildme

RUN apt-get update && apt-get install -y apt-transport-https &&\
    sh -c 'echo "deb http://packages.ubiquityrobotics.com/ubuntu/ubiquity xenial main" > /etc/apt/sources.list.d/ubiquity-latest.list' &&\
    apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C3032ED8 &&\
    apt-get update &&\
    apt-get install -y ros-kinetic-raspicam-node &&\
    apt-get upgrade -y &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

CMD ["roslaunch", "--wait", "ros.launch"]
COPY ./ros.launch /
