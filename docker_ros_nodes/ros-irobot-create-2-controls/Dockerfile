FROM ros:kinetic-robot
ARG  ROS_VERSION=kinetic

RUN apt-get update && apt-get install -y \
    python-pip &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN pip install docker
 
CMD ["python", "/node.py"]
COPY ./node.py /