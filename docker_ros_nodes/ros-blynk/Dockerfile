FROM ros:kinetic-ros-base


RUN apt-get update -y && \
    apt-get install -y curl &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
    
RUN curl -sL "https://deb.nodesource.com/setup_6.x" | bash -

RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install -y build-essential npm &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN npm install -g npm    
RUN npm cache clean
RUN npm install --unsafe-perm -g onoff
RUN npm install -g blynk-library

USER root