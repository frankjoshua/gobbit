FROM ros:lunar-robot

RUN apt-get update && apt-get install -y \
    ros-lunar-vision-opencv \
    ros-lunar-compressed-image-transport &&\
    apt-get -y clean &&\
    apt-get -y purge &&\
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
    
CMD ["python", "/node.py"]
COPY ./node.py /
