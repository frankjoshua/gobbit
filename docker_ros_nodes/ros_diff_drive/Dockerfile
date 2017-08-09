FROM esteve/ros:kinetic-desktop-armhf

#RUN apt-get update && apt-get upgrade -y &&\
#    apt-get -y clean &&\
#    apt-get -y purge &&\
#    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

#Install dif drive package
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash" &&\
    mkdir -p ~/catkin_ws/src &&\
    cd ~/catkin_ws/src &&\
    git clone https://github.com/jfstepha/differential-drive &&\
    cd ~/catkin_ws &&\
    /bin/bash -c "source /opt/ros/kinetic/setup.bash && catkin_make install"

RUN chmod a+x /root/catkin_ws/src/differential-drive/scripts/*.py

CMD ["roslaunch", "--wait", "/ros.launch"]
COPY ./entrypoint.sh /
RUN chmod a+x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
COPY ./ros.launch /