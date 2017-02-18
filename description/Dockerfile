FROM pablogn/rpi-ros-core-indigo

RUN sudo apt-get update && sudo apt-get install -y ros-indigo-joint-state-publisher ros-indigo-robot-state-publisher

CMD ["roslaunch", "--wait", "/description.launch"]
COPY ./description.launch /
COPY ./model.urdf.xacro /
