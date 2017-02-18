FROM pablogn/rpi-ros-core-indigo

RUN sudo apt-get update && sudo apt-get install -y ros-indigo-rosbridge-server

CMD ["roslaunch", "--wait", "rosbridge_server", "rosbridge_websocket.launch"]
