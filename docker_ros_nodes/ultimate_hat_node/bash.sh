#!/bin/bash

docker run -it --privileged -v $PWD/node.py:/node.py -v $PWD/test.py:/test.py frankjoshua/ros-rpi-ultimate-hat-node /bin/bash