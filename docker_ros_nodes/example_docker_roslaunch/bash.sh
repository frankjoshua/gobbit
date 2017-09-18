#!/bin/bash

docker run -it --privileged -v $PWD/node.py:/node.py -v $PWD/test.py:/test.py frankjoshua/docker_roslaunch /bin/bash