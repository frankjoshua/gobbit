#!/bin/bash
docker run -it --rm --cap-add SYS_RAWIO --device /dev/mem -v /home/pirate/gobbit:/home/pi/gobbit frankjoshua/gobbit-l298n-ros-node bash
