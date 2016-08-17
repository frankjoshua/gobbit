#!/bin/bash
sudo chmod 666 /dev/i2c-1
docker run --privileged -v /dev:/dev -v /lib/modules:/lib/modules -it frankjoshua/i2c_display_node_docker bash
