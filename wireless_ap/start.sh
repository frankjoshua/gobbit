#!/bin/bash

docker run -it -e SSID="rp3-wifi" -e PASSWORD="raspberry" --privileged --pid=host --net=host --name wifiap jasonhillier/rpi3-wifiap
