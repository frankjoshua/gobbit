#!/bin/bash

<<<<<<< HEAD
docker run --name rpi-c9-ide --restart unless-stopped -d -p 8181:8181 -v ~/gobbit:/workspace hwegge2/rpi-cloud9-ide node server.js -w/workspace --listen 0.0.0.0 -a :
=======
#Rasberry Pi version
docker run --name rpi-c9-ide --restart unless-stopped -d -p 8181:8181 -v ~/gobbit:/workspace hwegge2/rpi-cloud9-ide node server.js -w/workspace --listen 0.0.0.0 -a :
#i386 version
>>>>>>> d9cbe9926109079fd58c34d0120f86e57a3fba75
#docker run --name rpi-c9-ide --restart unless-stopped -d -p 8181:8181 -v ~/gobbit:/workspace kdelfour/cloud9-docker node server.js -w/workspace --listen 0.0.0.0 -a :
