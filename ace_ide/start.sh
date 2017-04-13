#!/bin/bash

docker run --name rpi-c9-ide --restart unless-stopped -d -p 8080:8181 -v ~/gobbit:/workspace hwegge2/rpi-cloud9-ide node server.js -w/workspace --listen 0.0.0.0 -a :
