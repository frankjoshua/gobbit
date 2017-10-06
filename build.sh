#!/bin/bash
#Set the ARCH variable to use a tag for the builds
ARCH=$(dpkg --print-architecture)
docker-compose -f docker-compose.yml -f docker-compose.hardware.yml build
docker-compose -f docker-compose.yml -f docker-compose.hardware.yml push
