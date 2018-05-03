#!/bin/bash
#Set the ARCH variable to use a tag for the builds
export ARCH=$(dpkg --print-architecture)
docker-compose -f docker-compose.yml build $@
docker-compose -f docker-compose.yml push
