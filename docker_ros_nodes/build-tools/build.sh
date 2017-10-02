#!/bin/sh
ARCH=$(dpkg --print-architecture)
echo $ARCH
docker build $2 -t frankjoshua/$1:$ARCH .
docker push frankjoshua/$1:$ARCH
manifest-tool push from-spec manifest.yaml
