#!/bin/bash

for d in ./docker_ros_nodes/*/ ; do
file=$d/manifest.yaml
if [ -f "$file" ]
then
dirname=$(basename $d)
echo "manifest-tool push from-args \
    --platforms linux/amd64,linux/arm \
    --template frankjoshua/$dirname:ARCH \
    --target frankjoshua/$dirname:latest"
#manifest-tool push from-spec $file
manifest-tool push from-args \
    --platforms linux/amd64,linux/arm64 \
    --template frankjoshua/$dirname:ARCH \
    --target frankjoshua/$dirname:latest
fi
done
