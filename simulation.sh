#!/bin/bash
docker-compose -f docker-compose.yml up &
sleep 3
./simulation/simulation.sh &
rviz
