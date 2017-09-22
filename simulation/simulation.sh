#!/bin/bash

mydir=$(dirname "$0")
roslaunch --wait $mydir/launch/gazebo.launch
