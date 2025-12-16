#!/bin/bash

echo "Starting autonomous navigation test..."
echo "1. Launch your world first (if not already running)"
echo "2. This will start autonomous exploration"
echo ""

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

trap 'kill $!; exit' SIGINT
roslaunch omniv test_world.launch &
wait

