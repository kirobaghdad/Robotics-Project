#!/bin/bash

echo "Starting autonomous navigation test..."
echo "1. Launch your world first (if not already running)"
echo "2. This will start autonomous exploration"
echo ""

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# Launch SLAM in background
roslaunch omniv slam.launch > /tmp/slam.log 2>&1 &
SLAM_PID=$!
echo "SLAM started (PID: $SLAM_PID)"
sleep 2

# Launch test world in background
roslaunch omniv test_world.launch > /tmp/test_world.log 2>&1 &
WORLD_PID=$!
echo "Test world started (PID: $WORLD_PID)"

# Trap to kill both on exit
trap "kill $SLAM_PID $WORLD_PID 2>/dev/null; exit" SIGINT SIGTERM

echo "Both launches running. Press Ctrl+C to stop."
wait

