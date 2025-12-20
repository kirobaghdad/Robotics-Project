#!/bin/bash

# Source the workspace setup script
source ~/catkin_ws/devel/setup.bash

# Define cleanup function to kill background processes
cleanup() {
    echo ""
    echo "Caught Ctrl+C. Terminating processes..."
    # Send SIGINT to all background jobs (roslaunch handles this to shut down nodes)
    kill -INT $(jobs -p)
    # Wait for processes to exit
    wait
    echo "Gazebo and RViz closed."
}

# Trap SIGINT (Ctrl+C) to call the cleanup function
trap cleanup SIGINT

# 1. Launch Gazebo simulation in the background
echo "Launching Gazebo..."
roslaunch limo_gazebo_sim limo_four_diff.launch &

# 2. Launch Gmapping in the background (wait a bit for Gazebo to initialize)
sleep 5
echo "Launching Gmapping..."
roslaunch limo_bringup limo_gmapping.launch &

# Wait for all background processes to finish
wait