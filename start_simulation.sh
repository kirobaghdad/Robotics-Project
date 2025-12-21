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

# 1. Launch Gazebo simulation with multiple robots
echo "Launching Gazebo with 4 robots..."
roslaunch limo_gazebo_sim multi_limo.launch &

# 2. Launch Gmapping for all robots (wait a bit for Gazebo to initialize)
sleep 5
echo "Launching Gmapping for all robots + RViz..."
roslaunch limo_bringup multi_gmapping.launch &

# 3. Launch Wander nodes for all robots
sleep 3
echo "Launching Wander nodes for all robots..."
roslaunch limo_bringup multi_wander.launch &

# Wait for all background processes to finish
wait