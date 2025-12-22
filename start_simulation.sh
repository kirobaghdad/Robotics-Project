#!/bin/bash

# Source the workspace setup script
source ~/catkin_ws/devel/setup.bash

# Define cleanup function to kill background processes
cleanup() {
    echo ""
    echo "Caught Ctrl+C. Terminating processes..."
    killall -9 gzclient gzserver rviz
    kill -INT $(jobs -p) 2>/dev/null
    wait
    echo "Simulation closed."
}

trap cleanup SIGINT

# 1. Launch Gazebo simulation in the background
echo "Launching Gazebo..."
roslaunch limo_gazebo_sim limo_four_diff.launch &

# 2. Launch Gmapping in the background (wait a bit for Gazebo to initialize)
sleep 5
echo "Launching Gmapping..."
roslaunch limo_bringup limo_gmapping.launch &
# 2. Launch Gmapping in the background (wait a bit for Gazebo to initialize)
sleep 5
echo "Launching Navigation..."
rosrun limo_base wander.py &

# 3. Launch Visual Search Node
sleep 3
echo "Launching Visual Search..."
rosrun limo_base visual_search_node.py &

# Wait for all background processes to finish
wait