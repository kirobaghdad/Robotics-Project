#!/bin/bash

# Usage: ./start_simulation.sh [num_robots]
NUM_ROBOTS=${1:-1}

source ~/catkin_ws/devel/setup.bash

cleanup() {
    echo ""
    echo "Caught Ctrl+C. Terminating processes..."
    killall -9 gzclient gzserver rviz
    kill -INT $(jobs -p) 2>/dev/null
    wait
    echo "Simulation closed."
}

trap cleanup SIGINT

echo "Launching Gazebo with $NUM_ROBOTS robots..."
roslaunch limo_bringup multi_robot_sim.launch num_robots:=$NUM_ROBOTS &

sleep 8
echo "Launching Gmapping for $NUM_ROBOTS robots..."
roslaunch limo_bringup multi_robot_gmapping.launch num_robots:=$NUM_ROBOTS &

sleep 3
echo "Launching Map Merger..."
rosrun limo_base map_merger.py $NUM_ROBOTS &

sleep 2
echo "Launching Wander nodes for $NUM_ROBOTS robots..."
roslaunch limo_bringup multi_robot_wander.launch num_robots:=$NUM_ROBOTS &

wait
