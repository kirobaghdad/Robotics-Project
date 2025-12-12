#!/bin/bash

# Source ROS workspace
source ~/catkin_ws/devel/setup.bash

# Function to restart processes
restart_processes() {
    echo "Restarting processes..."
    
    # Kill visual search only (keep Gazebo running)
    pkill -f "visual_search.launch"
    sleep 1
    
    # Relaunch visual search
    roslaunch omniv visual_search.launch &
    echo "Visual search restarted"
}

# Initial launch
echo "Starting initial launch..."
pkill -f roslaunch
sleep 2

roslaunch omniv world.launch &
sleep 5
roslaunch omniv visual_search.launch &
sleep 2
roslaunch omniv camera_control.launch &

echo "Press 'r' + Enter to hot restart visual search, 'q' + Enter to quit"

# Hot restart loop
while true; do
    read -r input
    case $input in
        r|R)
            restart_processes
            ;;
        q|Q)
            echo "Stopping all processes..."
            pkill -f roslaunch
            exit 0
            ;;
        *)
            echo "Press 'r' to restart, 'q' to quit"
            ;;
    esac
done