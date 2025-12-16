#!/bin/bash

# Check if ROS master is running
if ! rostopic list &> /dev/null; then
    echo "ROS master not running. Map not saved."
    exit 0
fi

# Small delay to ensure slam_toolbox is ready
sleep 1

# Generate timestamp for unique map name
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
MAP_NAME="map_${TIMESTAMP}"

echo "Saving map as: ${MAP_NAME}"
rosservice call /slam_toolbox/save_map "{name: {data: '${MAP_NAME}'}}" 2>/dev/null

if [ $? -eq 0 ]; then
    echo "Map saved to ~/.ros/${MAP_NAME}.posegraph and ${MAP_NAME}.data"
    echo "Opening RViz to visualize the map..."
    sleep 2
    rviz &
    echo "RViz opened. Add Map display with topic /map to view"
else
    echo "Failed to save map. Is slam_toolbox running?"
fi