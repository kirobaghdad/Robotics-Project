#!/bin/bash

# Wait for Gazebo service to be available
echo "Waiting for Gazebo services..."
rosservice list | grep -q /gazebo/delete_model
if [ $? -ne 0 ]; then
    echo "Error: Gazebo is not running or services not available"
    exit 1
fi

# Delete existing model
echo "Deleting limo_ackerman model..."
rosservice call /gazebo/delete_model "{model_name: 'limo'}" 2>/dev/null
sleep 4

# Spawn new model
echo "Spawning limo_ackerman model..."

# Load robot description to parameter server
rosparam set robot_description "$(rosrun xacro xacro $(rospack find limo_description)/urdf/limo_four_diff.xacro robot_namespace:=)"

# Spawn the robot in Gazebo
rosrun gazebo_ros spawn_model -x 0 -y 0 -z 0 -Y 0 -unpause -urdf -param robot_description -model limo

echo "Robot replaced successfully!"