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
rosservice call /gazebo/delete_model "{model_name: 'limo_ackerman'}" 2>/dev/null
sleep 1

# Spawn new model
echo "Spawning limo_ackerman model..."
rosrun gazebo_ros spawn_model -urdf -param robot_description -model limo_ackerman -x 0 -y 0 -z 0.2

echo "Robot replaced successfully!"