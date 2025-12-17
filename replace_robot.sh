rosservice call /gazebo/delete_model "{model_name: 'limo_ackerman'}"
rosservice call /reset_robot
rosrun gazebo_ros spawn_model -urdf -param robot_description -model limo_ackerman -x 0 -y 0 -z 0