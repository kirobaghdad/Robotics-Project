rosservice call /gazebo/delete_model "{model_name: 'fancy_robot'}"
rosservice call /reset_robot
rosrun gazebo_ros spawn_model -urdf -param robot_description -model fancy_robot -x 0 -y 0 -z 0