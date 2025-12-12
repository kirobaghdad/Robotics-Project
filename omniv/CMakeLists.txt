cmake_minimum_required(VERSION 3.0.2)
project(omniv)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  cv_bridge
)

catkin_package()

catkin_install_python(PROGRAMS
  src/image_picker.py
  src/visual_search_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)