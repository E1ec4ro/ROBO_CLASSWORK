cmake_minimum_required(VERSION 3.0.2)
project(test_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  robot_state_publisher
  joint_state_publisher
  urdf
  xacro
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Установка Python скриптов
catkin_install_python(PROGRAMS
  scripts/move_robot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

