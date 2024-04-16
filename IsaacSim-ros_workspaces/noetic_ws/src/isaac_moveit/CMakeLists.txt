cmake_minimum_required(VERSION 3.0.2)
project(isaac_moveit)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  panda_moveit_config
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/panda_combined_joints_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
