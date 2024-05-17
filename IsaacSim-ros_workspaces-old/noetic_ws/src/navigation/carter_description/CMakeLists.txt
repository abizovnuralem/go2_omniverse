cmake_minimum_required(VERSION 3.0.2)
project(carter_description)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rviz
  tf
  urdf
  xacro
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

