cmake_minimum_required(VERSION 3.5)
project(isaac_tutorials)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(DIRECTORY
  rviz2
  scripts
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/ros2_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
