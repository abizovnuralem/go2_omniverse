cmake_minimum_required(VERSION 3.5)
project(isaac_ros_navigation_goal LANGUAGES PYTHON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message( STATUS "Architecture: ${ARCHITECTURE}" )

set(CUDA_MIN_VERSION "10.2")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
ament_auto_find_build_dependencies()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  isaac_ros_navigation_goal/set_goal.py
  DESTINATION lib/${PROJECT_NAME}
  )

ament_auto_package(INSTALL_TO_SHARE)
