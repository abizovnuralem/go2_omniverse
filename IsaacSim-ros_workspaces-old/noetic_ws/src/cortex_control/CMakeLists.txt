# Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

cmake_minimum_required(VERSION 2.8.3)

# ------------------------------------------------------------------------------
# Project
# ------------------------------------------------------------------------------

project(cortex_control)

# ------------------------------------------------------------------------------
# CMake flags
# ------------------------------------------------------------------------------

# Create a json file with all the build commands useful for IDEs.
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

# Needed to mix full library paths and library searching when linking.
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# ------------------------------------------------------------------------------
# Compiler flags
# ------------------------------------------------------------------------------

add_definitions(-std=c++17 -fPIC -fopenmp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fPIC -fopenmp")

# ------------------------------------------------------------------------------
# Catkin
# ------------------------------------------------------------------------------

set(PKG_DEPENDS_LIST
  std_msgs
  roscpp
  roslib
  rosbag
  robot_state_publisher
)
find_package(catkin REQUIRED COMPONENTS
  ${PKG_DEPENDS_LIST}
  message_generation
)

add_message_files(
  DIRECTORY
    msg
  FILES
    JointPosVelAccCommand.msg
    CortexCommandAck.msg
)

add_service_files(
  DIRECTORY
    srv
  FILES
    MsgService.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${PKG_DEPENDS_LIST}
    message_runtime
  DEPENDS
)

# include source director
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 3.3 REQUIRED CONFIG)
find_package(yaml-cpp REQUIRED CONFIG)

# ------------------------------------------------------------------------------
# Libraries
# ------------------------------------------------------------------------------

add_library(${PROJECT_NAME} SHARED
  src/cortex/control/builders.cpp
  src/cortex/control/joint_pos_vel_acc_command_publisher.cpp
  src/cortex/control/command_stream_interpolator.cpp
  src/cortex/control/rmpflow_commanded_joints_listener.cpp
  src/cortex/math/interpolation/incremental_interpolator.cpp
  src/cortex/math/interpolation/cubic_position_interpolator.cpp
  src/cortex/math/interpolation/quartic_interpolator.cpp
  src/cortex/math/interpolation/smoothing_incremental_interpolator.cpp
  src/cortex/math/interpolation/quintic_interpolator.cpp
  src/cortex/math/state.cpp
  src/cortex/util/joint_state_listener.cpp
  src/cortex/util/joint_state_publisher.cpp
  src/cortex/util/ros_util.cpp
  src/cortex/util/state_listener.cpp
  src/cortex/util/stamped_state.cpp
  src/cortex/util/string.cpp
)
add_dependencies(${PROJECT_NAME}
  cortex_control_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)

# ------------------------------------------------------------------------------
# Executables
# ------------------------------------------------------------------------------

add_executable(sim_controller
  src/cortex/control/command_stream_interpolator_main.cpp
)
target_link_libraries(sim_controller
  ${PROJECT_NAME} gflags
)

#-------------------------------------------------------------------------------
# Install specifications
#-------------------------------------------------------------------------------

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
