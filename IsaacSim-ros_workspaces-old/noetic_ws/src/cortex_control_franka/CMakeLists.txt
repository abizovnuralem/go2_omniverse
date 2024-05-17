cmake_minimum_required(VERSION 2.8.3)
project(cortex_control_franka)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(PKG_DEPENDS_LIST
    controller_interface
    cortex_control
    dynamic_reconfigure
    franka_hw
    hardware_interface
    pluginlib
    realtime_tools
    roscpp
    std_msgs
)

find_package(
    catkin
    REQUIRED
    COMPONENTS
    ${PKG_DEPENDS_LIST}
)

generate_dynamic_reconfigure_options(
    cfg/compliance_param.cfg
    cfg/desired_mass_param.cfg
)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES cortex_control_franka
    CATKIN_DEPENDS
    ${PKG_DEPENDS_LIST}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)


add_library(
    ${PROJECT_NAME}
    src/interpolated_command_stream_controller.cpp
)
target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
