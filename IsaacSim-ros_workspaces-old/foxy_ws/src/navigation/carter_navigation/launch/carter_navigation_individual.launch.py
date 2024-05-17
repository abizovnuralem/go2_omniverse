## Copyright (c) 2022, NVIDIA CORPORATION. All rights reserved.
## NVIDIA CORPORATION and its licensors retain all intellectual property
## and proprietary rights in and to this software, related documentation
## and any modifications thereto.  Any use, reproduction, disclosure or
## distribution of this software and related documentation without an express
## license agreement from NVIDIA CORPORATION is strictly prohibited.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # Get the launch directory
    nav2_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")

    # Create the launch configuration variables
    slam = LaunchConfiguration("slam")
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    autostart = LaunchConfiguration("autostart")

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace", default_value="false", description="Whether to apply a namespace to the navigation stack"
    )

    declare_slam_cmd = DeclareLaunchArgument("slam", default_value="False", description="Whether run a SLAM")

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(nav2_launch_dir, "maps", "carter_warehouse_navigation.yaml"),
        description="Full path to map file to load",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="True", description="Use simulation (Isaac Sim) clock if true"
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(nav2_launch_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("nav2_bt_navigator"), "behavior_trees", "navigate_w_replanning_and_recovery.xml"
        ),
        description="Full path to the behavior tree xml file to use",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="true", description="Automatically startup the nav2 stack"
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, "bringup_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "use_namespace": use_namespace,
            "slam": slam,
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "default_bt_xml_filename": default_bt_xml_filename,
            "autostart": autostart,
        }.items(),   
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(bringup_cmd)

    return ld
