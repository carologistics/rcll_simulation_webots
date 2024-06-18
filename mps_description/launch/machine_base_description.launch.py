#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import os

import launch
import launch_ros.descriptions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

package_name = "mps_description"


def generate_launch_description():
    pkg_share = get_package_share_directory(package_name)
    default_model_path = os.path.join(pkg_share, "urdf/machines/machine_base_description.urdf")
    default_rviz2_path = os.path.join(pkg_share, "rviz/machine_base_description.rviz")

    # Initialize robot state publisher
    robot_state_publisher_node = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": Command(["xacro ", LaunchConfiguration("model")])}],
    )

    # Initialize joint state broadcaster
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=launch.conditions.UnlessCondition(LaunchConfiguration("gui")),
    )

    # Initialize joint state publisher gui
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=launch.conditions.IfCondition(LaunchConfiguration("gui")),
    )

    # Initialize rviz2
    rviz_node = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d" + default_rviz2_path],
    )

    return LaunchDescription(
        [
            # Declare launch arguments
            launch.actions.DeclareLaunchArgument(
                name="gui", default_value="True", description="Flag to enable joint_state_publisher_gui"
            ),
            launch.actions.DeclareLaunchArgument(
                name="model", default_value=default_model_path, description="Absolute path to robot urdf file"
            ),
            launch.actions.DeclareLaunchArgument(
                name="rvizconfig", default_value=default_rviz2_path, description="Absolute path to rviz config file"
            ),
            # Launch nodes
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
