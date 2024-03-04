#!/usr/bin/env python
# Author: Saurabh Borse(saurabh.borse@alumni.fh-aachen.de)
#  MIT License
#  Copyright (c) 2023 Saurabh Borse
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
import os
import pathlib

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():

    package_dir = get_package_share_directory("robotino3_simulation")

    # Declare launch configuration variables
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_joynode = LaunchConfiguration("launch_joynode")
    launch_teleopnode = LaunchConfiguration("launch_teleopnode")

    # Declare launch arguments
    launch_rviz_argument = DeclareLaunchArgument(
        "launch_rviz", default_value="true", description="Wheather to start Rvizor not based on launch environment"
    )

    launch_joynode_argument = DeclareLaunchArgument(
        "launch_joynode", default_value="true", description="Wheather to start Rvizor not based on launch environment"
    )

    launch_teleopnode_argument = DeclareLaunchArgument(
        "launch_teleopnode",
        default_value="true",
        description="Wheather to start Rvizor not based on launch environment",
    )

    # Load robot description file
    def load_file(filename):
        return pathlib.Path(os.path.join(package_dir, "urdf/robots", filename)).read_text()

    # Start Webots Controller
    robotino_driver = WebotsController(
        robot_name="robotinobase3",
        parameters=[
            {"robot_description": os.path.join(package_dir, "urdf/robots", "robotinobase3_description_plugin.urdf")},
            {"use_sim_time": True},
        ],
        respawn=True,
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": load_file("robotino3_description.urdf")},
            {"use_sim_time": True},
            {"frame_prefix": "robotinobase3/"},
        ],
        namespace="robotinobase3",
    )

    # Joy node for joystick control
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="teleop_control",
        output="log",
        namespace="robotinobase3",
        parameters=[
            {"device_id": 0},
            # {'device_name': '/dev/input/js0'}
        ],
        condition=IfCondition(launch_joynode),
    )

    # Joy teleop node
    robotino3_joyteleop_node = Node(
        package="robotino3_sensors",
        executable="robotino3_joyteleop",
        name="robotino3_joyteleop",
        output="log",
        namespace="robotinobase3",
        condition=IfCondition(launch_teleopnode),
    )

    # Laserscan republisher node
    robotino3_lasercsnrepublish_node = Node(
        package="robotino3_sensors",
        executable="robotino3_laserscan_republisher",
        name="robotino3_laserscan_republisher",
        output="log",
        parameters=[{"frame_prefix": "robotinobase3"}],
        namespace="robotinobase3",
    )

    # Irscan merge node
    robotino3_irscanmerege_node = Node(
        package="robotino3_sensors",
        executable="robotino3_irscanmerger",
        name="robotino3_irscanmerger",
        output="log",
        parameters=[{"frame_prefix": "robotinobase3"}],
        namespace="robotinobase3",
    )

    # Launch Integrate laserscan launch file
    robotino_laserscanmerge_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([FindPackageShare("laser_scan_integrator"), "launch", "integrated_scan.launch.py"])]
        ),
        launch_arguments={
            "integratedTopic": "/robotinobase3/scan",
            "integratedFrameId": "robotinobase3/laser_link",
            "scanTopic1": "/robotinobase3/SickLaser_Front_Remaped",
            "scanTopic2": "/robotinobase3/SickLaser_Rear_Remaped",
        }.items(),
    )

    # Spawn Rviz2 node for visualization
    rviz_config_dir = os.path.join(package_dir, "rviz", "robotinobase3_rvizconfig.rviz")

    # Rviz2 node
    robotino_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_dir],
        output="screen",
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        [
            # Launch nodes with arguments
            robotino_driver,
            robot_state_publisher,
            launch_joynode_argument,
            joy_node,
            launch_teleopnode_argument,
            robotino3_joyteleop_node,
            robotino3_lasercsnrepublish_node,
            robotino3_irscanmerege_node,
            robotino_laserscanmerge_node,
            launch_rviz_argument,
            robotino_rviz_node,
            # Kill all the nodes when the driver node is shut down
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=robotino_driver,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
