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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from webots_ros2_driver.webots_controller import WebotsController


def launch_nodes_withconfig(context, *args, **kwargs):

    package_dir = get_package_share_directory("robotino3_simulation")

    # Declare launch configuration variables
    namespace = LaunchConfiguration("namespace")
    joy_device_id = LaunchConfiguration("joy_device_id")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_joynode = LaunchConfiguration("launch_joynode")
    launch_teleopnode = LaunchConfiguration("launch_teleopnode")

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval  #

    # Load robot description file
    def load_file(filename):
        return pathlib.Path(os.path.join(package_dir, "urdf/robots", filename)).read_text()

    # Create a list of nodes to launch
    load_nodes = GroupAction(
        actions=[
            # Start Webots Controller
            WebotsController(
                robot_name=launch_configuration["namespace"],
                parameters=[
                    {
                        "robot_description": os.path.join(
                            package_dir, "urdf/robots", launch_configuration["namespace"] + "_description_plugin.urdf"
                        )
                    },
                    {"use_sim_time": True},
                ],
                respawn=True,
            ),
            # Robot state publisher node
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {"robot_description": load_file("robotino3_description.urdf")},
                    {"use_sim_time": use_sim_time},
                    {"frame_prefix": launch_configuration["namespace"] + "/"},
                ],
                namespace=namespace,
            ),
            # Joy node to enable joystick teleop
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="log",
                namespace=namespace,
                parameters=[
                    {
                        "device_id": joy_device_id,
                        "use_sim_time": use_sim_time,
                    }
                ],
                condition=IfCondition(launch_joynode),
            ),
            # Joy teleop node to enable joystick teleop
            Node(
                package="robotino3_sensors",
                executable="robotino3_joyteleop",
                name="robotino3_joyteleop",
                output="log",
                namespace=namespace,
                condition=IfCondition(launch_teleopnode),
            ),
            # Laserscan republisher node
            Node(
                package="robotino3_sensors",
                executable="robotino3_laserscan_republisher",
                name="robotino3_laserscan_republisher",
                output="log",
                parameters=[{"frame_prefix": namespace}],
                namespace=namespace,
            ),
            # Irscan merge node
            Node(
                package="robotino3_sensors",
                executable="robotino3_irscanmerger",
                name="robotino3_irscanmerger",
                output="log",
                parameters=[{"frame_prefix": namespace}],
                namespace=namespace,
            ),
            # Launch Integrate laserscan launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [FindPackageShare("laser_scan_integrator"), "launch", "integrated_scan.launch.py"]
                        )
                    ]
                ),
                launch_arguments={
                    "namespace": namespace,
                }.items(),
            ),
            # Launch Integrate laserscan launch file
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [FindPackageShare("robotino3_sensors"), "launch", "odom_ekffusion.launch.py"]
                        )
                    ]
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            # Spawn Rviz2 node for visualization
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    os.path.join(package_dir, "rviz", launch_configuration["namespace"] + "_rvizconfig.rviz"),
                ],
                output="screen",
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
    return [load_nodes]


def generate_launch_description():

    # Declare launch configuration variables
    declare_namespace_argument = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")

    declare_namespace_argument = DeclareLaunchArgument(
        "namespace", default_value="0", description="Device ID of the joystick"
    )

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock if true"
    )

    declare_launch_rviz_argument = DeclareLaunchArgument(
        "launch_rviz", default_value="true", description="Wheather to start Rvizor not based on launch environment"
    )

    declare_launch_joynode_argument = DeclareLaunchArgument(
        "launch_joynode", default_value="true", description="Wheather to start Rvizor not based on launch environment"
    )

    declare_launch_teleopnode_argument = DeclareLaunchArgument(
        "launch_teleopnode",
        default_value="true",
        description="Wheather to start Rvizor not based on launch environment",
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_launch_rviz_argument)
    ld.add_action(declare_launch_joynode_argument)
    ld.add_action(declare_launch_teleopnode_argument)

    # Add the actions to launch all nodes
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
