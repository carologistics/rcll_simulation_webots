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
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from webots_ros2_driver.webots_launcher import WebotsLauncher


def launch_nodes_withconfig(context, *args, **kwargs):

    package_dir = get_package_share_directory("robotino_simulation")
    description_dir = get_package_share_directory("rto_description")

    # Declare launch configuration variables
    namespace = LaunchConfiguration("namespace")
    namespace.perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    mps_config = LaunchConfiguration("mps_config")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_joynode = LaunchConfiguration("launch_joynode")
    launch_teleopnode = LaunchConfiguration("launch_teleopnode")

    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval

    # Load mps spawn node
    mpspawner = Node(
        package="robotino_simulation",
        executable="mps_publisher.py",
        name="mps_publisher",
        parameters=[mps_config, {"webots_world": "webots_" + launch_configuration["namespace"] + "_sim.wbt"}],
        output="log",
    )

    # Starts Webots simulation and superwisor nodes
    webots = WebotsLauncher(
        world=PathJoinSubstitution(
            [package_dir, "worlds", "modified_webots_" + launch_configuration["namespace"] + "_sim.wbt"]
        ),
        mode="realtime",
        ros2_supervisor=True,
    )

    # Load robot description file
    def load_file(filename):
        return pathlib.Path(os.path.join(description_dir, "urdf/robots", filename)).read_text()

    # Create a list of nodes to launch
    load_nodes = GroupAction(
        actions=[
            # Launch robotinobase1 controller
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [FindPackageShare("robotino_simulation"), "launch", "robotino_controller.launch.py"]
                        )
                    ]
                ),
                launch_arguments={
                    "namespace": "robotinobase1",
                    "joy_device_id": "0",
                    "use_sim_time": use_sim_time,
                    "launch_rviz": launch_rviz,
                    "launch_joynode": launch_joynode,
                    "launch_teleopnode": launch_teleopnode,
                }.items(),
            ),
        ]
    )
    return [
        mpspawner,
        # Register event handler to start webots and load nodes
        RegisterEventHandler(
            OnProcessStart(
                target_action=mpspawner,
                on_start=[
                    LogInfo(msg="Mpss spawn init, starting webots"),
                    TimerAction(
                        period=2.0,
                        actions=[
                            webots,
                            webots._supervisor,
                        ],
                    ),
                    TimerAction(period=30.0, actions=[load_nodes]),
                ],
            )
        ),
    ]


def generate_launch_description():
    package_dir = get_package_share_directory("robotino_simulation")

    # Declare launch configuration variables
    declare_namespace_argument = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")

    declare_mps_config_argument = DeclareLaunchArgument(
        "mps_config",
        default_value=os.path.join(package_dir, "config", "mps_pose.yaml"),
        description="Full path to mps_config.yaml file to load",
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
    ld.add_action(declare_mps_config_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_launch_rviz_argument)
    ld.add_action(declare_launch_joynode_argument)
    ld.add_action(declare_launch_teleopnode_argument)

    # Add the actions to launch all nodes
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))

    return ld
