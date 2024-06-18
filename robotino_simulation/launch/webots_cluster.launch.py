#!/usr/bin/env python
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory("robotino_simulation")

    # Declare launch configuration variables
    mps_config = os.path.join(package_dir, "config", "mps_pose.yaml")

    # Initialize mpspawner node
    mpspawner = Node(
        package="robotino_simulation",
        executable="mps_publisher",
        name="mps_publisher",
        parameters=[mps_config, {"webots_world": "webots_robotinocluster_sim.wbt"}],
        output="log",
    )

    # Start Webots simulation and supervisor nodes
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", "modified_webots_robotinocluster_sim.wbt"]),
        mode="realtime",
        ros2_supervisor=True,
    )

    return LaunchDescription(
        [
            mpspawner,
            # Register an event handler
            RegisterEventHandler(
                OnProcessStart(
                    target_action=mpspawner,
                    on_start=[
                        LogInfo(msg="Mpss spawned, starting webots"),
                        TimerAction(
                            period=2.0,
                            actions=[
                                webots,
                                webots._supervisor,
                            ],
                        ),
                    ],
                )
            ),
        ]
    )
