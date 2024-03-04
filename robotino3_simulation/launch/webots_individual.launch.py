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
    package_dir = get_package_share_directory("robotino3_simulation")

    # Declare launch configuration variables
    mps_config = os.path.join(package_dir, "config", "mps_pose.yaml")

    # Initialize robotino3_mpspawner node
    robotino3_mpspawner = Node(
        package="robotino3_simulation",
        executable="robotino3_mpspublisher",
        name="robotino3_mpspublisher",
        parameters=[mps_config, {"webots_world": "webots_robotinobase3_sim.wbt"}],
        output="log",
    )

    # Start Webots simulation and supervisor nodes
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", "modified_webots_robotinobase3_sim.wbt"]),
        mode="realtime",
        ros2_supervisor=True,
    )

    return LaunchDescription(
        [
            robotino3_mpspawner,
            # Register an event handler
            RegisterEventHandler(
                OnProcessStart(
                    target_action=robotino3_mpspawner,
                    on_start=[
                        LogInfo(msg="Mpss spawn init, starting webots"),
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
