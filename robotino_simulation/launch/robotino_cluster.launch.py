#!/usr/bin/env python
# Licensed under MIT. See LICENSE file. Copyright Carologistics.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Initialize robotino base 1
    robotinobase1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("robotino_simulation"), "launch"),
                "/robotinobase1_individual.launch.py",
            ]
        )
    )

    # Initialize robotino base 2
    robotinobase2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("robotino_simulation"), "launch"),
                "/robotinobase2_individual.launch.py",
            ]
        )
    )

    # Initialize robotino base 3
    robotinobase3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("robotino_simulation"), "launch"),
                "/robotinobase3_individual.launch.py",
            ]
        )
    )

    return LaunchDescription(
        [
            # Launch robotino base instances
            robotinobase1,
            robotinobase2,
            robotinobase3,
        ]
    )
