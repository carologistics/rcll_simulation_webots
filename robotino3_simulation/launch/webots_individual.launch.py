#!/usr/bin/env python

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
import pathlib
from launch.actions import (LogInfo, RegisterEventHandler, TimerAction)
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)


def generate_launch_description():
    package_dir = get_package_share_directory('robotino3_simulation')
    
    mps_config = os.path.join(package_dir, 'config', 'mps_pose.yaml')
    
    robotino3_mpsspawner = Node(
        package="robotino3_simulation",
        executable="robotino3_mpspublisher",
        name ="robotino3_mpspublisher",
        parameters = [mps_config, 
                      {'webots_world': 'webots_robotinobase1_sim.wbt'}],
        output ="log",
    )
    
    # Starts Webots simulation and superwisor nodes
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', 'modified_webots_robotinobase1_sim.wbt']),
        mode="realtime",
        ros2_supervisor=True
    )
    
    return LaunchDescription([
        robotino3_mpsspawner,
        RegisterEventHandler(
            OnProcessStart(
                target_action=robotino3_mpsspawner,
                on_start=[
                    LogInfo(msg='Mpss spawn init, starting webots'),
                    TimerAction(
                        period=2.0,
                        actions=[webots,webots._supervisor,],
                    )
                ]
            )
        ),
    ])
