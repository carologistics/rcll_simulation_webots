#!/usr/bin/env python

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from webots_ros2_driver.urdf_spawner import URDFSpawner, get_webots_driver_node
import pathlib


def generate_launch_description():
    package_dir = get_package_share_directory('robotino3_simulation')
    
    # # Spawn Rviz2 node for visualization
    rviz_config_dir = os.path.join(package_dir,'rviz', 'robotinobase1_rvizconfig.rviz')
    robotino_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )
      
    return LaunchDescription([
        robotino_rviz_node
    ])
