#!/usr/bin/env python3
# Licensed under MIT. See LICENSE file. Copyright Carologistics.

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the MAPF (Multi-Agent Path Finding) Executer node."""
    
    return LaunchDescription([
        Node(
            package='robotino_simulation',
            executable='mapf_executer',
            name='mapf_executer',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }],
        )
    ])
