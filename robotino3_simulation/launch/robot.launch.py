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
    
    def load_file(filename):
        return pathlib.Path(os.path.join(package_dir, 'urdf/robots', filename)).read_text()

    # Starts Webots simulation and superwisor nodes
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', 'webots_robotinosim_world_plugin.wbt']),
        mode="realtime",
        ros2_supervisor=True
    )

    # Start webots driver node
    robotino_driver = WebotsController(
        robot_name='robotino',
        parameters=[
            {'robot_description': os.path.join(package_dir, 'urdf/robots', 'robotino3_description_plugin.urdf')},
        ],
    )

    # Start Robot_state publisher for Rviz Vizualization 
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': load_file('robotino3_description.urdf')},
                    {'use_sim_time': False},
                ],
    )
    
    # Joy node for joystick control 
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="teleop_control",
        output="log",
    )
    
    return LaunchDescription([
        webots,
        webots._supervisor,
        
        # This action will kill all nodes once the Webots simulation has exited
        robotino_driver,
        robot_state_publisher,
        joy_node,

        # Kill all the nodes when the driver node is shut down
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=robotino_driver,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
