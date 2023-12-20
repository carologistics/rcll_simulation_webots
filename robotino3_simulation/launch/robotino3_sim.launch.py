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

machine_locations = \
[
    '0.40 2.0 0.4', '1.5 2.0 0.4', '3.5 0.5 0.4', '3.5 -0.5 0.4',
    '1.5 -2.0 0.4', '0.40 -2.0 0.4', '5.5 1.5 0.4', '5.5 -1.5 0.4',
    '-0.40 2.0 0.4', '-1.5 2.0 0.4', '-3.5 0.5 0.4', '-3.5 -0.5 0.4',
    '-1.5 -2.0 0.4', '-0.40 -2.0 0.4', '-5.5 1.5 0.4', '-5.5 -1.5 0.4',
]

machine_orientation = \
[
    '0 0 1 1.57', '0 0 1 1.57', '0 0 1 0', '0 0 1 0',
    '0 0 1 1.57', '0 0 1 1.57', '0 0 1 1.57', '0 0 1 1.57',
    '0 0 1 1.57', '0 0 1 1.57', '0 0 1 0', '0 0 1 0',
    '0 0 1 1.57', '0 0 1 1.57', '0 0 1 1.57', '0 0 1 1.57',
]

def generate_launch_description():
    package_dir = get_package_share_directory('robotino3_simulation')
    
    def load_file(filename):
        return pathlib.Path(os.path.join(package_dir, 'urdf/robots', filename)).read_text()

    
    world = os.path.join(package_dir, 'worlds', 'webots_robotinosim_world_plugin.wbt')
    with open(world, 'r') as f:
        world_txt = f.read()
        
    modified_world = os.path.join(package_dir, 'worlds', 'modified_world.wbt')
    with open(modified_world, 'w') as f:
        externproto_txt = ''
        externproto_txt = externproto_txt + 'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/objects/factory/containers/protos/WoodenBox.proto"\n'
        
        world_txt = world_txt.split('\n')
        world_txt[1] = externproto_txt
        world_txt = '\n'.join(world_txt)
        
        wooden_box = ''
        idx = 0 
        for box_location, box_orientation in \
            zip(machine_locations, machine_orientation):
            
            idx += 1
            wooden_box += \
"""
WoodenBox {
translation """ + box_location + """
rotation """+ box_orientation +"""
name \"machine""" + str(idx) + """\"
size 0.35 0.7 0.8
}"""
        f.write(world_txt+wooden_box)   
    
    # Starts Webots simulation and superwisor nodes
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', 'modified_world.wbt']),
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
                    {'use_sim_time': True},
                ],
    )
    
    # Joy node for joystick control 
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="teleop_control",
        output="log",
    )
    
    # Laserscan republisher node
    robotino3_lasercsnrepublish_node = Node(
        package="robotino3_sensors",
        executable="robotino3_laserscan_republisher",
        name ="robotino3_laserscan_republisher",
        output ="log",
    )
    
    # Irscan merge node
    robotino3_irscanmerege_node = Node(
        package="robotino3_sensors",
        executable="robotino3_irscanmerger",
        name ="robotino3_irscanmerger",
        output ="log", 
    )
    
    # Robotino teleopcontrol node
    robotino3_teleop = Node(
        package="robotino3_sensors",
        executable="robotino3_joyteleop",
        name ="robotino3_joyteleop",
        output ="log",
    )
    
    return LaunchDescription([
        webots,
        webots._supervisor,
        robotino_driver,
        robot_state_publisher,
        joy_node,
        robotino3_lasercsnrepublish_node,
        robotino3_irscanmerege_node,
        robotino3_teleop,
        
        # Kill all the nodes when the driver node is shut down
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=robotino_driver,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
