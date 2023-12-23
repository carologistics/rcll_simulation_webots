#!/usr/bin/env python

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression, LaunchConfiguration

def generate_launch_description():
    
    package_dir = get_package_share_directory('robotino3_simulation')

    def load_file(filename):
        return pathlib.Path(os.path.join(package_dir, 'urdf/robots', filename)).read_text()

    # Start webots driver node
    robotino_driver = WebotsController(
        robot_name='robotinobase3',
        parameters=[
            {'robot_description': os.path.join(package_dir, 'urdf/robots', 'robotinobase3_description_plugin.urdf')},
        ],
    )
    
    # Start Robot_state publisher for Rviz Vizualization 
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': load_file('robotino3_description.urdf')},
                    {'use_sim_time': True},
                    {'frame_prefix':'robotinobase3/'}],
        namespace='robotinobase3',
    )
    
    # Joy node for joystick control 
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="teleop_control",
        output="log",
        namespace='robotinobase3',
        parameters=[{'device_id': 0},
                    #{'device_name': '/dev/input/js0'}
        ]
    )
    
    # Laserscan republisher node
    robotino3_lasercsnrepublish_node = Node(
        package="robotino3_sensors",
        executable="robotino3_laserscan_republisher",
        name ="robotino3_laserscan_republisher",
        output ="log",
        parameters=[{'frame_prefix': 'robotinobase3'}],
        namespace='robotinobase3'
    )
    
    # Irscan merge node
    robotino3_irscanmerege_node = Node(
        package="robotino3_sensors",
        executable="robotino3_irscanmerger",
        name ="robotino3_irscanmerger",
        output ="log", 
        parameters=[{'frame_prefix': 'robotinobase3'}],
        namespace='robotinobase3'
    )
    
    # node to enable the joyteleop
    robotino3_joyteleop_node = Node(
        package="robotino3_sensors",
        executable="robotino3_joyteleop",
        name ="robotino3_joyteleop",
        output ="log", 
        namespace='robotinobase3'
    )

    # Launch Integrate laserscan launch file 
    robotino_laserscanmerge_node =  IncludeLaunchDescription(
                                        PythonLaunchDescriptionSource([
                                            PathJoinSubstitution([
                                                FindPackageShare('laser_scan_integrator'),
                                                'launch',
                                                'integrated_scan.launch.py'
                                            ])
                                        ]),
                                        launch_arguments={
                                            'integratedTopic': '/robotinobase3/scan',
                                            'integratedFrameId': 'robotinobase3/laser_link',
                                            'scanTopic1': '/robotinobase3/SickLaser_Front_Remaped',
                                            'scanTopic2': '/robotinobase3/SickLaser_Rear_Remaped',
                                        }.items()
    )
    
    # Spawn Rviz2 node for visualization
    rviz_config_dir = os.path.join(package_dir,'rviz', 'robotinobase3_rvizconfig.rviz')
    launch_rviz = LaunchConfiguration('launch_rviz')
    
    launch_rviz_argument = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true'
    )
    
    robotino_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen',
        condition = IfCondition(launch_rviz),
    )
    
    return LaunchDescription([
        robotino_driver,
        robot_state_publisher,
        joy_node,
        robotino3_lasercsnrepublish_node,
        robotino3_irscanmerege_node,
        robotino3_joyteleop_node,
        robotino_laserscanmerge_node,
        launch_rviz_argument,
        robotino_rviz_node, 
        
        # Kill all the nodes when the driver node is shut down
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=robotino_driver,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
