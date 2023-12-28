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

#!/usr/bin/env python

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from webots_ros2_driver.webots_controller import WebotsController
import pathlib
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_dir = get_package_share_directory('robotino3_simulation')
    
    # Declare launch configuration variables
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_joynode = LaunchConfiguration('launch_joynode')
    launch_teleopnode = LaunchConfiguration('launch_teleopnode')
    
    # Declare launch arguments
    launch_rviz_argument = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true', 
        description= 'Wheather to start Rvizor not based on launch environment')
    
    launch_joynode_argument = DeclareLaunchArgument(
        'launch_joynode',
        default_value='true', 
        description= 'Wheather to start Rvizor not based on launch environment')
    
    launch_teleopnode_argument = DeclareLaunchArgument(
        'launch_teleopnode',
        default_value='true', 
        description= 'Wheather to start Rvizor not based on launch environment')
    
    # Load robot description file
    def load_file(filename):
        return pathlib.Path(os.path.join(package_dir, 'urdf/robots', filename)).read_text()
    
    # Start Webots Controller
    robotino_driver = WebotsController(
        robot_name='robotinobase1',
        parameters=[
            {'robot_description': os.path.join(package_dir, 'urdf/robots', 'robotinobase1_description_plugin.urdf')},
            {'use_sim_time': True},
        ],
        respawn=True
    )
    
    # Robot state publisher node 
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': load_file('robotino3_description.urdf')},
                    {'use_sim_time': True},
                    {'frame_prefix':'robotinobase1/'}],
        namespace='robotinobase1',
    )
    
    # Joy node to enable joystick teleop  
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="teleop_control",
        output="log",
        namespace='robotinobase1',
        parameters=[{'device_id': 0},
                    #{'device_name': '/dev/input/js1'}
        ],
        condition = IfCondition(launch_joynode),
    )
    
    # Joy teleop node
    robotino_joyteleop_node = Node(
        package="robotino3_sensors",
        executable="robotino3_joyteleop",
        name ="robotino3_joyteleop",
        output ="log", 
        namespace='robotinobase1',
        condition = IfCondition(launch_teleopnode),
    )
    
    # Laserscan republisher node
    robotino_lasercsnrepublish_node = Node(
        package="robotino3_sensors",
        executable="robotino3_laserscan_republisher",
        name ="robotino3_laserscan_republisher",
        output ="log",
        parameters=[{'frame_prefix': 'robotinobase1'}],
        namespace='robotinobase1'
    )
    
    # Irscan merge node
    robotino_irscanmerege_node = Node(
        package="robotino3_sensors",
        executable="robotino3_irscanmerger",
        name ="robotino3_irscanmerger",
        output ="log", 
        parameters=[{'frame_prefix': 'robotinobase1'}],
        namespace='robotinobase1'
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
                                            'integratedTopic': '/robotinobase1/scan',
                                            'integratedFrameId': 'robotinobase1/laser_link',
                                            'scanTopic1': '/robotinobase1/SickLaser_Front_Remaped',
                                            'scanTopic2': '/robotinobase1/SickLaser_Rear_Remaped',
                                        }.items()
    )
    
    # Spawn Rviz2 node for visualization
    rviz_config_dir = os.path.join(package_dir,'rviz', 'robotinobase1_rvizconfig.rviz')
    
    robotino_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen',
        condition = IfCondition(launch_rviz),
    )
    
    return LaunchDescription([  
        # Launch nodes
        robotino_driver,
        robot_state_publisher,
        launch_joynode_argument,
        joy_node,
        launch_teleopnode_argument,
        robotino_joyteleop_node,
        robotino_lasercsnrepublish_node,
        robotino_irscanmerege_node,
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
