#!/usr/bin/env python

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
import pathlib
from launch.actions import (LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import  OnProcessStart


def launch_nodes_withconfig(context, *args, **kwargs):

    package_dir = get_package_share_directory('robotino3_simulation')
    

    use_sim_time = LaunchConfiguration('use_sim_time')
    mps_config = LaunchConfiguration('mps_config')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_joynode = LaunchConfiguration('launch_joynode')
    launch_teleopnode = LaunchConfiguration('launch_teleopnode')
    
    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval#
        
    robotino3_mpsspawner = Node(
        package="robotino3_simulation",
        executable="robotino3_mpspublisher",
        name ="robotino3_mpspublisher",
        parameters = [mps_config, 
                      {'webots_world': 'webots_robotinocluster_sim.wbt'}],
        output ="log",
    )
    
    # Starts Webots simulation and superwisor nodes
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', 'modified_webots_robotinocluster_sim.wbt']),
        mode="realtime",
        ros2_supervisor=True
    )
    
    def load_file(filename):
        return pathlib.Path(os.path.join(package_dir, 'urdf/robots', filename)).read_text()

    load_nodes = GroupAction(
        actions=[
        
        # Launch robotinobase1 controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robotino3_simulation'),'launch','robotino_controller.launch.py'
                ])
            ]),
            launch_arguments={
                'namespace': 'robotinobase1',
                'use_sim_time': use_sim_time,
                'launch_rviz': launch_rviz,
                'launch_joynode': launch_joynode,
                'launch_teleopnode': launch_teleopnode,
            }.items()
        ),
        
        # Launch robotinobase1 controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robotino3_simulation'),'launch','robotino_controller.launch.py'
                ])
            ]),
            launch_arguments={
                'namespace': 'robotinobase2',
                'use_sim_time': use_sim_time,
                'launch_rviz': launch_rviz,
                'launch_joynode': launch_joynode,
                'launch_teleopnode': launch_teleopnode,
            }.items()
        ),
        
        # Launch robotinobase1 controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robotino3_simulation'),'launch','robotino_controller.launch.py'
                ])
            ]),
            launch_arguments={
                'namespace': 'robotinobase3',
                'use_sim_time': use_sim_time,
                'launch_rviz': launch_rviz,
                'launch_joynode': launch_joynode,
                'launch_teleopnode': launch_teleopnode,
            }.items()
        ),
    ])
    
    return[robotino3_mpsspawner, 
           RegisterEventHandler(
            OnProcessStart(target_action=robotino3_mpsspawner,
                            on_start=[LogInfo(msg='Mpss spawn init, starting webots'),
                                    TimerAction(period=2.0,
                                                actions=[webots,webots._supervisor,]),
                                    TimerAction(
                                                period=30.0,
                                                actions=[load_nodes])]))]

def generate_launch_description():
    package_dir = get_package_share_directory('robotino3_simulation')
    
    declare_namespace_argument = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace')

    declare_mps_config_argument = DeclareLaunchArgument(
        'mps_config',default_value=os.path.join(package_dir, 'config', 'mps_pose.yaml'),
        description='Full path to mps_config.yaml file to load')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true')
    
    declare_launch_rviz_argument = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false', 
        description= 'Wheather to start Rvizor not based on launch environment')
    
    declare_launch_joynode_argument = DeclareLaunchArgument(
        'launch_joynode',
        default_value='true', 
        description= 'Wheather to start Rvizor not based on launch environment')
    
    declare_launch_teleopnode_argument = DeclareLaunchArgument(
        'launch_teleopnode',
        default_value='true', 
        description= 'Wheather to start Rvizor not based on launch environment')
    
     # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_argument)
    ld.add_action(declare_mps_config_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_launch_rviz_argument)
    ld.add_action(declare_launch_joynode_argument)
    ld.add_action(declare_launch_teleopnode_argument)

    # Add the actions to launch all of the localiztion nodes
    ld.add_action(OpaqueFunction(function=launch_nodes_withconfig))
    
    return ld