#!/usr/bin/env python3

from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.actions import (LogInfo, RegisterEventHandler, TimerAction, DeclareLaunchArgument)
from launch.event_handlers import OnProcessStart
import launch
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import (LogInfo, RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessStart
from nav2_common.launch import ReplaceString

def generate_launch_description():
    
    namespace = LaunchConfiguration('namespace')
    launch_mapserver = LaunchConfiguration('launch_mapserver')
    rviz_config = LaunchConfiguration('rviz_config')
    
    declare_namespace_argument = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_launchmapserver_argument = DeclareLaunchArgument(
        'launch_mapserver',
        default_value='true',
        description='Argument whether to launch map server or not')
    
    declare_rvizconfig_argument = DeclareLaunchArgument(
        'rviz_config',
        default_value='robotinobase3_nav2config.rviz',
        description='Argument whether to launch map server or not')
    
    launch_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robotino3_navigation'),'launch','robotino_localization.launch.py'])
            ]),
            launch_arguments={
                'namespace': namespace,
                'launch_mapserver': launch_mapserver,
            }.items()
        )
    
    launch_navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robotino3_navigation'),'launch','robotino_navigation.launch.py'])
            ]),
            launch_arguments={
                'namespace': namespace,
            }.items()
        )
    
    launch_rviz2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robotino3_navigation'),'launch','robotino_rviz.launch.py'])
            ]),
            launch_arguments={
                'namespace': namespace,
                'rviz_config': rviz_config,
            }.items()
        )
    
    return LaunchDescription([
        declare_namespace_argument,
        declare_launchmapserver_argument,
        declare_rvizconfig_argument,
        launch_localization,
        RegisterEventHandler(
            OnProcessStart(
                target_action=launch_localization,
                on_start=[
                    LogInfo(msg='AMCL launched, srtaing navigation stack'),
                    TimerAction(
                        period=5.0,
                        actions=[launch_navigation],
                    )
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=launch_navigation,
                on_start=[
                    LogInfo(msg='Navigation stack launched, starting RVIZ'),
                    TimerAction(
                        period=5.0,
                        actions=[launch_rviz2],
                    )
                ]
            )
        ),
        
        TimerAction(period=5.0, actions=[launch_navigation]),
        
        TimerAction(period=10.0,actions=[launch_rviz2]), 
        
        #Kill all the nodes when the nav2 stack dies
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=launch_navigation,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
