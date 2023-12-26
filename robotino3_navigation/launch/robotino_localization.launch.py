#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition
import launch

def launch_nodes_withconfig(context, *args, **kwargs):

    bringup_dir = get_package_share_directory('robotino3_navigation')
    
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_mapserver = LaunchConfiguration('launch_mapserver')
    lifecycle_nodes = ['map_server', 'amcl']
    
    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    launch_configuration = {}
    for argname, argval in context.launch_configurations.items():
        launch_configuration[argname] = argval
    
    remappings = [('/'+launch_configuration['namespace']+'/tf', '/tf'),
                  ('/'+launch_configuration['namespace']+'/tf_static', '/tf_static'),
                  ('/'+launch_configuration['namespace']+'/map','/map')]
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
        namespace=namespace)

    nav2_amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params],
        remappings=remappings,
        namespace=namespace,
        condition=IfCondition(launch_mapserver))

    nav2_lifecyclemanager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}],
        namespace=namespace)
    
    rviz_config_dir = os.path.join(bringup_dir,'rviz', 'robotino_localization.rviz')
    
    robotino_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen',
        condition = IfCondition(launch_rviz),
    )

    return[
        map_server_node,
        nav2_amcl_node,
        nav2_lifecyclemanager_node,
        robotino_rviz_node,
    ]

def generate_launch_description():
    
    bringup_dir = get_package_share_directory('robotino3_navigation')

    namespace_argument = DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace')

    map_argument = DeclareLaunchArgument(
        'map',default_value=os.path.join(bringup_dir, 'map', 'map.yaml'),
        description='Full path to map yaml file to load')

    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    autostart_argument = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    param_file_argument = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'localization_params.yaml'),
        description='Full path to the ROS2 parameters file to use')

    launch_rviz_argument = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false', 
        description= 'Wheather to start Rvizor not based on launch environment')
    
    launch_mapserver_argument = DeclareLaunchArgument(
        'launch_mapserver',
        default_value='true', 
        description= 'Wheather to launch map server or not')

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        namespace_argument,
        map_argument,
        use_sim_time_argument,
        autostart_argument,
        param_file_argument,
        launch_rviz_argument,
        launch_mapserver_argument,
        OpaqueFunction(function=launch_nodes_withconfig),
    ])