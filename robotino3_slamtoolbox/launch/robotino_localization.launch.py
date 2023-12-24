#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('robotino3_slamtoolbox')

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    launch_rviz = LaunchConfiguration('launch_rviz')
    lifecycle_nodes = ['map_server', 'amcl']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    
    namespace_argument = DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace')

    map_argument = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'map', 'map.yaml'),
        description='Full path to map yaml file to load')

    use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
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
        default_value='true', 
        description= 'Wheather to start Rvizor not based on launch environment')
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    nav2_amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params],
        remappings=remappings)

    nav2_lifecyclemanager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])
    
    rviz_config_dir = os.path.join(bringup_dir,'rviz', 'robotino_localization.rviz')
    
    robotino_rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen',
        condition = IfCondition(launch_rviz),
    )

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        namespace_argument,
        map_argument,
        use_sim_time_argument,
        autostart_argument,
        param_file_argument,
        launch_rviz_argument,
        map_server_node,
        nav2_amcl_node,
        nav2_lifecyclemanager_node,
        robotino_rviz_node,
    ])