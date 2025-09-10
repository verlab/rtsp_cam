#!/usr/bin/env python3
"""
Launch file for Generic RTSP Camera ROS 2 package
This launch file starts the main camera node, PTZ controller, and photo service
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for RTSP camera package."""
    
    # Get package directory
    pkg_share = FindPackageShare('rtsp_cam')
    
    # Launch arguments
    camera_config_arg = DeclareLaunchArgument(
        'camera_config',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'rtsp_cam_config.yaml']),
        description='Path to camera configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Get configuration file path
    config_file = LaunchConfiguration('camera_config')
    
    # Main camera node
    camera_node = Node(
        package='rtsp_cam',
        executable='rtsp_cam_node.py',
        name='rtsp_cam_node',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[
            ('image_raw', 'camera/main/image_raw'),
            ('camera_info', 'camera/main/camera_info'),
        ]
    )
    
    # Secondary stream node
    sub_stream_node = Node(
        package='rtsp_cam',
        executable='rtsp_cam_node.py',
        name='rtsp_sub_stream_node',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'info', '--sub-stream'],
        remappings=[
            ('image_raw', 'camera/sub/image_raw'),
            ('camera_info', 'camera/sub/camera_info'),
        ]
    )
    
    # PTZ controller node
    ptz_controller_node = Node(
        package='rtsp_cam',
        executable='rtsp_ptz_controller.py',
        name='rtsp_ptz_controller',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[
            ('ptz_status', 'camera/ptz_status'),
            ('ptz_cmd', 'camera/ptz_cmd'),
        ]
    )
    
    # Photo service node
    photo_service_node = Node(
        package='rtsp_cam',
        executable='rtsp_photo_service.py',
        name='rtsp_photo_service',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=[
            ('take_photo', 'camera/take_photo'),
        ]
    )
    
    # Create launch description
    return LaunchDescription([
        # Launch arguments
        camera_config_arg,
        use_sim_time_arg,
        
        # Nodes
        camera_node,
        sub_stream_node,
        ptz_controller_node,
        photo_service_node,
    ]) 