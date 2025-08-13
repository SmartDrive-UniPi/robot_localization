#!/usr/bin/env python3
"""
PSD EKF Launch File
This launch file starts the robot_localization EKF node configured for the PSD autonomous racing vehicle.
It fuses IMU data with visual odometry from cone detection for robust localization.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get package directories
    pkg_robot_localization = FindPackageShare('robot_localization')
    
    # Config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('robot_localization'),
        'params',
        'psd_ekf.yaml'
    ])
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    output_final_position_arg = DeclareLaunchArgument(
        'output_final_position',
        default_value='false',
        description='Output final position when node exits'
    )
    
    output_location_arg = DeclareLaunchArgument(
        'output_location',
        default_value='~/psd_ws/ekf_final_position.txt',
        description='File path for final position output'
    )
    
    # EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Remap control input if using MPC
            ('cmd_vel', '/cmd_vel'),
            # Output remappings
            ('odometry/filtered', '/ekf/odometry'),
            ('accel/filtered', '/ekf/acceleration'),
        ]
    )
    
    # Optional: Transform publisher from map to odom if SLAM is providing map->ego_vehicle
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=None  # Always publish for now
    )
    
    # Log info
    log_info = LogInfo(
        msg="PSD EKF Localization started. Fusing IMU (/imu/data) with SLAM pose (/slam/pose)"
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        output_final_position_arg,
        output_location_arg,
        
        # Nodes
        log_info,
        ekf_node,
        static_transform_publisher,
    ])