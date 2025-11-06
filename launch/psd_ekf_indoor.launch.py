#!/usr/bin/env python3
"""
PSD Indoor EKF Launch File
This launch file starts the robot_localization EKF node configured for indoor racing.
It fuses ZED visual odometry with IMU data for robust localization without GNSS.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('robot_localization'),
        'params',
        'psd_ekf_indoor.yaml'
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

    # EKF node for indoor localization
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_indoor',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Output remappings
            ('odometry/filtered', '/ekf/odometry'),
            ('accel/filtered', '/ekf/acceleration'),
        ]
    )

    # Static transform publisher from map to odom
    # Required for proper TF tree when not using GNSS
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Log info
    log_info = LogInfo(
        msg="PSD Indoor EKF Localization started. Using ZED visual odometry and IMU data (no GNSS)."
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