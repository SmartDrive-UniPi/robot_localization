#!/usr/bin/env python3
"""
PSD Outdoor EKF Launch File
This launch file starts the robot_localization EKF node configured for outdoor racing.
It fuses MTi-680 INS/GNSS data with IMU for accurate outdoor localization.
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
        'psd_ekf_outdoor.yaml'
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

    # EKF node for outdoor localization with GNSS
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_outdoor',
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

    # Navsat transform node (converts GPS coordinates to local frame)
    # This is needed when using raw GNSS data
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[
            {'frequency': 30.0},
            {'delay': 0.0},
            {'magnetic_declination_radians': 0.0},  # Update based on location
            {'yaw_offset': 0.0},
            {'zero_altitude': True},
            {'broadcast_utm_transform': True},
            {'publish_filtered_gps': True},
            {'use_odometry_yaw': True},
            {'wait_for_datum': False},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('imu/data', '/imu/data'),
            ('gps/fix', '/gnss'),
            ('odometry/filtered', '/ekf/odometry'),
            ('odometry/gps', '/gnss_pose'),
        ]
    )

    # Static transform publisher from map to odom
    # May be updated by navsat_transform when using GPS
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Log info
    log_info = LogInfo(
        msg="PSD Outdoor EKF Localization started. Fusing MTi-680 INS/GNSS with IMU data."
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        output_final_position_arg,
        output_location_arg,

        # Nodes
        log_info,
        ekf_node,
        navsat_transform_node,
        static_transform_publisher,
    ])