#!/usr/bin/env python3
"""
PSD EKF Calibrated Launch File
Launches EKF with calibrated parameters from the calibration tool
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Config file path - uses calibrated config
    config_file = PathJoinSubstitution([
        FindPackageShare('robot_localization'),
        'params',
        'psd_ekf_calibrated.yaml'
    ])
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='ego_vehicle',
        description='Base frame of the robot'
    )
    
    monitor_arg = DeclareLaunchArgument(
        'monitor',
        default_value='true',
        description='Launch performance monitor'
    )
    
    # Main EKF node with calibrated configuration
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'base_link_frame': LaunchConfiguration('base_frame'),
            }
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
            ('accel/filtered', '/accel/filtered'),
        ]
    )
    
    # Performance monitor (optional)
    monitor_node = Node(
        package='robot_localization',
        executable='ekf_monitor.py',
        name='ekf_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('monitor'))
    )
    
    # Static transforms
    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ego_to_imu_tf',
        arguments=[
            '0.0', '0.0', '0.1',
            '0.0', '0.0', '0.0',
            'ego_vehicle', 'imu_onBoard_link'
        ]
    )
    
    zed_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ego_to_zed_tf',
        arguments=[
            '0.3', '0.0', '0.4',
            '0.0', '0.0', '0.0',
            'ego_vehicle', 'zed2_base_link'
        ]
    )
    
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            'map', 'odom'
        ]
    )
    
    # Info message
    info_msg = LogInfo(
        msg="""
        ╔══════════════════════════════════════════════════════════╗
        ║         PSD EKF with CALIBRATED Parameters              ║
        ╠══════════════════════════════════════════════════════════╣
        ║                                                          ║
        ║  Running with automatically tuned parameters from:      ║
        ║  • psd_ekf_calibrated.yaml                              ║
        ║                                                          ║
        ║  Features:                                               ║
        ║  • Noise-optimized covariance matrices                  ║
        ║  • Drift compensation enabled (if detected)             ║
        ║  • Sensor trust levels adjusted                         ║
        ║  • Performance monitoring enabled                       ║
        ║                                                          ║
        ║  Output: /odometry/filtered                             ║
        ║                                                          ║
        ╚══════════════════════════════════════════════════════════╝
        """
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        base_frame_arg,
        monitor_arg,
        
        # Info
        info_msg,
        
        # Nodes
        ekf_node,
        monitor_node,
        
        # Static transforms
        imu_static_tf,
        zed_static_tf,
        map_to_odom_tf,
    ])