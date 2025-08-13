#!/usr/bin/env python3
"""
PSD EKF Advanced Launch File - Minimal Drift Configuration
Optimized multi-sensor fusion for minimal drift using:
- Xsens IMU for high-quality orientation
- ZED2 visual-inertial odometry for position
- ZED2 IMU for redundant acceleration
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('robot_localization'),
        'params',
        'psd_ekf_advanced.yaml'
    ])
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish TF transforms'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='ego_vehicle',
        description='Base frame (ego_vehicle or chassis)'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug output'
    )
    
    # Main EKF node with advanced configuration
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
                'publish_tf': LaunchConfiguration('publish_tf'),
                'debug': LaunchConfiguration('debug'),
            }
        ],
        remappings=[
            # Keep original topic names for clarity
            ('odometry/filtered', '/odometry/filtered'),
            ('accel/filtered', '/accel/filtered'),
        ]
    )
    
    # Static transform: ego_vehicle to imu_onBoard_link
    # Adjust these values based on actual IMU mounting position
    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ego_to_imu_tf',
        arguments=[
            '0.0', '0.0', '0.1',  # x, y, z translation (IMU slightly above base)
            '0.0', '0.0', '0.0',  # roll, pitch, yaw rotation
            'ego_vehicle', 'imu_onBoard_link'
        ]
    )
    
    # Static transform: ego_vehicle to zed2_base_link (if needed)
    # Adjust based on ZED2 mounting position
    zed_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='ego_to_zed_tf',
        arguments=[
            '0.3', '0.0', '0.4',  # x, y, z (ZED mounted forward and up)
            '0.0', '0.0', '0.0',  # roll, pitch, yaw
            'ego_vehicle', 'zed2_base_link'
        ]
    )
    
    # Initial map to odom transform (identity)
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
    
    # Diagnostic aggregator for monitoring sensor health
    diagnostic_aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        parameters=[{
            'analyzers': {
                'ekf': {
                    'type': 'diagnostic_aggregator/GenericAnalyzer',
                    'path': 'EKF',
                    'contains': ['ekf_filter_node']
                },
                'sensors': {
                    'type': 'diagnostic_aggregator/GenericAnalyzer',
                    'path': 'Sensors',
                    'contains': ['imu', 'zed']
                }
            }
        }],
        condition=IfCondition(LaunchConfiguration('debug'))
    )
    
    # Log startup info
    log_info = LogInfo(
        msg="""
        ╔══════════════════════════════════════════════════════════╗
        ║     PSD EKF Advanced - Minimal Drift Configuration      ║
        ╠══════════════════════════════════════════════════════════╣
        ║ Sensor Fusion Strategy:                                 ║
        ║ • Xsens IMU:  Orientation + Angular Velocity (PRIMARY)  ║
        ║ • ZED Odom:   Position + Linear Velocity (PRIMARY)      ║
        ║ • ZED IMU:    Linear Acceleration (BACKUP)              ║
        ║ • MPC:        Control Prediction (OPTIONAL)             ║
        ╠══════════════════════════════════════════════════════════╣
        ║ Key Features:                                            ║
        ║ • 100 Hz update rate for smooth estimation              ║
        ║ • Multi-sensor redundancy                               ║
        ║ • Optimized covariances to minimize drift               ║
        ║ • Visual-inertial odometry from ZED2                    ║
        ╠══════════════════════════════════════════════════════════╣
        ║ Output: /odometry/filtered                              ║
        ╚══════════════════════════════════════════════════════════╝
        """
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        publish_tf_arg,
        base_frame_arg,
        debug_arg,
        
        # Info message
        log_info,
        
        # Core nodes
        ekf_node,
        
        # Static transforms
        imu_static_tf,
        zed_static_tf,
        map_to_odom_tf,
        
        # Optional diagnostic aggregator
        diagnostic_aggregator,
    ])