#!/usr/bin/env python3
"""
Common interface launch file for PSD EKF that provides consistent topic remapping
regardless of which EKF variant (calibrated, standalone, advanced) is used.

This allows changing output topics in one place for all EKF configurations.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate the launch description for EKF interface."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('robot_localization')
    
    # Declare arguments
    ekf_variant_arg = DeclareLaunchArgument(
        'ekf_variant',
        default_value='advanced',
        description='EKF variant to use: calibrated, standalone, or advanced'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='ego_vehicle',
        description='Base frame (ego_vehicle or chassis)'
    )
    
    # Common output topic remappings - CHANGE THESE AS NEEDED
    odometry_output_topic_arg = DeclareLaunchArgument(
        'odometry_output_topic',
        default_value='/odometry',  # Common output topic for all EKF variants
        description='Output odometry topic name'
    )
    
    pose_output_topic_arg = DeclareLaunchArgument(
        'pose_output_topic',
        default_value='/robot_pose',
        description='Output pose topic name'
    )
    
    # Input topic remappings (common for all variants)
    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data',
        description='IMU input topic'
    )
    
    visual_odom_topic_arg = DeclareLaunchArgument(
        'visual_odom_topic',
        default_value='/zed/zed_node/odom',
        description='Visual odometry input topic'
    )
    
    # Get launch configurations
    ekf_variant = LaunchConfiguration('ekf_variant')
    use_sim_time = LaunchConfiguration('use_sim_time')
    base_frame = LaunchConfiguration('base_frame')
    odometry_output_topic = LaunchConfiguration('odometry_output_topic')
    pose_output_topic = LaunchConfiguration('pose_output_topic')
    
    # Include the appropriate EKF launch file based on variant
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            pkg_dir, '/launch/psd_ekf_',
            ekf_variant,
            '.launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'base_frame': base_frame,
        }.items()
    )
    
    # Topic relay node to remap odometry/filtered to the common output topic
    odometry_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odometry_relay',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=[
            '/odometry/filtered',  # Source topic from EKF
            odometry_output_topic   # Target topic
        ]
    )
    
    # Topic relay node to remap accel/filtered to common output if needed
    accel_relay = Node(
        package='topic_tools',
        executable='relay',
        name='accel_relay',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        arguments=[
            '/accel/filtered',
            '/acceleration'
        ]
    )
    
    # Info node to display configuration
    info_node = Node(
        package='ros2',
        executable='ros2',
        name='ekf_info',
        output='screen',
        arguments=[
            'topic', 'echo', '--once',
            '/rosout',
            PythonExpression([
                '"EKF Interface initialized with variant: "',
                ekf_variant,
                '" outputting to: "',
                odometry_output_topic,
                '"'
            ])
        ],
        on_exit='ignore'
    )
    
    return LaunchDescription([
        # Arguments
        ekf_variant_arg,
        use_sim_time_arg,
        base_frame_arg,
        odometry_output_topic_arg,
        pose_output_topic_arg,
        imu_topic_arg,
        visual_odom_topic_arg,
        
        # Launch EKF
        ekf_launch,
        
        # Topic relays for common interface
        odometry_relay,
        accel_relay,
    ])