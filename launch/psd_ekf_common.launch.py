#!/usr/bin/env python3
"""
Common EKF launch file that provides a unified interface for all EKF variants.
This allows changing output topics in one central place.

Usage:
    ros2 launch robot_localization psd_ekf_common.launch.py ekf_variant:=advanced
    ros2 launch robot_localization psd_ekf_common.launch.py ekf_variant:=calibrated
    ros2 launch robot_localization psd_ekf_common.launch.py ekf_variant:=standalone
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    """Generate the common EKF launch description."""
    
    # Package directories
    pkg_share = FindPackageShare('robot_localization')
    
    # ============================================
    # LAUNCH ARGUMENTS
    # ============================================
    
    ekf_variant_arg = DeclareLaunchArgument(
        'ekf_variant',
        default_value='advanced',
        description='EKF variant: calibrated, standalone, or advanced'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='ego_vehicle',
        description='Robot base frame'
    )
    
    # ============================================
    # COMMON OUTPUT TOPICS - CHANGE HERE
    # ============================================
    # These are the unified output topics that all nodes should use
    
    odometry_output_arg = DeclareLaunchArgument(
        'odometry_output',
        default_value='/odometry',  # Common odometry output
        description='Unified odometry output topic'
    )
    
    pose_output_arg = DeclareLaunchArgument(
        'pose_output',
        default_value='/robot_pose',  # Common pose output
        description='Unified pose output topic'
    )
    
    accel_output_arg = DeclareLaunchArgument(
        'accel_output',
        default_value='/acceleration',  # Common acceleration output
        description='Unified acceleration output topic'
    )
    
    # ============================================
    # COMMON INPUT TOPICS
    # ============================================
    
    imu_input_arg = DeclareLaunchArgument(
        'imu_input',
        default_value='/imu/data',
        description='IMU input topic'
    )
    
    visual_odom_input_arg = DeclareLaunchArgument(
        'visual_odom_input',
        default_value='/zed/zed_node/odom',
        description='Visual odometry input topic'
    )
    
    # Get launch configurations
    ekf_variant = LaunchConfiguration('ekf_variant')
    use_sim_time = LaunchConfiguration('use_sim_time')
    base_frame = LaunchConfiguration('base_frame')
    odometry_output = LaunchConfiguration('odometry_output')
    pose_output = LaunchConfiguration('pose_output')
    accel_output = LaunchConfiguration('accel_output')
    
    # ============================================
    # EKF NODE CONFIGURATIONS FOR EACH VARIANT
    # ============================================
    
    # Advanced EKF configuration
    ekf_node_advanced = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'params', 'psd_ekf_advanced.yaml']),
            {
                'use_sim_time': use_sim_time,
                'base_link_frame': base_frame,
            }
        ],
        remappings=[
            ('odometry/filtered', odometry_output),
            ('accel/filtered', accel_output),
        ],
        condition=LaunchConfigurationEquals('ekf_variant', 'advanced')
    )
    
    # Calibrated EKF configuration
    ekf_node_calibrated = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'params', 'psd_ekf_calibrated.yaml']),
            {
                'use_sim_time': use_sim_time,
                'base_link_frame': base_frame,
            }
        ],
        remappings=[
            ('odometry/filtered', odometry_output),
            ('accel/filtered', accel_output),
        ],
        condition=LaunchConfigurationEquals('ekf_variant', 'calibrated')
    )
    
    # Standalone EKF configuration
    ekf_node_standalone = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'params', 'psd_ekf_standalone.yaml']),
            {
                'use_sim_time': use_sim_time,
                'base_link_frame': base_frame,
            }
        ],
        remappings=[
            ('odometry/filtered', odometry_output),
            ('accel/filtered', accel_output),
        ],
        condition=LaunchConfigurationEquals('ekf_variant', 'standalone')
    )
    
    # ============================================
    # STATIC TRANSFORMS (if needed)
    # ============================================
    
    # Static transform: ego_vehicle to imu_onBoard_link
    static_tf_ego_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_ego_to_imu',
        arguments=['0', '0', '0', '0', '0', '0',
                   'ego_vehicle', 'imu_onBoard_link']
    )
    
    return LaunchDescription([
        # Launch arguments
        ekf_variant_arg,
        use_sim_time_arg,
        base_frame_arg,
        odometry_output_arg,
        pose_output_arg,
        accel_output_arg,
        imu_input_arg,
        visual_odom_input_arg,
        
        # EKF nodes (only one will be active based on variant)
        ekf_node_advanced,
        ekf_node_calibrated,
        ekf_node_standalone,
        
        # Static transforms
        static_tf_ego_to_imu,
    ])