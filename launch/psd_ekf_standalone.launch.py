#!/usr/bin/env python3
"""
PSD EKF Standalone Backup Launch File
Launches robot_localization EKF as a standalone backup localization system.
This does NOT depend on SLAM and provides pose estimation using:
- IMU data for orientation and acceleration
- Visual odometry from ZED2 camera (optional)
- Control inputs from MPC (optional)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('robot_localization'),
        'params',
        'psd_ekf_standalone.yaml'
    ])
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    use_zed_odom_arg = DeclareLaunchArgument(
        'use_zed_odom',
        default_value='true',
        description='Use ZED2 visual odometry if available'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='ego_vehicle',
        description='Base frame of the robot (ego_vehicle or chassis)'
    )
    
    # EKF node - the main localization node
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
            # Input remappings
            ('imu/data', '/imu/data'),  # Xsens IMU
            ('odom', '/zed/zed_node/odom'),  # ZED visual odometry
            ('cmd_vel', '/cmd_vel'),  # Control input
            
            # Output remappings  
            ('odometry/filtered', '/odometry/filtered'),  # Main output
            ('accel/filtered', '/accel/filtered'),
        ]
    )
    
    # Static transform for IMU if needed
    # This connects imu_onBoard_link to the base frame
    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_to_base_tf',
        arguments=['0', '0', '0', '0', '0', '0', 
                   'ego_vehicle', 'imu_onBoard_link']
    )
    
    # Static transform from map to odom (identity initially)
    # This can be updated by a separate localization node if available
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 
                   'map', 'odom']
    )
    
    # Log info
    log_info = LogInfo(
        msg="""
        ========================================
        PSD EKF Standalone Backup Localization
        ========================================
        This node provides pose estimation using:
        - IMU: /imu/data (Xsens MTi-680)
        - Visual Odometry: /zed/zed_node/odom (if available)
        - Control: /cmd_vel (if available)
        
        Output: /odometry/filtered
        
        This is a BACKUP system that does not depend on SLAM.
        ========================================
        """
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        use_zed_odom_arg,
        base_frame_arg,
        
        # Info message
        log_info,
        
        # Nodes
        ekf_node,
        imu_static_tf,
        map_to_odom_tf,
    ])