#!/usr/bin/env python3
"""
PSD EKF Calibration Launch File
Launches the calibration tool to automatically tune EKF parameters
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    duration_arg = DeclareLaunchArgument(
        'duration',
        default_value='30.0',
        description='Calibration duration in seconds (keep vehicle stationary!)'
    )
    
    input_config_arg = DeclareLaunchArgument(
        'input_config',
        default_value='psd_ekf_advanced.yaml',
        description='Input configuration file to optimize'
    )
    
    output_config_arg = DeclareLaunchArgument(
        'output_config',
        default_value='psd_ekf_calibrated.yaml',
        description='Output calibrated configuration file'
    )
    
    # Calibration node
    calibration_node = Node(
        package='robot_localization',
        executable='ekf_calibration.py',
        name='ekf_calibration',
        output='screen',
        parameters=[{
            'calibration_duration': LaunchConfiguration('duration'),
            'config_file': LaunchConfiguration('input_config'),
            'output_file': LaunchConfiguration('output_config'),
        }]
    )
    
    # Instructions
    instructions = LogInfo(
        msg="""
        ╔════════════════════════════════════════════════════════════════╗
        ║                  EKF CALIBRATION PROCEDURE                      ║
        ╠════════════════════════════════════════════════════════════════╣
        ║                                                                  ║
        ║  IMPORTANT: Vehicle must be COMPLETELY STATIONARY!              ║
        ║                                                                  ║
        ║  1. Park vehicle on level ground                                ║
        ║  2. Turn off engine/motors                                      ║
        ║  3. Ensure no vibrations (fans, people walking nearby)          ║
        ║  4. Do NOT touch the vehicle during calibration                 ║
        ║                                                                  ║
        ║  The calibration will:                                          ║
        ║  • Measure sensor noise characteristics                         ║
        ║  • Detect any drift in sensors                                  ║
        ║  • Automatically tune covariance matrices                       ║
        ║  • Generate optimized configuration file                        ║
        ║                                                                  ║
        ║  Duration: 30 seconds (default)                                 ║
        ║                                                                  ║
        ╚════════════════════════════════════════════════════════════════╝
        """
    )
    
    return LaunchDescription([
        # Arguments
        duration_arg,
        input_config_arg,
        output_config_arg,
        
        # Show instructions
        instructions,
        
        # Run calibration
        calibration_node,
    ])