#!/usr/bin/env python3
"""
EKF Calibration Tool for Drift Rejection
Analyzes sensor noise while stationary and automatically tunes EKF parameters
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np
import yaml
import time
from collections import deque
import os

class EKFCalibration(Node):
    def __init__(self):
        super().__init__('ekf_calibration')
        
        self.get_logger().info("EKF Calibration Tool Started")
        self.get_logger().info("Keep vehicle COMPLETELY STATIONARY during calibration!")
        
        # Parameters
        self.declare_parameter('calibration_duration', 30.0)  # seconds
        self.declare_parameter('config_file', 'psd_ekf_advanced.yaml')
        self.declare_parameter('output_file', 'psd_ekf_calibrated.yaml')
        
        self.calibration_duration = self.get_parameter('calibration_duration').value
        self.config_file = self.get_parameter('config_file').value
        self.output_file = self.get_parameter('output_file').value
        
        # Data buffers
        self.imu_data = {
            'orientation': deque(maxlen=1000),
            'angular_velocity': deque(maxlen=1000),
            'linear_acceleration': deque(maxlen=1000)
        }
        
        self.zed_imu_data = {
            'orientation': deque(maxlen=1000),
            'angular_velocity': deque(maxlen=1000),
            'linear_acceleration': deque(maxlen=1000)
        }
        
        self.zed_odom_data = {
            'position': deque(maxlen=1000),
            'orientation': deque(maxlen=1000),
            'linear_velocity': deque(maxlen=1000),
            'angular_velocity': deque(maxlen=1000)
        }
        
        # Noise statistics
        self.noise_stats = {}
        
        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        self.zed_imu_sub = self.create_subscription(
            Imu, '/zed/zed_node/imu/data', self.zed_imu_callback, 10)
        
        self.zed_odom_sub = self.create_subscription(
            Odometry, '/zed/zed_node/odom', self.zed_odom_callback, 10)
        
        # Start calibration timer
        self.start_time = time.time()
        self.calibration_timer = self.create_timer(1.0, self.check_calibration_progress)
        
        self.get_logger().info(f"Collecting data for {self.calibration_duration} seconds...")
    
    def quaternion_to_euler(self, q):
        """Convert quaternion to euler angles"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def imu_callback(self, msg):
        """Collect Xsens IMU data"""
        roll, pitch, yaw = self.quaternion_to_euler(msg.orientation)
        self.imu_data['orientation'].append([roll, pitch, yaw])
        
        self.imu_data['angular_velocity'].append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        self.imu_data['linear_acceleration'].append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
    
    def zed_imu_callback(self, msg):
        """Collect ZED IMU data"""
        roll, pitch, yaw = self.quaternion_to_euler(msg.orientation)
        self.zed_imu_data['orientation'].append([roll, pitch, yaw])
        
        self.zed_imu_data['angular_velocity'].append([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])
        
        self.zed_imu_data['linear_acceleration'].append([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
    
    def zed_odom_callback(self, msg):
        """Collect ZED visual odometry data"""
        self.zed_odom_data['position'].append([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        
        roll, pitch, yaw = self.quaternion_to_euler(msg.pose.pose.orientation)
        self.zed_odom_data['orientation'].append([roll, pitch, yaw])
        
        self.zed_odom_data['linear_velocity'].append([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        
        self.zed_odom_data['angular_velocity'].append([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])
    
    def check_calibration_progress(self):
        """Check if calibration is complete"""
        elapsed = time.time() - self.start_time
        remaining = self.calibration_duration - elapsed
        
        if remaining > 0:
            self.get_logger().info(f"Calibration in progress... {remaining:.1f}s remaining")
        else:
            self.get_logger().info("Data collection complete. Analyzing...")
            self.calibration_timer.cancel()
            self.analyze_noise()
    
    def analyze_noise(self):
        """Analyze sensor noise characteristics"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("SENSOR NOISE ANALYSIS")
        self.get_logger().info("="*60)
        
        # Analyze Xsens IMU
        if len(self.imu_data['orientation']) > 10:
            self.analyze_sensor_data('Xsens IMU', self.imu_data, 'imu0')
        
        # Analyze ZED IMU
        if len(self.zed_imu_data['orientation']) > 10:
            self.analyze_sensor_data('ZED IMU', self.zed_imu_data, 'imu1')
        
        # Analyze ZED Odometry
        if len(self.zed_odom_data['position']) > 10:
            self.analyze_sensor_data('ZED Odometry', self.zed_odom_data, 'odom0')
        
        # Generate optimized configuration
        self.generate_optimized_config()
    
    def analyze_sensor_data(self, sensor_name, data, sensor_id):
        """Analyze individual sensor data"""
        self.get_logger().info(f"\n{sensor_name}:")
        self.get_logger().info("-" * 40)
        
        self.noise_stats[sensor_id] = {}
        
        for data_type, values in data.items():
            if len(values) > 0:
                arr = np.array(values)
                
                # Calculate statistics
                mean = np.mean(arr, axis=0)
                std = np.std(arr, axis=0)
                max_val = np.max(np.abs(arr), axis=0)
                
                # Detect drift
                if len(values) > 100:
                    # Compare first 10% with last 10%
                    n = len(values) // 10
                    early = np.mean(arr[:n], axis=0)
                    late = np.mean(arr[-n:], axis=0)
                    drift = late - early
                else:
                    drift = np.zeros_like(mean)
                
                # Store statistics
                self.noise_stats[sensor_id][data_type] = {
                    'std': std.tolist(),
                    'max': max_val.tolist(),
                    'drift': drift.tolist()
                }
                
                # Report findings
                labels = ['x', 'y', 'z'] if len(std) == 3 else ['roll', 'pitch', 'yaw']
                self.get_logger().info(f"  {data_type}:")
                for i, label in enumerate(labels[:len(std)]):
                    self.get_logger().info(
                        f"    {label}: σ={std[i]:.6f}, max={max_val[i]:.6f}, drift={drift[i]:.6f}"
                    )
                
                # Warnings for high noise or drift
                if data_type == 'position' and np.any(std > 0.01):
                    self.get_logger().warn(f"    ⚠ High position noise detected!")
                
                if data_type == 'orientation' and np.any(std > 0.005):
                    self.get_logger().warn(f"    ⚠ High orientation noise detected!")
                
                if np.any(np.abs(drift) > 0.001):
                    self.get_logger().warn(f"    ⚠ Drift detected in {data_type}!")
    
    def generate_optimized_config(self):
        """Generate optimized EKF configuration based on noise analysis"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("GENERATING OPTIMIZED CONFIGURATION")
        self.get_logger().info("="*60)
        
        # Load base configuration
        config_path = os.path.expanduser(f"~/psd_ws/src/slam/robot_localization/params/{self.config_file}")
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        ekf_config = config['ekf_filter_node']['ros__parameters']
        
        # Update process noise based on measured noise
        process_noise = ekf_config['process_noise_covariance']
        
        # Optimize based on sensor noise
        if 'imu0' in self.noise_stats:
            # Orientation noise (indices 45, 60, 75 for roll, pitch, yaw)
            if 'orientation' in self.noise_stats['imu0']:
                orient_std = self.noise_stats['imu0']['orientation']['std']
                process_noise[45] = max(0.0001, orient_std[0]**2)  # roll
                process_noise[60] = max(0.0001, orient_std[1]**2)  # pitch
                process_noise[75] = max(0.0001, orient_std[2]**2)  # yaw
                
                self.get_logger().info(f"Updated orientation process noise: {orient_std}")
            
            # Angular velocity noise
            if 'angular_velocity' in self.noise_stats['imu0']:
                ang_vel_std = self.noise_stats['imu0']['angular_velocity']['std']
                process_noise[135] = max(0.001, ang_vel_std[0]**2)  # vroll
                process_noise[150] = max(0.001, ang_vel_std[1]**2)  # vpitch
                process_noise[165] = max(0.001, ang_vel_std[2]**2)  # vyaw
                
                self.get_logger().info(f"Updated angular velocity process noise: {ang_vel_std}")
        
        if 'odom0' in self.noise_stats:
            # Position noise
            if 'position' in self.noise_stats['odom0']:
                pos_std = self.noise_stats['odom0']['position']['std']
                process_noise[0] = max(0.001, pos_std[0]**2)   # x
                process_noise[15] = max(0.001, pos_std[1]**2)  # y
                
                # Adjust rejection thresholds based on noise
                if np.mean(pos_std) > 0.01:
                    ekf_config['odom0_pose_rejection_threshold'] = 5.0
                else:
                    ekf_config['odom0_pose_rejection_threshold'] = 2.0
                
                self.get_logger().info(f"Updated position process noise: {pos_std}")
        
        # Detect and compensate for drift
        drift_detected = False
        for sensor_id, sensor_data in self.noise_stats.items():
            for data_type, stats in sensor_data.items():
                if np.any(np.abs(stats['drift']) > 0.001):
                    drift_detected = True
                    self.get_logger().warn(f"Drift detected in {sensor_id} {data_type}")
        
        if drift_detected:
            # Enable drift compensation features
            ekf_config['smooth_lagged_data'] = True
            ekf_config['history_length'] = 5.0
            ekf_config['dynamic_process_noise_covariance'] = True
            self.get_logger().info("Enabled drift compensation features")
        
        # Adjust sensor trust based on noise levels
        self.adjust_sensor_trust(ekf_config)
        
        # Save optimized configuration
        output_path = os.path.expanduser(f"~/psd_ws/src/slam/robot_localization/params/{self.output_file}")
        
        # Write config with metadata as comments
        with open(output_path, 'w') as f:
            # Write metadata as comments
            f.write("### PSD EKF Calibrated Configuration ###\n")
            f.write(f"# Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"# Calibration duration: {self.calibration_duration} seconds\n")
            f.write("#\n")
            f.write("# Sensor Noise Statistics:\n")
            for sensor_id, sensor_data in self.noise_stats.items():
                f.write(f"# {sensor_id}:\n")
                for data_type, stats in sensor_data.items():
                    f.write(f"#   {data_type}:\n")
                    f.write(f"#     std: {stats['std']}\n")
                    f.write(f"#     max: {stats['max']}\n")
                    f.write(f"#     drift: {stats['drift']}\n")
            f.write("#\n")
            f.write("# Drift compensation: ENABLED\n" if drift_detected else "# Drift compensation: DISABLED\n")
            f.write("#\n\n")
            
            # Write the actual config
            yaml.dump(config, f, default_flow_style=None, sort_keys=False)
        
        self.get_logger().info(f"\n✓ Calibrated configuration saved to: {output_path}")
        self.print_recommendations()
    
    def adjust_sensor_trust(self, config):
        """Adjust sensor trust levels based on noise analysis"""
        # Calculate relative noise levels
        sensor_scores = {}
        
        # Score IMU orientation
        if 'imu0' in self.noise_stats and 'orientation' in self.noise_stats['imu0']:
            orient_noise = np.mean(self.noise_stats['imu0']['orientation']['std'])
            sensor_scores['imu0_orientation'] = 1.0 / (1.0 + orient_noise * 100)
        
        # Score visual odometry position
        if 'odom0' in self.noise_stats and 'position' in self.noise_stats['odom0']:
            pos_noise = np.mean(self.noise_stats['odom0']['position']['std'])
            sensor_scores['odom0_position'] = 1.0 / (1.0 + pos_noise * 10)
        
        # Adjust configurations based on scores
        if sensor_scores:
            best_sensor = max(sensor_scores, key=sensor_scores.get)
            self.get_logger().info(f"\nSensor trust scores: {sensor_scores}")
            self.get_logger().info(f"Most reliable sensor: {best_sensor}")
            
            # Boost trust in best sensor
            if 'imu0' in best_sensor:
                config['imu0_pose_rejection_threshold'] = 1.0
                config['imu0_twist_rejection_threshold'] = 1.0
            elif 'odom0' in best_sensor:
                config['odom0_pose_rejection_threshold'] = 5.0
                config['odom0_twist_rejection_threshold'] = 3.0
    
    def print_recommendations(self):
        """Print recommendations based on calibration"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("CALIBRATION RECOMMENDATIONS")
        self.get_logger().info("="*60)
        
        recommendations = []
        
        # Check for high noise
        for sensor_id, sensor_data in self.noise_stats.items():
            for data_type, stats in sensor_data.items():
                if np.any(np.array(stats['std']) > 0.01):
                    recommendations.append(
                        f"• {sensor_id}: High noise in {data_type} - check sensor mounting"
                    )
        
        # Check for drift
        drift_sensors = []
        for sensor_id, sensor_data in self.noise_stats.items():
            for data_type, stats in sensor_data.items():
                if np.any(np.abs(stats['drift']) > 0.001):
                    drift_sensors.append(f"{sensor_id}/{data_type}")
        
        if drift_sensors:
            recommendations.append(
                f"• Drift detected in: {', '.join(drift_sensors)} - ensure vehicle is stationary"
            )
        
        # Print recommendations
        if recommendations:
            for rec in recommendations:
                self.get_logger().warn(rec)
        else:
            self.get_logger().info("✓ All sensors operating within normal parameters")
        
        self.get_logger().info("\nNext steps:")
        self.get_logger().info("1. Review the calibrated configuration file")
        self.get_logger().info("2. Test with: ros2 launch robot_localization psd_ekf_calibrated.launch.py")
        self.get_logger().info("3. Monitor performance with: ros2 run robot_localization ekf_monitor.py")

def main(args=None):
    rclpy.init(args=args)
    
    calibration = EKFCalibration()
    
    try:
        rclpy.spin(calibration)
    except KeyboardInterrupt:
        calibration.get_logger().info("\nCalibration interrupted by user")
    finally:
        calibration.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()