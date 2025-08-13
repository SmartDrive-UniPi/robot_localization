#!/usr/bin/env python3
"""
EKF Performance Monitor
Monitors the EKF performance and helps identify drift issues
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
import numpy as np
import time

class EKFMonitor(Node):
    def __init__(self):
        super().__init__('ekf_monitor')
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', 
            self.odom_callback, 10)
        
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', 
            self.imu_callback, 10)
        
        self.zed_odom_sub = self.create_subscription(
            Odometry, '/zed/zed_node/odom',
            self.zed_odom_callback, 10)
        
        # State tracking
        self.last_odom_time = None
        self.last_position = None
        self.total_distance = 0.0
        self.max_velocity = 0.0
        self.max_acceleration = 0.0
        self.position_covariance_trace = []
        
        # Statistics
        self.imu_rate = 0
        self.odom_rate = 0
        self.zed_rate = 0
        self.last_imu_time = time.time()
        self.last_zed_time = time.time()
        self.imu_count = 0
        self.zed_count = 0
        
        # Create timer for periodic reporting
        self.timer = self.create_timer(2.0, self.report_status)
        
        self.get_logger().info('EKF Monitor started')
    
    def odom_callback(self, msg):
        """Monitor filtered odometry output"""
        current_time = self.get_clock().now()
        
        # Calculate rate
        if self.last_odom_time:
            dt = (current_time - self.last_odom_time).nanoseconds / 1e9
            self.odom_rate = 1.0 / dt if dt > 0 else 0
        
        self.last_odom_time = current_time
        
        # Track position
        position = np.array([msg.pose.pose.position.x, 
                           msg.pose.pose.position.y])
        
        if self.last_position is not None:
            distance = np.linalg.norm(position - self.last_position)
            self.total_distance += distance
        
        self.last_position = position
        
        # Track velocity
        velocity = np.sqrt(msg.twist.twist.linear.x**2 + 
                          msg.twist.twist.linear.y**2)
        self.max_velocity = max(self.max_velocity, velocity)
        
        # Monitor covariance (diagonal elements)
        cov_trace = sum([msg.pose.covariance[i*7] for i in range(6)])
        self.position_covariance_trace.append(cov_trace)
        
        # Keep only last 100 samples
        if len(self.position_covariance_trace) > 100:
            self.position_covariance_trace.pop(0)
    
    def imu_callback(self, msg):
        """Monitor IMU data"""
        self.imu_count += 1
        
        # Track max acceleration
        accel = np.sqrt(msg.linear_acceleration.x**2 + 
                       msg.linear_acceleration.y**2)
        self.max_acceleration = max(self.max_acceleration, accel)
    
    def zed_odom_callback(self, msg):
        """Monitor ZED odometry"""
        self.zed_count += 1
    
    def report_status(self):
        """Report system status"""
        current_time = time.time()
        
        # Calculate rates
        if self.imu_count > 0:
            dt_imu = current_time - self.last_imu_time
            self.imu_rate = self.imu_count / dt_imu if dt_imu > 0 else 0
            self.imu_count = 0
            self.last_imu_time = current_time
        
        if self.zed_count > 0:
            dt_zed = current_time - self.last_zed_time
            self.zed_rate = self.zed_count / dt_zed if dt_zed > 0 else 0
            self.zed_count = 0
            self.last_zed_time = current_time
        
        # Calculate covariance trend
        cov_trend = "STABLE"
        if len(self.position_covariance_trace) > 10:
            recent = np.mean(self.position_covariance_trace[-10:])
            older = np.mean(self.position_covariance_trace[-20:-10])
            if recent > older * 1.2:
                cov_trend = "INCREASING (drift likely)"
            elif recent < older * 0.8:
                cov_trend = "DECREASING (converging)"
        
        # Print report
        self.get_logger().info(
            f"\n{'='*50}\n"
            f"EKF Performance Monitor\n"
            f"{'='*50}\n"
            f"Update Rates:\n"
            f"  EKF Output:  {self.odom_rate:.1f} Hz\n"
            f"  IMU Input:   {self.imu_rate:.1f} Hz\n"
            f"  ZED Input:   {self.zed_rate:.1f} Hz\n"
            f"Motion Statistics:\n"
            f"  Total Distance:    {self.total_distance:.2f} m\n"
            f"  Max Velocity:      {self.max_velocity:.2f} m/s\n"
            f"  Max Acceleration:  {self.max_acceleration:.2f} m/s²\n"
            f"Covariance:\n"
            f"  Trend: {cov_trend}\n"
            f"  Current Trace: {self.position_covariance_trace[-1] if self.position_covariance_trace else 0:.6f}\n"
            f"{'='*50}"
        )
        
        # Warnings
        if self.odom_rate < 40:
            self.get_logger().warn(f"Low EKF rate: {self.odom_rate:.1f} Hz")
        
        if self.imu_rate < 50:
            self.get_logger().warn(f"Low IMU rate: {self.imu_rate:.1f} Hz")
        
        if cov_trend == "INCREASING (drift likely)":
            self.get_logger().warn("Covariance increasing - check sensor data quality")

def main(args=None):
    rclpy.init(args=args)
    monitor = EKFMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()