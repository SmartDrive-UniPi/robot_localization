# Robot Localization EKF - PSD Backup System

## Overview
The robot_localization package provides a **backup localization system** for the PSD autonomous racing vehicle. It uses an Extended Kalman Filter (EKF) to fuse multiple sensor inputs for robust pose estimation without depending on SLAM.

## Quick Start

### 1. Calibrate First (RECOMMENDED)
```bash
# Park vehicle, keep completely stationary, then run:
ros2 launch robot_localization psd_ekf_calibration.launch.py

# After calibration, use the optimized configuration:
ros2 launch robot_localization psd_ekf_calibrated.launch.py
```

### 2. Basic Configuration (Simple Testing)
```bash
# For initial testing with minimal sensors
ros2 launch robot_localization psd_ekf_standalone.launch.py
```

### 3. Advanced Configuration (Racing/Production)
```bash
# For racing with multi-sensor fusion and drift reduction
ros2 launch robot_localization psd_ekf_advanced.launch.py

# With performance monitoring
ros2 run robot_localization ekf_monitor.py
```

## System Architecture

### Sensor Fusion Strategy
```
┌─────────────────┐     ┌──────────────────┐     ┌──────────────┐
│  Xsens MTi-680  │────▶│                  │◀────│  ZED2 Camera │
│  (Orientation)  │     │                  │     │   (Position) │
│  /imu/data      │     │   EKF Filter     │     │  /zed/odom   │
└─────────────────┘     │   (100 Hz)       │     └──────────────┘
                        │                  │
┌─────────────────┐     │                  │     ┌──────────────┐
│   ZED2 IMU      │────▶│                  │────▶│   Output     │
│ (Acceleration)  │     │                  │     │  /odometry/  │
│ /zed/imu/data   │     │                  │     │   filtered   │
└─────────────────┘     └──────────────────┘     └──────────────┘
```

### Sensor Responsibilities

| Sensor | Primary Use | Why | Trust Level |
|--------|------------|-----|-------------|
| **Xsens IMU** | Orientation & Angular Velocity | Professional-grade, minimal drift | HIGH |
| **ZED Visual Odometry** | Position & Linear Velocity | Visual SLAM prevents drift | HIGH |
| **ZED IMU** | Linear Acceleration | Backup for dead reckoning | MEDIUM |
| **MPC Control** | Motion Prediction | Known commands improve prediction | MEDIUM |

## Configuration Files

### 1. **Basic**: `psd_ekf_standalone.yaml`
- Single IMU + optional visual odometry
- 50 Hz update rate
- Good for testing and debugging

### 2. **Advanced**: `psd_ekf_advanced.yaml`
- Multi-sensor fusion
- 100 Hz update rate
- Optimized covariances for minimal drift
- Production-ready for racing

## Key Features for Drift Reduction

### 1. **High Update Rate (100 Hz)**
- Reduces integration errors
- Handles high-speed dynamics better

### 2. **Smart Sensor Selection**
- Uses each sensor's strengths
- Xsens for orientation (best gyros)
- ZED for position (visual prevents drift)
- Redundant IMUs for reliability

### 3. **Optimized Covariances**
```yaml
# Trust levels (lower = more trust)
Position noise: 0.01      # High trust in visual odometry
Orientation noise: 0.001   # Very high trust in Xsens
Velocity noise: 0.05       # Moderate trust
```

### 4. **Outlier Rejection**
- Automatically filters bad sensor data
- Configurable thresholds per sensor

## Performance Monitoring

### Real-time Monitor
```bash
# Run the performance monitor
ros2 run robot_localization ekf_monitor.py
```

Shows:
- Update rates for all sensors
- Drift indicators (covariance trends)
- Motion statistics
- Warnings for issues

### Expected Performance

| Metric | Target | Acceptable |
|--------|--------|------------|
| EKF Rate | 100 Hz | >80 Hz |
| Position Drift (100m) | <1m | <2m |
| Orientation Drift | <1° | <2° |
| Covariance Trend | Stable | Slowly increasing |

## Troubleshooting Guide

### Problem: Position Drifting

**Symptoms**: Position error accumulates over time

**Solutions**:
1. Check ZED odometry quality:
   ```bash
   ros2 topic hz /zed/zed_node/odom  # Should be >15 Hz
   ```

2. In `psd_ekf_advanced.yaml`, adjust:
   ```yaml
   # Increase trust in visual odometry
   odom0_pose_rejection_threshold: 5.0  # Increase from 3.0
   
   # Reduce position process noise
   process_noise_covariance[0]: 0.005   # Reduce from 0.01
   ```

### Problem: Orientation Drifting

**Symptoms**: Heading angle error accumulates

**Solutions**:
1. Ensure IMU calibration (keep stationary for 10s at startup)
2. Check IMU mounting (should be rigid, away from vibration)
3. Adjust in config:
   ```yaml
   # Increase IMU trust
   imu0_pose_rejection_threshold: 0.5  # Decrease from 0.8
   ```

### Problem: Velocity Jumps

**Symptoms**: Sudden velocity changes without acceleration

**Solutions**:
1. Enable smoothing:
   ```yaml
   smooth_lagged_data: true
   history_length: 5.0
   ```

2. Increase velocity noise tolerance:
   ```yaml
   process_noise_covariance[91]: 0.1   # vx
   process_noise_covariance[105]: 0.1  # vy
   ```

### Problem: Covariance Growing

**Symptoms**: Uncertainty continuously increases

**Solutions**:
1. Check sensor data rates (use monitor script)
2. Reduce process noise values
3. Check for sensor timeouts
4. Verify time synchronization

## Input/Output Topics

### Inputs
- `/imu/data` - Xsens IMU (orientation, angular velocity)
- `/zed/zed_node/odom` - ZED visual odometry (position, velocity)
- `/zed/zed_node/imu/data` - ZED IMU (acceleration)
- `/cmd_vel` - Control commands (optional)

### Outputs
- `/odometry/filtered` - Fused pose estimate
- `/accel/filtered` - Filtered acceleration
- `/tf` - Transform broadcasts (odom → ego_vehicle)

## Launch Parameters

```bash
# Basic parameters
use_sim_time:=false      # Use real/sim time
base_frame:=ego_vehicle  # Robot base frame
publish_tf:=true         # Publish transforms
debug:=false             # Enable debug output

# Example with parameters
ros2 launch robot_localization psd_ekf_advanced.launch.py \
  use_sim_time:=false \
  debug:=true
```

## Testing Procedure

### 1. Static Test (Stationary)
```bash
# Launch EKF
ros2 launch robot_localization psd_ekf_advanced.launch.py

# Monitor for 60 seconds
ros2 run robot_localization ekf_monitor.py

# Expected: <0.1m position drift, <1° orientation drift
```

### 2. Dynamic Test (Manual Driving)
- Drive a closed loop circuit
- Return to starting position
- Check final position error (<2% of distance traveled)

### 3. Racing Test
- Full acceleration/braking
- Sharp turns at speed
- Monitor covariance stability

## Comparison: Configurations

| Feature | Standalone | Advanced |
|---------|------------|----------|
| **Update Rate** | 50 Hz | 100 Hz |
| **Sensors** | 1 IMU | 2 IMUs + Visual |
| **Position Drift** | 5-10m/100m | <2m/100m |
| **CPU Usage** | 3% | 5% |
| **Use Case** | Testing | Racing |

## When to Use This Backup System

### Use EKF Backup When:
- Graph-SLAM fails or crashes
- CPU resources are limited
- Real-time guarantees needed
- Quick testing required
- Track has no loops (no loop closure benefit)

### Use Graph-SLAM When:
- Maximum accuracy needed
- Loop closure is beneficial
- Global consistency required
- CPU resources available

## Integration with PSD Stack

The EKF integrates seamlessly with:
- **Perception**: Uses cone detections via ZED
- **Planning**: Provides pose for trajectory planning
- **Control**: Uses MPC commands for prediction
- **Visualization**: Standard ROS topics for RViz

## Advanced Tuning Tips

### For Different Track Conditions

**Smooth Track** (low vibration):
- Decrease IMU acceleration noise
- Increase trust in all sensors

**Rough Track** (high vibration):
- Increase IMU acceleration noise
- Decrease trust in IMU acceleration
- Rely more on visual odometry

**High-Speed Sections**:
- Increase control gains
- Reduce sensor timeout
- Increase update rate if possible

### Dynamic Adjustments
```yaml
# Enable for varying conditions
dynamic_process_noise_covariance: true

# Adjust for sensor reliability
sensor_timeout: 0.05  # Decrease for reliable sensors

# Control latency compensation
control_timeout: 0.05  # Adjust based on system latency
```

## Common Warnings and Solutions

| Warning | Meaning | Solution |
|---------|---------|----------|
| "Low EKF rate" | Update rate <40 Hz | Check CPU load, reduce frequency |
| "Low IMU rate" | IMU data rate low | Check IMU driver, USB connection |
| "Covariance increasing" | Drift detected | Check sensor quality, adjust covariances |
| "X acceleration disabled" | Normal operation | EKF using IMU over control input |

## Automatic Calibration Tool

### Purpose
The calibration tool analyzes sensor noise while the vehicle is stationary to automatically tune EKF parameters for optimal drift rejection.

### How to Use

#### Step 1: Prepare Vehicle
1. Park on level ground
2. Turn off engine/motors
3. Ensure no vibrations (fans, AC, people walking)
4. Do NOT touch vehicle during calibration

#### Step 2: Run Calibration
```bash
# Default 30-second calibration
ros2 launch robot_localization psd_ekf_calibration.launch.py

# Custom duration (60 seconds for more accuracy)
ros2 launch robot_localization psd_ekf_calibration.launch.py duration:=60.0

# Calibrate specific config
ros2 launch robot_localization psd_ekf_calibration.launch.py \
  input_config:=psd_ekf_advanced.yaml \
  output_config:=psd_ekf_race_tuned.yaml
```

#### Step 3: Review Results
The tool will:
- Measure noise on all sensors
- Detect any drift
- Calculate optimal covariances
- Generate calibrated config file
- Provide recommendations

Example output:
```
SENSOR NOISE ANALYSIS
----------------------------------------
Xsens IMU:
  orientation:
    roll: σ=0.000012, max=0.000045, drift=0.000001
    pitch: σ=0.000015, max=0.000052, drift=0.000002
    yaw: σ=0.000025, max=0.000089, drift=0.000003
  
ZED Odometry:
  position:
    x: σ=0.002145, max=0.008234, drift=0.001234
    y: σ=0.001876, max=0.007123, drift=0.000987
    ⚠ Drift detected in position!

CALIBRATION RECOMMENDATIONS
========================================
✓ Xsens IMU operating within normal parameters
• odom0: High noise in position - check sensor mounting
• Drift compensation enabled
```

#### Step 4: Use Calibrated Config
```bash
# Launch with optimized parameters
ros2 launch robot_localization psd_ekf_calibrated.launch.py

# Monitor performance
ros2 run robot_localization ekf_monitor.py
```

### What the Calibration Does

1. **Noise Analysis**
   - Measures standard deviation of each sensor
   - Identifies maximum deviations
   - Detects drift over time

2. **Automatic Tuning**
   - Sets process noise based on measured noise
   - Adjusts rejection thresholds
   - Enables drift compensation if needed
   - Optimizes sensor trust levels

3. **Drift Detection**
   - Compares early vs late measurements
   - Identifies systematic bias
   - Enables smoothing if drift found

### When to Recalibrate

Recalibrate when:
- Sensors are remounted or replaced
- Persistent drift issues occur
- After major vehicle modifications
- Environmental conditions change significantly
- Before important races/tests

### Calibration Best Practices

1. **Environment**
   - Indoor or very calm conditions
   - Level surface
   - No nearby magnetic interference
   - Temperature similar to operating conditions

2. **Duration**
   - 30 seconds: Quick calibration
   - 60 seconds: Standard calibration
   - 120 seconds: High-precision calibration

3. **Validation**
   - Always test calibrated config
   - Compare with previous calibrations
   - Monitor for improvements

## Support and Debugging

### Debug Commands
```bash
# Check sensor rates
ros2 topic hz /imu/data
ros2 topic hz /zed/zed_node/odom

# Monitor covariance
ros2 topic echo /odometry/filtered | grep -A 36 "covariance:"

# Check transforms
ros2 run tf2_ros tf2_echo odom ego_vehicle

# View all diagnostics
ros2 topic echo /diagnostics_agg
```

### Best Practices
1. Always calibrate IMU before racing (10s stationary)
2. Monitor covariance trends during practice
3. Test configuration changes incrementally
4. Keep backup of working configurations
5. Document track-specific tunings

## Performance Benchmarks

### Minimum Requirements
- IMU rate: >50 Hz
- Visual odometry: >10 Hz
- EKF output: >40 Hz
- Position accuracy: <5% of distance
- Orientation accuracy: <5°

### Optimal Performance
- IMU rate: >100 Hz
- Visual odometry: >30 Hz
- EKF output: 100 Hz
- Position accuracy: <1% of distance
- Orientation accuracy: <1°

Remember: Small parameter changes can have significant effects. Always test incrementally and monitor performance!