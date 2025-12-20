# ROS 2 MPU6050 Sensor Fusion Node

This package implements a ROS 2 Lifecycle Node for the MPU6050 IMU sensor. It utilizes a custom C++ hardware driver for low-level I2C communication and performs real-time sensor fusion using Complementary / Madgwick filter. 

## Overview

The node connects to the MPU6050 sensor via the Linux I2C bus, retrieves raw accelerometer and gyroscope data using burst reads, and computes the sensor orientation (Roll, Pitch, Yaw) in the form of a Quaternion.

Key capabilities:
* **Custom Hardware Driver**: Uses a template-based, dependency-injected driver for direct register access.
* **Lifecycle Management**: Fully implemented `rclcpp_lifecycle` state machine (Unconfigured -> Inactive -> Active).
* **Sensor Fusion**: Adaptive Complementary Filter with dynamic accelerometer weighting based on magnitude vectors.

## MPU6050 Custom Driver

This node relies on a custom C++ driver implementation that handles I2C protocol, register configuration, and data consistency (burst read).

**[ Detailed Driver Documentation ](./include/imu_sensor_cpp/driver_core/README.md)**

*Please refer to the link above for details regarding I2C implementation, template usage, and calibration procedures.*

## Fusion Algorithms

### Complementary filter:

* Fuses high-frequency Gyroscope data with low-frequency Accelerometer data.
* **Dynamic Weighting**: The filter monitors the magnitude of the acceleration vector. If significant external force (non-gravitational) is detected (magnitude deviates from 1g), the accelerometer input is ignored to prevent drift artifacts.
* **Singularity Handling**: Robustly handles mathematical discontinuities and wrap-around issues associated with Euler angle calculations.