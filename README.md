# ROS 2 MPU6050 Sensor Fusion Node

![Rviz Visualization](./images/Screenshot%20from%202025-12-26%2017-28-21.png)
*Figure 1: Real-time visualization of the MPU6050 sensor orientation in Rviz2.*

This package implements a ROS 2 Lifecycle Node for the MPU6050 IMU sensor. It utilizes a custom C++ hardware driver for low-level I2C communication and performs real-time sensor fusion using selectable algorithms: **Complementary Filter** or **Madgwick AHRS**.

## Overview

The node connects to the MPU6050 sensor via the Linux I2C bus, retrieves raw accelerometer and gyroscope data using burst reads, and computes the sensor orientation (Roll, Pitch, Yaw) in the form of a standard ROS `sensor_msgs/msg/Imu` message (Quaternion).

**Key Capabilities:**
* **Custom Hardware Driver:** Uses a template-based, dependency-injected driver for direct register access and minimal overhead.
* **Lifecycle Management:** Fully implemented `rclcpp_lifecycle` state machine (Unconfigured → Inactive → Active).
* **Dual-Mode Sensor Fusion:** Supports both standard Complementary Filter and Madgwick Gradient Descent optimization.
* **Drift Prevention:** Implements dynamic accelerometer confidence thresholds to ignore non-gravitational acceleration spikes.

## MPU6050 Custom Driver

This node relies on a custom C++ driver implementation that handles I2C protocol, register configuration, and data consistency (burst read).

**[ Detailed Driver Documentation ](./include/imu_sensor_cpp/driver_core/README.md)**

*Please refer to the link above for details regarding I2C implementation, template usage, and hardware calibration procedures.*

## Fusion Algorithms

This node allows dynamic switching between algorithms via the `mode` parameter (`"comp"` or `"madgwick"`).

### 1. Complementary Filter

A lightweight filter that fuses high-frequency Gyroscope data with low-frequency Accelerometer data. It relies on the assumption that the gyroscope is precise in the short term, while the accelerometer provides a stable long-term gravity reference.

**Basic form**: 

$$\theta_{t} = \alpha \cdot (\theta_{t-1} + \omega_{gyro} \cdot \Delta t) + (1 - \alpha) \cdot \theta_{accel}$$

>But we are using modified formula which allows for better singularities handling.

**Core Equations:**

1.  **Prediction (Gyroscope):** Estimate the new angle based on angular velocity.
    $$\hat{\theta}_{pred} = \theta_{t-1} + \omega_{gyro} \cdot \Delta t$$
2.  **Error Calculation:** Determine the difference between the accelerometer measurement and the prediction.
    $$e = \theta_{accel} - \hat{\theta}_{pred}$$
3.  **Correction:** Apply a fraction of the error to correct the drift.
    $$\theta_{t} = \hat{\theta}_{pred} + (1 - \alpha) \cdot e$$



**Features:**
* **Dynamic Weighting:** The filter monitors the magnitude of the acceleration vector. If significant external force is detected (magnitude deviates from $1g$ thresholds), the accelerometer input is ignored $(\alpha = 1.0)$ to prevent drift artifacts caused by linear motion.
* **Singularity Handling:** Robustly handles discontinuities and wrap-around issues associated with Euler angle calculations.

### 2. Madgwick Filter (Gradient Descent)

An efficient orientation filter based on Sebastian Madgwick's IMU/MARG algorithm. It avoids the singularities of Euler angles by operating entirely in quaternion space and uses gradient descent to correct gyroscope drift.

**Core Equation:**
The estimated rate of change of the orientation $\dot{q}_{est}$ is calculated by fusing the gyroscope derivative with a gradient descent correction step:

$$\dot{q}_{est} = \dot{q}_{\omega} - \beta \cdot \frac{\nabla f}{||\nabla f||}$$

Where:
* $\dot{q}_{\omega}$ is the rate of change measured by the gyroscope.
* $\nabla f$ is the gradient of the error function defined by the difference between the measured gravity vector and the predicted gravity vector.
* $\beta$ (Beta) represents the divergence rate or "trust" in the accelerometer correction.

**Features:**
* **Gimbal Lock Free:** Operates using 4D Quaternions, eliminating mathematical singularities present in Euler-based filters.
* **Computational Efficiency:** Uses an optimized gradient descent formulation that requires significantly fewer operations than a Kalman Filter.
* **Tunable Responsiveness:** The $\beta$ gain allows fine-tuning between stability (low noise) and responsiveness (fast convergence).