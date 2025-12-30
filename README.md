# ROS 2 MPU6050 Sensor Fusion Node

![Rviz Visualization](./images/Screenshot%20from%202025-12-26%2017-28-21.png)
*Figure 1: Real-time visualization of the MPU6050 sensor orientation in Rviz2.*

This package implements a ROS 2 Lifecycle Node for the MPU6050 IMU sensor. It utilizes a custom C++ hardware driver for low-level I2C communication and performs real-time sensor fusion using selectable algorithms: **Complementary Filter** or **Madgwick AHRS**.

## Overview

This package implements a high-performance ROS 2 Lifecycle Node for the MPU6050 6-DOF IMU, designed for robotic applications requiring low latency and reliability.

At its core, the node utilizes a **custom C++ hardware driver** engineered for efficient, direct register access via the Linux I2C bus. By employing a template-based, dependency-injected architecture and I2C burst reads, the system ensures data consistency and minimal overhead without relying on heavy external libraries.

>**[Detailed Driver Documentation](./include/imu_sensor_cpp/driver_core/README.md)** > *Refer to the link above for a deep dive into the low-level I2C implementation, template architecture, and hardware calibration procedures.*

The node computes real-time sensor orientation (Roll, Pitch, Yaw) as a standard ROS `sensor_msgs/msg/Imu` quaternion message, offering the following key capabilities:

* **Lifecycle Management:** Fully implemented `rclcpp_lifecycle` state machine (Unconfigured → Inactive → Active), enabling deterministic startup and shutdown procedures.
* **Multi-Algorithm Sensor Fusion:** Dynamic switching between three estimation algorithms: **Complementary Filter**, **Madgwick AHRS**, and **Extended Kalman Filter (EKF)** with covariance tuning.
* **Drift Prevention & Stability:** Implements hardware calibration logic, gyroscope dead-zoning, and dynamic accelerometer confidence thresholds to maintain a drift-free horizon even during vibration or rapid movement.

## Fusion Algorithms

This node allows dynamic switching between algorithms via the `mode` parameter (`"comp"`, `"madg"` or `"ekf"`).

### 1. Complementary Filter

A lightweight filter that fuses high-frequency Gyroscope data with low-frequency Accelerometer data. It relies on the assumption that the gyroscope is precise in the short term, while the accelerometer provides a stable long-term gravity reference.

**Basic form**: 

$$\theta_{t} = \alpha \cdot (\theta_{t-1} + \omega_{gyro} \cdot \Delta t) + (1 - \alpha) \cdot \theta_{accel}$$

>Note: This implementation utilizes a modified formulation to robustly handle singularities and gimbal lock.

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

### 3. Extended Kalman Filter (EKF)

An industry-standard, probabilistic state estimation algorithm optimized for non-linear systems. Unlike the Complementary or Madgwick filters, the EKF maintains a **covariance matrix (P)** representing the system's uncertainty, allowing it to mathematically "fuse" sensor data based on statistical confidence rather than fixed weights.

**State Vector:**
The filter tracks the 4D Quaternion orientation state:

$$
x = [q_w, q_x, q_y, q_z]^T
$$

**Process Model (Prediction):**
The filter predicts the next state using the Gyroscope (angular velocity $\omega$) as the control input. Since 3D rotation is non-linear, the state transition is linearized using a **Jacobian Matrix (F)** at the current estimate.

$$
\dot{q} = \frac{1}{2} q_{k-1} \otimes \omega_{gyro}
$$

$$
P_{k|k-1} = F_{k-1} P_{k-1} F_{k-1}^T + Q
$$

*Where Q is the Process Noise Covariance Matrix (trust in Gyroscope).*

**Measurement Model (Correction):**
The predicted orientation is corrected using the Accelerometer data (gravity vector reference). The measurement model $h(x)$ predicts the expected gravity vector based on the current quaternion, and the difference (innovation) updates the state.

$$
K_k = P_{k|k-1} H_k^T (H_k P_{k|k-1} H_k^T + R)^{-1}
$$

$$
x_k = x_{k|k-1} + K_k (z_{accel} - h(x_{k|k-1}))
$$

*Where R is the Measurement Noise Covariance Matrix (trust in Accelerometer) and K is the Kalman Gain.*

**Key Features:**
* **Dynamic Covariance Tuning:** The matrices $Q$ (Process Noise) and $R$ (Measurement Noise) can be reconfigured at runtime via ROS 2 parameters. This allows for precise tuning between "smoothness" (high $R$) and "responsiveness" (low $R$).
* **Lazy Initialization:** The filter implements a "lazy" startup logic, ensuring the initial orientation is calculated from the first valid accelerometer reading before beginning the prediction cycle.
* **Normalization Constraint:** To prevent numerical drift inherent in iterative integration, the quaternion state is re-normalized ($|q|=1$) after every update step, ensuring valid rotation representation.

## Usage

Follow these steps to build and run the IMU EKF node. Navigate to your workspace root and compile:

```bash
colcon build --packages-select imu_sensor_cpp --symlink-install
source install/setup.bash
ros2 launch imu_sensor_cpp imu_launch.py
```

**Configuration Parameters**

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `mode` | string | `'ekf'` | Filter algorithm selection. <br>Options: `'ekf'`, `'comp'` (Complementary), `'madg'` (Madgwick). |
| `frame_id` | string | `'imu_link'` | The TF frame ID used in the header of published messages. |
| `gyro_deadzone` | double | `0.02` | **Noise Gate** (rad/s). Angular velocities below this value are forced to 0.0 to prevent drift when stationary. |
| `R` | double[] | `[6.0, 6.0, 6.0]` | **EKF Only**. Measurement Noise Covariance (Accel X, Y, Z). Higher values increase trust in the model over the sensor. Size must be 3. |
| `Q` | double[] | `[0.008, 0.008, 0.008, 0.008]` | **EKF Only**. Process Noise Covariance (Quaternion w, x, y, z). Size must be 4. |
| `beta` | double | `0.1` | **Madgwick Only**. Filter gain. Represents gyroscope measurement error. Must be `> 0.0`. |
| `alpha` | double | `0.98` | **Complementary Only**. Weighting factor. `0.98` means 98% trust in Gyro, 2% in Accel. Range: `[0.0, 1.0]`. |
| `magnitude_low_threshold` | double | `0.85` | **EKF & Comp**. Minimum normalized gravity magnitude (in g). Readings below this are ignored (freefall protection). |
| `magnitude_high_threshold` | double | `1.15` | **EKF & Comp**. Maximum normalized gravity magnitude (in g). Readings above this are ignored (linear acceleration protection). |
| `gimbal_lock_threshold` | double | `0.97` | **Complementary Only**. Cosine of the pitch angle used to detect vertical singularity.|
| `start_calibration` | bool | `false` | If `true`, triggers static IMU bias calibration on startup (blocking operation). |
| `delete_calibration_data` | bool | `false` | If `true`, deletes existing calibration data. |