# MPU6050 C++ Template Driver

A platform-agnostic, header-only C++ driver for MPU6050 IMU sensor.
Designed for portability across Linux (SMBus), STM32 (HAL), ESP32, and other embedded systems.

## Key Features
- **Hardware Abstraction**: Uses C++ Templates to decouple logic from I2C implementation.
- **Zero Dependencies**: Does not depend on ROS, Arduino, or specific HAL libraries.
- **Data Consistency**: Implements Burst Read (reading 14 bytes at once) to ensure accelerometers and gyroscopes represent the exact same moment in time. 
- **Software Calibration**: Includes methods for calculating and applying gyro/accel offsets to remove zero-drift.
- **Configurable**: Adjustable DLPF, Gyro Range, and Accelerometer Range.
- **Offline testing**: Fully offline quick unit test for math debuging purpouses.

## Offline Testing (Mocking)

This project includes a set of unit tests (Mock Tests) designed to verify the driver logic and algorithms without requiring physical hardware (Raspberry Pi / IMU). These tests check initialization, data reading, and error handling for the I2C communication.

To run the tests and view detailed results directly in the console, use the following command:

```bash
colcon test --packages-select imu_sensor_cpp --event-handlers console_direct+
```

## How to use (Porting Guide)

To use this driver on your platform (e.g., STM32), you need to create a simple wrapper class that satisfies the `I2C_Interface` contract.

### 1. Create your I2C Wrapper
Your wrapper class must implement these public methods:

```cpp
class MyPlatformI2C {
public:
    // Constructor: Initialize your hardware resources here (e.g., pass HAL handles)
    MyPlatformI2C(I2C_HandleTypeDef* hi2c); 

    // Basic I2C operations
    void writeRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t value);
    uint8_t readRegister(uint8_t dev_addr, uint8_t reg_addr);
    
    // Word reading (Handling Endianness if necessary)
    int16_t readWord(uint8_t dev_addr, uint8_t reg_addr);
    
    // Burst Read (Critical for IMU performance)
    // Should return the number of bytes read successfully
    int readDataBlock(uint8_t dev_addr, uint8_t start_reg, uint8_t size, uint8_t* buffer);
};
```
### 2. Instantiate the driver.
Inject your wrapper into template:

```cpp
#include "mpu6050_driver.hpp"
#include "linux_i2c_wrapper.hpp" // Or "stm32_i2c_wrapper.hpp"

int main() {
    // 1. Setup your low-level I2C interface
    // Example for Linux (opening /dev/i2c-1)
    LinuxI2C i2c_bus(1); 

    // 2. Create Sensor Object (Dependency Injection)
    // The driver is now specialized for LinuxI2C
    mpu6050cust_driver::MPU6050CustomDriver<LinuxI2C> imu(
        i2c_bus, 
        0x68, // Device Address
        mpu6050cust_driver::MPU6050CustomDriver<LinuxI2C>::DLPF_184_BAND,
        mpu6050cust_driver::MPU6050CustomDriver<LinuxI2C>::GYRO_RANGE_500,
        mpu6050cust_driver::MPU6050CustomDriver<LinuxI2C>::ACCEL_RANGE_4
    ); 

    // 3. Main Loop
    while(true) {
        auto data = imu.getAllData();
        // ... use data.accel_x, data.gyro_z etc.
    }
}
```

> [!NOTE] The driver constructor handles the device wake-up sequence automatically. No manual power management is required upon initialization.
