/**
 * Quick mock test platform for simle driver offline debugging
 */

#include <iostream>
#include <map>
#include <vector>
#include <cassert>
#include <iomanip>

#include <gtest/gtest.h>
#include "imu_sensor_cpp/driver_core/mpu6050_driver.hpp" 


class MockI2C {
public:
    // Registers symulators
    std::map<uint8_t, uint8_t> registers;
    std::vector<std::string> operation_log;

    MockI2C() {
        
        registers[0x75] = 0x68; 
        // High and low byte in accel X register 
        registers[0x3B] = 0b00100000;
        registers[0x3C] = 0b00000000; 
    }

    void writeRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t value) {
        if (dev_addr != 0x68) {
             throw std::runtime_error("Invalid device address"); 
        } else {
            registers[reg_addr] = value;
    
            std::stringstream ss;
            ss << "WRITE [Reg: 0x" << std::hex << (int)reg_addr 
            << " Val: 0x" << (int)value << "]";
            operation_log.push_back(ss.str());
        }
       
    }

    uint8_t readRegister(uint8_t dev_addr, uint8_t reg_addr) {
        if (dev_addr != 0x68) {
             throw std::runtime_error("Invalid device address"); 
        } else {
            std::stringstream ss;
            ss << "READ  [Reg: 0x" << std::hex << (int)reg_addr << "]";
            operation_log.push_back(ss.str());

            return registers[reg_addr];
        }
    }

    int16_t readWord(uint8_t dev_addr, uint8_t reg_addr) {
        if (dev_addr != 0x68) {
             throw std::runtime_error("Invalid device address"); 
        } else {
            uint8_t h = registers[reg_addr];
            uint8_t l = registers[reg_addr + 1];
            uint16_t v = (int16_t(h) << 8) | l;

            std::stringstream ss;
            ss << "READ [Reg: 0x" << std::hex << (int)v << "]";
            operation_log.push_back(ss.str());

            return (int16_t(h) << 8) | l;
        }
       
    }

    int readDataBlock(uint8_t dev_addr, uint8_t start_reg, uint8_t size, uint8_t* buffer) {
        if (dev_addr != 0x68) {
             throw std::runtime_error("Invalid device address"); 
        } else {
            for(int i=0; i<size; ++i) {
                buffer[i] = registers[start_reg + i];
            }

            std::stringstream ss;
            ss << "READ BLOCK [Start Reg: 0x" << std::hex << (int)start_reg << " Size: " << std::dec << (int)size << "]";
            operation_log.push_back(ss.str());
            
            return size;
        }
    }
};

TEST(MPU6050Test, InitializationAndDataRead) {
    MockI2C mock_i2c;

    EXPECT_NO_THROW({
        mpu6050cust_driver::MPU6050CustomDriver<MockI2C> imu_driver(
            mock_i2c, 
            0x68,
            mpu6050cust_driver::MPU6050CustomDriver<MockI2C>::DLPF_184_BAND,
            mpu6050cust_driver::MPU6050CustomDriver<MockI2C>::GYRO_RANGE_500,
            mpu6050cust_driver::MPU6050CustomDriver<MockI2C>::ACCEL_RANGE_4
        );
    });

    mpu6050cust_driver::MPU6050CustomDriver<MockI2C> imu_driver(
        mock_i2c, 
        0x68,
        mpu6050cust_driver::MPU6050CustomDriver<MockI2C>::DLPF_184_BAND,
        mpu6050cust_driver::MPU6050CustomDriver<MockI2C>::GYRO_RANGE_500,
        mpu6050cust_driver::MPU6050CustomDriver<MockI2C>::ACCEL_RANGE_4
    );

    float accel_x = imu_driver.getAccelerationX();
    EXPECT_FLOAT_EQ(accel_x, 1.0f); 
}

TEST(MPU6050Test, InvalidAddress) {
    MockI2C mock_i2c;

    EXPECT_THROW({
        mpu6050cust_driver::MPU6050CustomDriver<MockI2C> imu_driver(
            mock_i2c, 
            0x99,
            mpu6050cust_driver::MPU6050CustomDriver<MockI2C>::DLPF_184_BAND,
            mpu6050cust_driver::MPU6050CustomDriver<MockI2C>::GYRO_RANGE_500,
            mpu6050cust_driver::MPU6050CustomDriver<MockI2C>::ACCEL_RANGE_4
        );
    }, std::runtime_error);
}




