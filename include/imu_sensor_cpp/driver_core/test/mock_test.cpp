/**
 * Quick mock test platform for simle driver offline debugging
 */

#include <iostream>
#include <map>
#include <vector>
#include <cassert>
#include <iomanip>
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
        registers[reg_addr] = value;
        

        std::stringstream ss;
        ss << "WRITE [Reg: 0x" << std::hex << (int)reg_addr 
           << " Val: 0x" << (int)value << "]";
        operation_log.push_back(ss.str());
    }

    uint8_t readRegister(uint8_t dev_addr, uint8_t reg_addr) {
        
        std::stringstream ss;
        ss << "READ  [Reg: 0x" << std::hex << (int)reg_addr << "]";
        operation_log.push_back(ss.str());

        
        return registers[reg_addr];
    }

    int16_t readWord(uint8_t dev_addr, uint8_t reg_addr) {
        // Symulacja odczytu 2 bajtów (Big Endian)
        uint8_t h = registers[reg_addr];
        uint8_t l = registers[reg_addr + 1];
        return (int16_t(h) << 8) | l;
    }

    int readDataBlock(uint8_t dev_addr, uint8_t start_reg, uint8_t size, uint8_t* buffer) {
        for(int i=0; i<size; ++i) {
            buffer[i] = registers[start_reg + i];
        }
        return size; 
    }
};

// --- TEST SCENARIO ---

int main() {
    std::cout << "--- STARTING OFFLINE DRIVER TEST ---\n";

    MockI2C mock_i2c;

    std::cout << "[INFO] Initializing Driver...\n";
    
    try {
        mpu6050cust_driver::MPU6050CustomDriver<MockI2C> imu(
            mock_i2c, 
            0x68,
            mpu6050cust_driver::MPU6050CustomDriver<MockI2C>::DLPF_184_BAND,
            mpu6050cust_driver::MPU6050CustomDriver<MockI2C>::GYRO_RANGE_500,
            mpu6050cust_driver::MPU6050CustomDriver<MockI2C>::ACCEL_RANGE_4
        );
        
        std::cout << "[PASS] Driver initialized successfully (WHO_AM_I check passed).\n";

        auto accel = imu.getAccelerationX();
        assert(accel == 1.0);
        std::cout << "[PASS] ACCELERATION test: " << accel << std::endl;
       
        uint8_t config_val = mock_i2c.registers[0x1A];
        assert(config_val == 0x01);
        std::cout << "[PASS] CONFIG register set correctly to 0x01.\n";

        uint8_t gyro_val = mock_i2c.registers[0x1B];
        assert(gyro_val == 0x08);
        std::cout << "[PASS] GYRO_CONFIG register set correctly to 0x08.\n";

    } catch (const std::exception& e) {
        std::cerr << "[FAIL] Exception thrown: " << e.what() << "\n";
        return 1;
    }

    std::cout << "--- ALL TESTS PASSED ---\n";
    return 0;
}


