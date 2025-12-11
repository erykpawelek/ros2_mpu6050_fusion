extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdexcept>
#include <cstdint>

#include "imu_sensor_cpp/mpu6050_driver.hpp"

namespace mpu6050cust_driver
{
    MPU6050CustomDriver::MPU6050CustomDriver(int addr, int adapter_nr)
    :
    addr_(addr),
    adapter_nr_(adapter_nr)
    {
        // Obtaining I2C bus file content
        char file_name[20];
        snprintf(file_name, 19, "/dev/i2c-%d", adapter_nr_);
        file_identyficator_ = open(file_name, O_RDWR);
        
        if (file_identyficator_ < 0){
            throw std::runtime_error("Unable to load system file");
        }
        // Choosing direct I2C bus addres in file thet we operate with
        if (ioctl(file_identyficator_, I2C_SLAVE, addr_) < 0){
            throw std::runtime_error("Unable to connect to specified address");
        }
        // Handling test register read
        auto test = readRegister(0x75);
        printf("Register reading test from 0x75 register (WHO_AM_I): %x", test);
        this->writeRegister(PWR_MGMT_1,0x00);
    }

    MPU6050CustomDriver::~MPU6050CustomDriver(){
        close(file_identyficator_);
    }

    uint8_t MPU6050CustomDriver::readRegister(uint8_t register_addr){
        // Reading single register value using SMBUS protocol.
        uint8_t register_value = i2c_smbus_read_byte_data(file_identyficator_, register_addr);
        if (register_value < 0){
            throw std::runtime_error("Unable to read data from register");
        }
        return static_cast<uint8_t>(register_value);
    }

    void MPU6050CustomDriver::writeRegister(uint8_t register_addr, uint8_t value){
        int feedback = i2c_smbus_write_byte_data(file_identyficator_, register_addr, value);
        if (feedback < 0){
            throw std::runtime_error("Unable to write data to register");
        }
    }

    int16_t MPU6050CustomDriver::readWord(uint8_t register_addr_h){
        // Obtaining 2 neighboring registers coresponding to one world
        uint8_t register_value_h = readRegister(register_addr_h);
        uint8_t register_value_l = readRegister(register_addr_h + 1);
        // Bitwise move to place corectly 2 bytes creating word
        int16_t register_value = (static_cast<int16_t>(register_value_h) << 8) | register_value_l;
        return register_value;
    }

    float MPU6050CustomDriver::getAccelerationX(){
        int16_t x_accel_register = readWord(ACCEL_XOUT_H);
        float accel_x = static_cast<float>(x_accel_register) / 16384.0;
        return accel_x;
    }
}


