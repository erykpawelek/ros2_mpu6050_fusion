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
#include <math.h>

#include "imu_sensor_cpp/mpu6050_driver.hpp"

namespace mpu6050cust_driver
{
    MPU6050CustomDriver::MPU6050CustomDriver(
        int addr,
         int adapter_nr,
         uint8_t dlpf_mode, 
         uint8_t gyro_range, 
         uint8_t accel_range)
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
            // Closing file during exception handling for leackage avoidance
            close(file_identyficator_);
            throw std::runtime_error("Unable to connect to specified address");
        }
        // Handling test register read
        auto test = readRegister(0x75);
        printf("Register reading test from 0x75 register (WHO_AM_I): %x", test);
        config(dlpf_mode, gyro_range, accel_range);
    }

    MPU6050CustomDriver::~MPU6050CustomDriver(){
        close(file_identyficator_);
    }

    void MPU6050CustomDriver::config(uint8_t dlpf_mode, uint8_t gyro_range, uint8_t accel_range){
        // Cacluclating sensitivity for each defined operating gyroscope range.
        switch (gyro_range)
        {
        case GYRO_RANGE_250:
            gyro_sensitivity_ = 32768.0 / 250.0;
            break;

        case GYRO_RANGE_500:
            gyro_sensitivity_ =  32768.0 / 500.0;
            break;

        case GYRO_RANGE_1000:
            gyro_sensitivity_ =  32768.0 / 1000.0;
            break;

        case GYRO_RANGE_2000:
            gyro_sensitivity_ =  32768.0 / 2000.0;
            break;

        default:
            throw std::runtime_error("Wrong gyroscope configuration value, use preset values.");
        }

        // Cacluclating sensitivity for each defined operating accelerometer range.
                switch (accel_range)
        {
        case ACCEL_RANGE_2:
            accel_sensitivity_ = 32768.0 / 2.0;
            break;

        case ACCEL_RANGE_4:
            accel_sensitivity_ =  32768.0 / 4.0;
            break;

        case ACCEL_RANGE_8:
            accel_sensitivity_ =  32768.0 / 8.0;
            break;

        case ACCEL_RANGE_16:
            accel_sensitivity_ =  32768.0 / 16.0;
            break;

        default:
            throw std::runtime_error("Wrong accelerometer configuration value, use preset values.");
        }
        // Writing configuration values to registers
        writeRegister(CONFIG, dlpf_mode);
        writeRegister(GYRO_CONFIG, gyro_range);
        writeRegister(ACCEL_CONFIG, accel_range);
    }

    void MPU6050CustomDriver::reset(){
        
        writeRegister(PWR_MGMT_1, RESET_REGISTER_VALUE);
    }

    void MPU6050CustomDriver::wakeUp(){
        uint8_t register_value = readRegister(PWR_MGMT_1);
        uint8_t value_to_write = register_value & WAKE_UP_REGISTER_VALUE;
        writeRegister(PWR_MGMT_1, value_to_write);
    }

    void MPU6050CustomDriver::sleep(){
        uint8_t register_value = readRegister(PWR_MGMT_1);
        uint8_t value_to_write = register_value | SLEEP_REGISTER_VALUE;
        writeRegister(PWR_MGMT_1, value_to_write);
    }

    float MPU6050CustomDriver::getTemperature(){
        int16_t temp_register = readWord(TEMP_OUT_H);
        float temp_deg = (temp_register / 340.0) + 36.53;
        return temp_deg;  
    }


    uint8_t MPU6050CustomDriver::readRegister(uint8_t register_addr){
        // Reading single register value using SMBUS protocol.
        int16_t register_value = i2c_smbus_read_byte_data(file_identyficator_, register_addr);
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
        int16_t accel_register_x = readWord(ACCEL_XOUT_H);
        float accel_x = static_cast<float>(accel_register_x) / accel_sensitivity_;
        return accel_x;
    }

    float MPU6050CustomDriver::getAccelerationY(){
        int16_t accel_register_y = readWord(ACCEL_YOUT_H);
        float accel_y = static_cast<float>(accel_register_y) / accel_sensitivity_;
        return accel_y;
    }

    float MPU6050CustomDriver::getAccelerationZ(){
        int16_t accel_register_z = readWord(ACCEL_ZOUT_H);
        float accel_z = static_cast<float>(accel_register_z) / accel_sensitivity_;
        return accel_z;
    }

    float MPU6050CustomDriver::getGyroX(bool metrics){
        int16_t gyro_register_x = readWord(GYRO_XOUT_H);
        float gyro_x = static_cast<float>(gyro_register_x) / gyro_sensitivity_;
        if (metrics){
            gyro_x = (gyro_x / 360.0) * 2 * M_PI;
            return gyro_x;
        }
        return gyro_x;
    }

    float MPU6050CustomDriver::getGyroY(bool metrics){
        int16_t gyro_register_y = readWord(GYRO_YOUT_H);
        float gyro_y = static_cast<float>(gyro_register_y) / gyro_sensitivity_;
        if (metrics){
            gyro_y = (gyro_y / 360.0) * 2 * M_PI;
            return gyro_y;
        }
        return gyro_y;
    }

    float MPU6050CustomDriver::getGyroZ(bool metrics){
        int16_t gyro_register_z = readWord(GYRO_ZOUT_H);
        float gyro_z = static_cast<float>(gyro_register_z) / gyro_sensitivity_;
        if (metrics){
            gyro_z = (gyro_z / 360.0) * 2 * M_PI;
            return gyro_z;
        }
        return gyro_z;
    }

    MPU6050CustomDriver::ImuData MPU6050CustomDriver::getAllData(bool metrics){
        // Buffer for data
        uint8_t data[14];
        MPU6050CustomDriver::ImuData data_struct;
        // Burst read data
        int feedback = i2c_smbus_read_i2c_block_data(
            file_identyficator_,
            MPU6050CustomDriver::ACCEL_XOUT_H,
            14,
            data);
        if (feedback < 0){
            throw std::runtime_error("Unable to read data block from register");
        }else{
            // Obtaining accelerometer data
            int16_t accel_register_x = (static_cast<int16_t>(data[0]) << 8 | static_cast<int16_t>(data[1]));
            int16_t accel_register_y = (static_cast<int16_t>(data[2]) << 8 | static_cast<int16_t>(data[3]));
            int16_t accel_register_z = (static_cast<int16_t>(data[4]) << 8 | static_cast<int16_t>(data[5]));
            // Obtaining temperature data
            int16_t temp_register = (static_cast<int16_t>(data[6]) << 8 | static_cast<int16_t>(data[7]));
            // Obtaining gyroscope data
            int16_t gyro_register_x = (static_cast<int16_t>(data[8]) << 8 | static_cast<int16_t>(data[9]));
            int16_t gyro_register_y = (static_cast<int16_t>(data[10]) << 8 | static_cast<int16_t>(data[11]));
            int16_t gyro_register_z = (static_cast<int16_t>(data[12]) << 8 | static_cast<int16_t>(data[13]));

            // Recalculeting data to get wright metrics
            data_struct.accel_x = static_cast<float>(accel_register_x) / accel_sensitivity_;
            data_struct.accel_y = static_cast<float>(accel_register_y) / accel_sensitivity_;
            data_struct.accel_z = static_cast<float>(accel_register_z) / accel_sensitivity_;

            data_struct.temperature = (temp_register / 340.0) + 36.53;

            if(metrics){
                data_struct.gyro_x = ((static_cast<float>(gyro_register_x) / gyro_sensitivity_) / 360.0) * 2 * M_PI;
                data_struct.gyro_y = ((static_cast<float>(gyro_register_y) / gyro_sensitivity_) / 360.0) * 2 * M_PI;
                data_struct.gyro_z = ((static_cast<float>(gyro_register_z) / gyro_sensitivity_) / 360.0) * 2 * M_PI;
            }else{
                data_struct.gyro_x = static_cast<float>(gyro_register_x) / gyro_sensitivity_;
                data_struct.gyro_y = static_cast<float>(gyro_register_y) / gyro_sensitivity_;
                data_struct.gyro_z = static_cast<float>(gyro_register_z) / gyro_sensitivity_;
            }
            return data_struct;
        }
    }
    
}


