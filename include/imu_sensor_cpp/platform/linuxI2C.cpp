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
#include <cerrno>
#include <cstring>

#include "imu_sensor_cpp/core/mpu6050_driver.hpp"

namespace mpu6050cust_driver
{
    LinuxI2C::LinuxI2C(int adapter_nr)
    :
    adapter_nr_(adapter_nr)
    {
        // Obtaining I2C bus file content    
        char file_name[20];
        snprintf(file_name, 19, "/dev/i2c-%d", adapter_nr_);
        file_identyficator_ = open(file_name, O_RDWR);
        
        if (file_identyficator_ < 0){
            throw std::runtime_error("Unable to load system file");
        }
    }
    
    LinuxI2C::~LinuxI2C(){
        close(file_identyficator_);
    }

    LinuxI2C::LinuxI2C(LinuxI2C&& other) noexcept {
        this->file_identyficator_ = other.file_identyficator_;
        this->adapter_nr_ = other.adapter_nr_;

        other.file_identyficator_ = -1; 
    }

    void LinuxI2C::writeRegister(uint8_t dev_addr, uint8_t register_addr, uint8_t value ){
        // Choosing direct I2C bus addres in file thet we operate with
        if (ioctl(file_identyficator_, I2C_SLAVE, dev_addr) < 0){
            //throw std::runtime_error("Unable to connect to specified address");
            //debuging
            char err_msg[200];
            snprintf(err_msg, 199, "IOCTL Fail! Addr: 0x%02X, Errno: %d (%s)", 
            dev_addr, errno, strerror(errno));
            throw std::runtime_error(err_msg);

        }else{
            // Writing single register byte value using SMBUS protocol.
            int feedback = i2c_smbus_write_byte_data(file_identyficator_, register_addr, value);
            if (feedback < 0){
                throw std::runtime_error("Unable to write data to register");
            }
        } 
    }

    uint8_t LinuxI2C::readRegister(uint8_t dev_addr, uint8_t register_addr){
        // Choosing direct I2C bus addres in file thet we operate with
        if (ioctl(file_identyficator_, I2C_SLAVE, dev_addr) < 0){
            //throw std::runtime_error("Unable to connect to specified address");
            //debuging
            char err_msg[256];
            snprintf(err_msg, 255, "READ IOCTL Fail! Addr: 0x%02X, Errno: %d (%s)", 
                      dev_addr, errno, strerror(errno));
            throw std::runtime_error(err_msg);
        }else{
            // Reading single register byte value using SMBUS protocol.
            int16_t register_value = i2c_smbus_read_byte_data(file_identyficator_, register_addr);
            if (register_value < 0){
            throw std::runtime_error("Unable to read data from register");
            }
        return static_cast<uint8_t>(register_value);
        }
    }

    int16_t LinuxI2C::readWord(uint8_t dev_addr, uint8_t register_addr_h){
        // Obtaining 2 neighboring registers coresponding to one world
        uint8_t word_data[2];
        int feedback = readDataBlock(dev_addr, register_addr_h, 2, word_data);
        if (feedback < 0){
            throw std::runtime_error("Unable to read data block from register");
        }else{
        // Bitwise move to place corectly 2 bytes creating word
        // MPU6050 is Big-Endian, manual reconstruction required
        int16_t word = (static_cast<int16_t>(word_data[0]) << 8) | static_cast<int16_t>(word_data[1]);
        return word;
        }
    }

    int LinuxI2C::readDataBlock(uint8_t dev_addr, uint8_t start_register_addr, uint8_t size, uint8_t* buffor){
        if (ioctl(file_identyficator_, I2C_SLAVE, dev_addr) < 0){
            throw std::runtime_error("Unable to connect to specified address");
        }else{
            int feedback = i2c_smbus_read_i2c_block_data(
            file_identyficator_,
            start_register_addr,
            size,
            buffor);

            if (feedback < 0){
            throw std::runtime_error("Unable to read data block from register");
            }
            return feedback;
        }
    }
}