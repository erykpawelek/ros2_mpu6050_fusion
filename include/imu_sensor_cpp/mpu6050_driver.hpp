#ifndef MPU6050_DRIVER_HPP
#define MPU6050_DRIVER_HPP

#include <cstdint>

namespace mpu6050cust_driver
{
    class MPU6050CustomDriver
    {
    public: 
        /**
        * @brief MPU6050CustomDriver class contructor.
        * * Initialization of data stream from file systems.
        * @param addr Adress in I2C data bus file which we want to connect to.
        * @param adapter_nr Adapter number of data bus file
        */
        MPU6050CustomDriver(int addr, int adapter_nr);

        /**
        * @brief MPU6050CustomDriver class destructor.
        * * Closes I2C data bus file.
        */
        ~MPU6050CustomDriver();

        /**
        * @brief Aquire acceleration on X axe.
        * @return Raw value of acceleration on X axe.
        */
        float getAccelerationX();

    private:
        // Accelerometer registers adresses
        static constexpr uint8_t ACCEL_XOUT_H = 0x3B;
        static constexpr uint8_t ACCEL_XOUT_L = 0x3C;
        static constexpr uint8_t ACCEL_YOUT_H = 0x3D;
        static constexpr uint8_t ACCEL_YOUT_L = 0x3E;
        static constexpr uint8_t ACCEL_ZOUT_H = 0x3F;
        static constexpr uint8_t ACCEL_ZOUT_L = 0x40;
        // Gyroscope registers adresses
        static constexpr uint8_t GYRO_XOUT_H = 0x43;
        static constexpr uint8_t GYRO_XOUT_L = 0x44;
        static constexpr uint8_t GYRO_YOUT_H = 0x45;
        static constexpr uint8_t GYRO_YOUT_L = 0x46;
        static constexpr uint8_t GYRO_ZOUT_H = 0x47;
        static constexpr uint8_t GYRO_ZOUT_L = 0x48;
        // Temperature registers adresses
        static constexpr uint8_t TEMP_OUT_H = 0x41;
        static constexpr uint8_t TEMP_OUT_L = 0x42;

        static constexpr uint8_t PWR_MGMT_1 = 0x6B;


        /**Handle to I2C bus system file.*/
        int file_identyficator_;
        /**Adapter number of I2C system file.*/
        int adapter_nr_;
        /**Adress in I2C system bus coresponding to device*/
        int addr_;
        
        /**
        * @brief Reads defined register value.
        * @param register_addr Adress of register to be read.
        * @return Value contained in choosen register.
        */
        uint8_t readRegister(uint8_t register_addr);

        /**
        * @brief Writes defined value to defined register.
        * @param register_addr Adress of register to be overwriten.
        * @param value Vale to be written to the register.
        */
        void writeRegister(uint8_t register_addr, uint8_t value);

        /**
        * @brief Reads word value from 2 registers.
        * @param register_addr_h Adress of higher part of register.
        * @return Value contained in choosen registers.
        */
        int16_t readWord(uint8_t register_addr_h);
    };
}

#endif //MPU6050_DRIVER_HPP