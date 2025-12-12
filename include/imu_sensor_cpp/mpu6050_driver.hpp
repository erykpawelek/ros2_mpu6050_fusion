#ifndef MPU6050_DRIVER_HPP
#define MPU6050_DRIVER_HPP

#include <cstdint>

namespace mpu6050cust_driver
{
    class MPU6050CustomDriver
    {
    public: 
        /**Degrees metric flag */
        static constexpr bool DEG = false;
        /**Radians metic flag */
        static constexpr bool RAD = true;

        /**Digital low pas filter bandwidth flag*/
        static constexpr uint8_t DLPF_184_BAND = 0b00000001;
        /**Digital low pas filter bandwidth flag*/
        static constexpr uint8_t DLPF_94_BAND = 0b00000010;

        /**Gyroscope range +-250 deg register preset */
        static constexpr uint8_t GYRO_RANGE_250 = 0b00000000;
        /**Gyroscope range +-500 deg register preset */
        static constexpr uint8_t GYRO_RANGE_500 = 0b00001000;
        /**Gyroscope range +-1000 deg register preset */
        static constexpr uint8_t GYRO_RANGE_1000 = 0b00010000;
        /**Gyroscope range +-2000 deg register preset */
        static constexpr uint8_t GYRO_RANGE_2000 = 0b00011000;

        /**Accelerometer range +-2 deg register preset */
        static constexpr uint8_t ACCEL_RANGE_2 = 0b00000000;
        /**Accelerometer range +-4 deg register preset */
        static constexpr uint8_t ACCEL_RANGE_4 = 0b00001000;
        /**Accelerometer range +-8 deg register preset */
        static constexpr uint8_t ACCEL_RANGE_8 = 0b00010000;
        /**Accelerometer range +-16 deg register preset */
        static constexpr uint8_t ACCEL_RANGE_16 = 0b00011000;

        /** Acceleration data sesitivity */
        float accel_sensitivity_;
        /** Gyroscope data sensitivity */
        float gyro_sensitivity_;
        /** Data struct containing accelerometer, gyroscope and temperature data */
        struct ImuData {
            float accel_x;
            float accel_y;
            float accel_z;

            float temperature;

            float gyro_x;
            float gyro_y;
            float gyro_z;
        };
        


        /**
        * @brief MPU6050CustomDriver class contructor.
        * * Initialization of data stream from file systems.
        * @param addr Adress in I2C data bus file which we want to connect to.
        * @param adapter_nr Adapter number of data bus file
        */
        MPU6050CustomDriver(
            int addr,
            int adapter_nr, 
            uint8_t dlpf_mode, 
            uint8_t gyro_range, 
            uint8_t accel_range);

        /**
        * @brief MPU6050CustomDriver class destructor.
        * * Closes I2C data bus file.
        */
        ~MPU6050CustomDriver();

        /**
         * @brief Configures workparameters of MPU6050
         */
        void config(uint8_t dlpf_mode, uint8_t gyro_range, uint8_t accel_range);

        /**
         * @brief Resets device and sets all parameters to it's default values
         */
        void reset();

        /**
        * @brief Wakes up MPU6050 from sleep mode.
        */
        void wakeUp();

        /**
        * @brief Puts MPU6050 into sleep mode.
        */
        void sleep();

        /**
        * @brief Aquires working temperature of MPU6050 module in Celcius degrees.
        * @return Value of temperature in degrees.
        */
        float getTemperature();

        /**
        * @brief Aquires acceleration on X axe.
        * @return Raw value of acceleration on X axe.
        */
        float getAccelerationX();

        /**
        * @brief Aquires acceleration on Y axe.
        * @return Raw value of acceleration on Y axe.
        */
        float getAccelerationY();

        /**
        * @brief Aquires acceleration on z axe.
        * @return Raw value of acceleration on Z axe.
        */
        float getAccelerationZ();
        /**
        * @brief Aquires acceleration on Z axe.
        * @return Raw value of acceleration on Z axe.
        */

        /**
        * @brief Aquire gyroscope value on X axe.
        * @return Raw value of gyroscope on X axe in defined metric (rad/deg).
        */
        float getGyroX(bool metrics = true);

        /**
        * @brief Aquire gyroscope value on Y axe.
        * @return Raw value of gyroscope on Y axe in defined metric (rad/deg).
        */
        float getGyroY(bool metrics = true);

        /**
        * @brief Aquire gyroscope value on Z axe.
        * @return Raw value of gyroscope on Z axe in defined metric (rad/deg).
        */
        float getGyroZ(bool metrics = true);
        
        /**
         * @brief Returns all IMU operational data in one package.
         * Uses Burst Read to efficiently aquire all operational data.
         * @param metrics Flag for choosing metric if true -> Radians.
         * @return ImuData struct containing accelerometer, gyroscope and temperature data.
         */
        ImuData getAllData(bool metrics = true);


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
        // PWR_MGMT_1 register adress responible for sleep/wake up modes.
        static constexpr uint8_t PWR_MGMT_1 = 0x6B;
        // CONFIG registers
        static constexpr uint8_t CONFIG = 0x1A;
        static constexpr uint8_t GYRO_CONFIG = 0x1B;
        static constexpr uint8_t ACCEL_CONFIG = 0x1C;
        // IMU module control
        static constexpr uint8_t WAKE_UP_REGISTER_VALUE = 0b10111111;
        static constexpr uint8_t SLEEP_REGISTER_VALUE = 0b01000000;
        static constexpr uint8_t RESET_REGISTER_VALUE = 0b10000000;


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