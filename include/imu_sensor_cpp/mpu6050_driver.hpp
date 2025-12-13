#ifndef MPU6050_DRIVER_HPP
#define MPU6050_DRIVER_HPP

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

namespace mpu6050cust_driver
{
    template <typename I2C_Interface>

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
        */
        MPU6050CustomDriver(
            I2C_Interface & bus,
            int dev_addr, 
            uint8_t dlpf_mode, 
            uint8_t gyro_range, 
            uint8_t accel_range)
            :
            i2c_(bus),
            dev_addr_(dev_addr)
            {
                auto test = readRegister(0x75); // Używamy nowej metody readRegister!
                printf("Register reading test from 0x75 register (WHO_AM_I): %x\n", test);
                config(dlpf_mode, gyro_range, accel_range);
            }

        /**
         * @brief Configures workparameters of MPU6050
         */    
        void config(uint8_t dlpf_mode, uint8_t gyro_range, uint8_t accel_range){
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

    /**
    * @brief Resets device and sets all parameters to it's default values
    */
    void reset(){
        
        writeRegister(PWR_MGMT_1, RESET_REGISTER_VALUE);
    }

    /**
    * @brief Wakes up MPU6050 from sleep mode.
    */
    void wakeUp(){
        uint8_t register_value = readRegister(PWR_MGMT_1);
        uint8_t value_to_write = register_value & WAKE_UP_REGISTER_VALUE;
        writeRegister(PWR_MGMT_1, value_to_write);
    }

    /**
    * @brief Puts MPU6050 into sleep mode.
    */
    void sleep(){
        uint8_t register_value = readRegister(PWR_MGMT_1);
        uint8_t value_to_write = register_value | SLEEP_REGISTER_VALUE;
        writeRegister(PWR_MGMT_1, value_to_write);
    }

    /**
    * @brief Aquires working temperature of MPU6050 module in Celcius degrees.
    * @return Value of temperature in degrees.
    */
    float getTemperature(){
        int16_t temp_register = readWord(TEMP_OUT_H);
        float temp_deg = (temp_register / 340.0) + 36.53;
        return temp_deg;  
    }


    uint8_t readRegister(uint8_t register_addr){
        uint8_t value = i2c_.readRegister(dev_addr_, register_addr);
        return value;
    }

    void writeRegister(uint8_t register_addr, uint8_t value){
        i2c_.writeRegister(dev_addr_, register_addr, value);
    }

    int16_t readWord(uint8_t register_addr_h){
        // Obtaining 2 neighboring registers coresponding to one world
        uint8_t register_value_h = readRegister(register_addr_h);
        uint8_t register_value_l = readRegister(register_addr_h + 1);
        // Bitwise move to place corectly 2 bytes creating word
        // MPU6050 is Big-Endian, manual reconstruction required
        int16_t register_value = (static_cast<int16_t>(register_value_h) << 8) | register_value_l;
        return register_value;
    }

    /**
    * @brief Aquires acceleration on X axe.
    * @return Raw value of acceleration on X axe.
    */
    float getAccelerationX(){
        int16_t accel_register_x = readWord(ACCEL_XOUT_H);
        float accel_x = static_cast<float>(accel_register_x) / accel_sensitivity_;
        return accel_x;
    }

    /**
    * @brief Aquires acceleration on Y axe.
    * @return Raw value of acceleration on Y axe.
    */
    float getAccelerationY(){
        int16_t accel_register_y = readWord(ACCEL_YOUT_H);
        float accel_y = static_cast<float>(accel_register_y) / accel_sensitivity_;
        return accel_y;
    }

    /**
    * @brief Aquires acceleration on Y axe.
    * @return Raw value of acceleration on Y axe.
    */
    float getAccelerationZ(){
        int16_t accel_register_z = readWord(ACCEL_ZOUT_H);
        float accel_z = static_cast<float>(accel_register_z) / accel_sensitivity_;
        return accel_z;
    }

    /**
    * @brief Aquire gyroscope value on X axe.
    * @return Raw value of gyroscope on X axe in defined metric (rad/deg).
    */
    float getGyroX(bool metrics){
        int16_t gyro_register_x = readWord(GYRO_XOUT_H);
        float gyro_x = static_cast<float>(gyro_register_x) / gyro_sensitivity_;
        if (metrics){
            gyro_x = (gyro_x / 360.0) * 2 * M_PI;
            return gyro_x;
        }
        return gyro_x;
    }

    /**
    * @brief Aquire gyroscope value on Y axe.
    * @return Raw value of gyroscope on Y axe in defined metric (rad/deg).
    */
    float getGyroY(bool metrics){
        int16_t gyro_register_y = readWord(GYRO_YOUT_H);
        float gyro_y = static_cast<float>(gyro_register_y) / gyro_sensitivity_;
        if (metrics){
            gyro_y = (gyro_y / 360.0) * 2 * M_PI;
            return gyro_y;
        }
        return gyro_y;
    }

    /**
    * @brief Aquire gyroscope value on Z axe.
    * @return Raw value of gyroscope on Z axe in defined metric (rad/deg).
    */
    float getGyroZ(bool metrics){
        int16_t gyro_register_z = readWord(GYRO_ZOUT_H);
        float gyro_z = static_cast<float>(gyro_register_z) / gyro_sensitivity_;
        if (metrics){
            gyro_z = (gyro_z / 360.0) * 2 * M_PI;
            return gyro_z;
        }
        return gyro_z;
    }

    /**
    * @brief Returns all IMU operational data in one package.
    * Uses Burst Read to efficiently aquire all operational data.
    * @param metrics Flag for choosing metric if true -> Radians.
    * @return ImuData struct containing accelerometer, gyroscope and temperature data.
    */
    ImuData getAllData(bool metrics){
        // Buffer for data
        uint8_t data[14];
        MPU6050CustomDriver::ImuData data_struct;
        // Burst read data
        int feedback = i2c_.readDataBlock(
            dev_addr_,
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

            // Recalculeting data to get right metrics
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

        // I2C interface template 
        I2C_Interface & i2c_;
        
        /**Adress in I2C system bus coresponding to device*/
        int dev_addr_;
    };

    class LinuxI2C 
    {
    public: 
        /**
        * @brief LinuxI2C class constructor.
        * Obtains handle to system file responsible for I2C communication.
        * @param adapter_nr Identification number of phisical I2C adapter/bus.
        * @exception std::runtime_error Thrown if the I2C adapter system file cannot be opened for read/write access.
        */
        LinuxI2C(int adapter_nr);

        /**
         * @brief LinuxI2C class destructor.
         * Closes the file handle to the system I2C bus opened in the constructor.
         */
        ~LinuxI2C();

        /**
         * @brief Writes one byte of data into defined register.
         * @param dev_addr Device address in opened system file.
         * @param register_addr Register address that is going to be overwritten.
         * @param value Value that is going to be written into choosen register.
         * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails).
         * @exception std::runtime_error Thrown if system is unable to overwrite register with specified value (i2c_smbus_write_byte_data) fails).
         */
        void writeRegister(uint8_t dev_addr, uint8_t register_addr, uint8_t value);

        /**
         * @brief Reads one byte of data from specified register address.
         * @param dev_addr Device address in opened system file.
         * @param register_addr Register address that is being read from.
         * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails).
         * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_byte_data) fails).
         * @return Value from register that has been read data in uint8_t format.
         */
        uint8_t readRegister(uint8_t dev_addr, uint8_t register_addr);
        
        /**
         * @brief Reads word data (2 bytes) according to Big-Endian concept.
         * @param dev_addr Device address in opened system file.
         * @param register_addr_h Start address of two byte data.
         * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_i2c_block_data() fails).
         * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails).
         * @return Value from two neighboring registers which create int16_t format.
         */
        int16_t readWord(uint8_t dev_addr, uint8_t register_addr_h);

        /**
         * @brief Reads specified number of bytes starting from defined start address.
         * @param dev_addr Device address in opened system file.
         * @param start_register_addr Start address from which reading begins.
         * @param size Size of data that is going to be read in bytes.
         * @param buffor Pointer to data buffer prepared for readed data.
         * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_i2c_block_data() fails).
         * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails).
         * @return The actual number of bytes read and placed into the buffer (success is indicated by a non-negative value).
         */
        int readDataBlock(uint8_t dev_addr, uint8_t start_register_addr, uint8_t size, uint8_t* buffor);

        private:

        /**Adapter number of I2C system file.*/
        int adapter_nr_;
        /**Handle to I2C bus system file.*/
        int file_identyficator_;

    };
}

#endif //MPU6050_DRIVER_HPP