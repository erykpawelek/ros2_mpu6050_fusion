#ifndef MPU6050_DRIVER_HPP
#define MPU6050_DRIVER_HPP

extern "C" {
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}

#include <fcntl.h>
#include <sys/ioctl.h>
#include <cstdio>
#include <unistd.h>
#include <stdexcept>
#include <cstdint>

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
        /**PI const */
        static constexpr float PI = 3.1415926535;

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
        * @param bus Reference to the I2C interface object (dependency injection).
        * @param dev_addr Device address in opened system file.
        * @param dlpf_mode Digital low passfilter mode build into MPU6050 module. Note that class MPU6050CustomDriver contains two preset const values which are commonly used, for more detailed setup look into MPU6050 register map.
        * @param gyro_range Gyroscope values range.
        * @param accel_range Accelerometer values range.
        * * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_i2c_block_data() fails), exception from test read from WHO_AM_I register.
         * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails), exception from test read from WHO_AM_I register.
         * @exception std::runtime_error Thrown if device id (WHO_AM_I register value) do not match expected.
        */
        MPU6050CustomDriver(
            I2C_Interface & bus,
            int dev_addr, 
            uint8_t dlpf_mode, 
            uint8_t gyro_range, 
            uint8_t accel_range)
            :
            i2c_(bus),
            dev_addr_(dev_addr),
            offset_accel_x_(0.0),
            offset_accel_y_(0.0),
            offset_accel_z_(0.0),
            offset_gyro_x_(0.0),
            offset_gyro_y_(0.0),
            offset_gyro_z_(0.0)
            {
                auto test = readRegister(WHO_AM_I_REG); 
                if(test != 0 && (test == 0x98 || test == 0x68)){
                    printf("Register reading test from 0x75 register (WHO_AM_I): %x\n", test);
                    wakeUp();
                    usleep(100000);
                    config(dlpf_mode, gyro_range, accel_range);
                }else{
                    throw std::runtime_error("Device ID test error");
                }
            }   

        /**
         * @brief Configures workparameters of mpu6050 module.
         * @param dlpf_mode Digital low passfilter mode build into MPU6050 module. Note that class MPU6050CustomDriver contains two preset const values which are commonly used, for more detailed setup look into MPU6050 register map.
         * @param gyro_range Gyroscope values range.
         * @param accel_range Accelerometer values range.
         * @exception std::runtime_error Thrown if user defines incorect gyro_range value.
         * @exception std::runtime_error Thrown if user defines incorect accel_range value.
         * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails), during register write.
         * @exception std::runtime_error Thrown if system is unable to overwrite a register with specified value (i2c_smbus_write_byte_data fails).
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
     * @brief Calculate offset calues for each axis.
     * @param samples Number of samples to be sampled during calibration.
     * @param sampling_freq Sampling frequency of measurement.
     * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_i2c_block_data() fails in getAllData function).
    * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails in   getAllData function).
     */
    void calibrate(int samples = 1000, int sampling_freq = 100){ 
        wakeUp();
        float sum_accel_x = 0.0f, sum_accel_y = 0.0f, sum_accel_z = 0.0f;
        float sum_gyro_x = 0.0f,  sum_gyro_y = 0.0f,  sum_gyro_z = 0.0f;
        
        // Calculating duration of one cycle
        int delay_us = 1000000 / sampling_freq; 
        
        float duration = (float)samples / sampling_freq;
        printf("Calibration started, it will take: %.1f s\n", duration); 

        for (int i = 0; i < samples; ++i){
            ImuData measurement_data = getAllData(false); 

            sum_accel_x += measurement_data.accel_x;
            sum_accel_y += measurement_data.accel_y;
            sum_accel_z += measurement_data.accel_z;

            sum_gyro_x += measurement_data.gyro_x;
            sum_gyro_y += measurement_data.gyro_y;
            sum_gyro_z += measurement_data.gyro_z;

            usleep(delay_us);
        }

        // Means calculations
        offset_accel_x_ = sum_accel_x / samples;
        offset_accel_y_ = sum_accel_y / samples;
        
        // Considering gravitation
        offset_accel_z_ = (sum_accel_z / samples) - 1.0f; 

        offset_gyro_x_ = sum_gyro_x / samples;
        offset_gyro_y_ = sum_gyro_y / samples;
        offset_gyro_z_ = sum_gyro_z / samples;

        printf("Calibration complete\n");
        printf("Accel Offsets: X: %.3f, Y: %.3f, Z: %.3f\n", offset_accel_x_, offset_accel_y_, offset_accel_z_);
        printf("Gyro Offsets:  X: %.3f, Y: %.3f, Z: %.3f\n", offset_gyro_x_, offset_gyro_y_, offset_gyro_z_);
    }
    /**
     * @brief Delete calibration offsets data.
     */
    void delete_calibration_data(){
        offset_accel_x_ = 0.0;
        offset_accel_y_ = 0.0;
        offset_accel_z_ = 0.0;
        offset_gyro_x_ = 0.0;
        offset_gyro_y_ = 0.0;
        offset_gyro_z_ = 0.0;
    }

    /**
    * @brief Resets device and sets all parameters to it's default values
    * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails in writeRegister function).
    * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_byte_data) fails in writeRegister function).
    */
    void reset(){
        writeRegister(PWR_MGMT_1, RESET_REGISTER_VALUE);
    }

    /**
    * @brief Wakes up MPU6050 from sleep mode.
    * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails in writeRegister function).
    * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_byte_data) fails in writeRegister function).
    * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails in readRegister function).
    * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_byte_data) fails in readRegister function).
    */
    void wakeUp(){
        uint8_t register_value = readRegister(PWR_MGMT_1);
        uint8_t value_to_write = register_value & WAKE_UP_REGISTER_VALUE;
        writeRegister(PWR_MGMT_1, value_to_write);
    }

    /**
    * @brief Puts MPU6050 into sleep mode.
    * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails in writeRegister function).
    * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_byte_data) fails in writeRegister function).
    * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails in readRegister function).
    * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_byte_data) fails in readRegister function).
    */
    void sleep(){
        uint8_t register_value = readRegister(PWR_MGMT_1);
        uint8_t value_to_write = register_value | SLEEP_REGISTER_VALUE;
        writeRegister(PWR_MGMT_1, value_to_write);
    }

    /**
    * @brief Acquires working temperature of MPU6050 module in Celcius degrees.
    * @return Value of temperature in degrees.
    * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_i2c_block_data() fails in readWord function).
    * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails in readWord function.
    */
    float getTemperature(){
        int16_t temp_register = readWord(TEMP_OUT_H);
        float temp_deg = (temp_register / 340.0) + 36.53;
        return temp_deg;  
    }

    /**
    * @brief Reads one byte of data from specified register address.
    * @param register_addr Register address that is being read from.
    * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails).
    * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_byte_data) fails).
    * @return Value from register that has been read data in uint8_t format.
    */
    uint8_t readRegister(uint8_t register_addr){
        uint8_t value = i2c_.readRegister(dev_addr_, register_addr);
        return value;
    }

    /**
    * @brief Writes one byte of data into defined register.
    * @param register_addr Register address that is going to be overwritten.
    * @param value Value that is going to be written into choosen register.
    * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails in writeRegister function).
    * @exception std::runtime_error Thrown if system is unable to overwrite register with specified value (i2c_smbus_write_byte_data) fails in writeRegister function).
    */
    void writeRegister(uint8_t register_addr, uint8_t value){
        i2c_.writeRegister(dev_addr_, register_addr, value);
    }

    /**
    * @brief Reads word (2 bytes) data.
    * @param register_addr_h Start address of two byte data.
    * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_i2c_block_data() fails in readWord function).
    * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails in readWord function.
    */
    int16_t readWord(uint8_t register_addr_h){
        int16_t register_value = i2c_.readWord(dev_addr_, register_addr_h);
        return register_value;
    }

    /**
    * @brief Acquires acceleration on X axis.
    * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_i2c_block_data() fails in readWord function).
    * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails in readWord function.
    * @return Raw value of acceleration on X axis.
    */
    float getAccelerationX(){
        int16_t accel_register_x = readWord(ACCEL_XOUT_H);
        float accel_x = (static_cast<float>(accel_register_x) / accel_sensitivity_) - offset_accel_x_;
        return accel_x;
    }

    /**
    * @brief Acquires acceleration on Y axis.
    * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_i2c_block_data() fails in readWord function).
    * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails in readWord function.
    * @return Raw value of acceleration on Y axis.
    */
    float getAccelerationY(){
        int16_t accel_register_y = readWord(ACCEL_YOUT_H);
        float accel_y = (static_cast<float>(accel_register_y) / accel_sensitivity_) - offset_accel_y_;
        return accel_y;
    }

    /**
    * @brief Aquires acceleration on Y axis.
    * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_i2c_block_data() fails in readWord function).
    * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails in readWord function.
    * @return Raw value of acceleration on Y axis.
    */
    float getAccelerationZ(){
        int16_t accel_register_z = readWord(ACCEL_ZOUT_H);
        float accel_z = (static_cast<float>(accel_register_z) / accel_sensitivity_) - offset_accel_z_;
        return accel_z;
    }

    /**
    * @brief Aquire gyroscope value on X axis.
    * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_i2c_block_data() fails in readWord function).
    * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails in readWord function.
    * @return Raw value of gyroscope on X axis in defined metric (rad/deg).
    */
    float getGyroX(bool metrics){
        int16_t gyro_register_x = readWord(GYRO_XOUT_H);
        float gyro_x = (static_cast<float>(gyro_register_x) / gyro_sensitivity_) - offset_gyro_x_;
        if (metrics){
            gyro_x = (gyro_x / 360.0) * 2 * PI;
            return gyro_x;
        }
        return gyro_x;
    }

    /**
    * @brief Aquire gyroscope value on Y axis.
    * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_i2c_block_data() fails in readWord function).
    * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails in readWord function.
    * @return Raw value of gyroscope on Y axis in defined metric (rad/deg).
    */
    float getGyroY(bool metrics){
        int16_t gyro_register_y = readWord(GYRO_YOUT_H);
        float gyro_y = (static_cast<float>(gyro_register_y) / gyro_sensitivity_) - offset_gyro_y_;
        if (metrics){
            gyro_y = (gyro_y / 360.0) * 2 * PI;
            return gyro_y;
        }
        return gyro_y;
    }

    /**
    * @brief Aquire gyroscope value on Z axis.
    * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_i2c_block_data() fails in readWord function).
    * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails in readWord function.
    * @return Raw value of gyroscope on Z axis in defined metric (rad/deg).
    */
    float getGyroZ(bool metrics){
        int16_t gyro_register_z = readWord(GYRO_ZOUT_H);
        float gyro_z = static_cast<float>(gyro_register_z) / gyro_sensitivity_ - offset_gyro_z_;
        if (metrics){
            gyro_z = (gyro_z / 360.0) * 2 * PI;
            return gyro_z;
        }
        return gyro_z;
    }

    /**
    * @brief Returns all IMU operational data in one package.
    * Uses Burst Read to efficiently aquire all operational data.
    * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_i2c_block_data() fails in radBlockData function).
    * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails in radBlockData function).
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
            // Worth mention that is recomended to asign bitwise operations to uint and then cast it onto int
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
            data_struct.accel_x = (static_cast<float>(accel_register_x) / accel_sensitivity_) - offset_accel_x_;
            data_struct.accel_y = (static_cast<float>(accel_register_y) / accel_sensitivity_) - offset_accel_y_;
            data_struct.accel_z = (static_cast<float>(accel_register_z) / accel_sensitivity_) - offset_accel_z_;

            data_struct.temperature = (temp_register / 340.0) + 36.53;

            if(metrics){
                data_struct.gyro_x = (((static_cast<float>(gyro_register_x) / gyro_sensitivity_) - offset_gyro_x_) / 360.0) * 2 * PI;
                data_struct.gyro_y = (((static_cast<float>(gyro_register_y) / gyro_sensitivity_) - offset_gyro_y_) / 360.0) * 2 * PI;
                data_struct.gyro_z = (((static_cast<float>(gyro_register_z) / gyro_sensitivity_) - offset_gyro_z_) / 360.0) * 2 * PI;
            }else{
                data_struct.gyro_x = (static_cast<float>(gyro_register_x) / gyro_sensitivity_) - offset_gyro_x_;
                data_struct.gyro_y = (static_cast<float>(gyro_register_y) / gyro_sensitivity_) - offset_gyro_y_;
                data_struct.gyro_z = (static_cast<float>(gyro_register_z) / gyro_sensitivity_) - offset_gyro_z_;
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
        // Who am I test register 
        static constexpr uint8_t WHO_AM_I_REG = 0x75;

        // I2C interface template 
        I2C_Interface & i2c_;
        
        /**Adress in I2C system bus coresponding to device*/
        int dev_addr_;
        /**Calibration offset value */
        float offset_accel_x_;
        /**Calibration offset value */
        float offset_accel_y_;
        /**Calibration offset value */
        float offset_accel_z_;
        
        /**Calibration offset value */
        float offset_gyro_x_;
        /**Calibration offset value */
        float offset_gyro_y_;
        /**Calibration offset value */
        float offset_gyro_z_;
        

    };

    class LinuxI2C 
    {
    public: 
        /**
        * @brief LinuxI2C class constructor.
        * Obtains handle to system file responsible for I2C communication.
        * @param adapter_nr Identification number of physical I2C adapter/bus.
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
         * @param value Value that is going to be written into chosen register.
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
         * @param buffer Pointer to data buffer prepared for readed data.
         * @exception std::runtime_error Thrown if system is unable to read register value (i2c_smbus_read_i2c_block_data() fails).
         * @exception std::runtime_error Thrown if system is unable to connect to specified address (ioctl(I2C_SLAVE) fails).
         * @return The actual number of bytes read and placed into the buffer (success is indicated by a non-negative value).
         */
        int readDataBlock(uint8_t dev_addr, uint8_t start_register_addr, uint8_t size, uint8_t* buffer);

        private:
        /**Adapter number of I2C system file.*/
        int adapter_nr_;
        /**Handle to I2C bus system file.*/
        int file_identyficator_;

    };
}
#endif //MPU6050_DRIVER_HPP