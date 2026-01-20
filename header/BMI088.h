#pragma once

#include <pigpio.h>
#include <iostream>
#include <cstdint>
#include <vector>
#include <cstring>
#include <cmath>
#include <chrono>
#include <atomic>

#define PI 3.141592653589793
//#define DEBUG_BMI088 // Uncomment for debug output

class BMI088 {
    public:
        enum class AccelOversampling {
            OSR4   = 0x08,  // Strongest filtering (lowest bandwidth, least noise)
            OSR2   = 0x09,  // Medium filtering
            Normal = 0x0A   // Lightest filtering (highest bandwidth) – recommended for high ODR
        };

            enum class GyroRange {
            DPS_2000 = 0x00,
            DPS_1000 = 0x01,
            DPS_500  = 0x02,
            DPS_250  = 0x03,
            DPS_125  = 0x04
        };

        enum class GyroBandwidth {
            ODR_2000Hz_BW_532Hz = 0x00,  
            ODR_2000Hz_BW_230Hz = 0x01,
            ODR_1000Hz_BW_116Hz = 0x02,
            ODR_400Hz_BW_47Hz   = 0x03,
            ODR_200Hz_BW_23Hz   = 0x04,
            ODR_100Hz_BW_12Hz   = 0x05,
            ODR_200Hz_BW_64Hz   = 0x06,
            ODR_100Hz_BW_32Hz   = 0x07
        };

        BMI088(){
            spi_gyro = spiOpen(CS_GYR, SPI_BAUD, SPI_FLAGS);
            if (spi_gyro < 0)
            {
                std::cerr << "SPI gyro open failed\n";
                gpioTerminate();
            }
            spi_acc = spiOpen(CS_ACC, SPI_BAUD, SPI_FLAGS);

            if (spi_acc < 0)
            {
                std::cerr << "SPI accel open failed\n";
                gpioTerminate();
            }
            // Verify gyro identity
            initGyro(BMI088::GyroRange::DPS_1000, BMI088::GyroBandwidth::ODR_1000Hz_BW_116Hz);
            uint8_t id = spiRead8(spi_gyro, BMI088_GYR_CHIP_ID);

            #ifdef DEBUG_BMI088
                std::cout << "Gyro CHIP_ID: 0x" << std::hex << int(id) << std::dec << "\n";
            #endif

            if (id != BMI088_GYR_ID)
            {
                std::cerr << "Gyro not detected\n";
                spiClose(spi_gyro);
                spiClose(spi_acc);
                gpioTerminate();
            }

            // Verify accel identity/needed for startup with spi
            initAccelerometer(6, 800.0, BMI088::AccelOversampling::Normal);
            id = readAccelChipID();

            #ifdef DEBUG_BMI088
                std::cout << "Accel CHIP_ID: 0x"<< std::hex << int(id) << std::dec << "\n";
            #endif

            if (id != BMI088_ACC_ID)
            {
                std::cerr << "Accel not detected\n";
                spiClose(spi_gyro);
                spiClose(spi_acc);
                gpioTerminate();
            }
        }
        BMI088(uint8_t accel_range, double accel_odr, AccelOversampling accel_osr, 
            GyroRange gyro_range, GyroBandwidth gyro_bandwidth){
            spi_gyro = spiOpen(CS_GYR, SPI_BAUD, SPI_FLAGS);
            if (spi_gyro < 0)
            {
                std::cerr << "SPI gyro open failed\n";
                gpioTerminate();
            }
            spi_acc = spiOpen(CS_ACC, SPI_BAUD, SPI_FLAGS);

            if (spi_acc < 0)
            {
                std::cerr << "SPI accel open failed\n";
                gpioTerminate();
            }
            // Verify gyro identity
            initGyro(gyro_range, gyro_bandwidth);
            uint8_t id = spiRead8(spi_gyro, BMI088_GYR_CHIP_ID);

            #ifdef DEBUG_BMI088
                std::cout << "Gyro CHIP_ID: 0x" << std::hex << int(id) << std::dec << "\n";
            #endif

            if (id != BMI088_GYR_ID)
            {
                std::cerr << "Gyro not detected\n";
                spiClose(spi_gyro);
                spiClose(spi_acc);
                gpioTerminate();
            }

            // Verify accel identity/needed for startup with spi
            initAccelerometer(accel_range, accel_odr, accel_osr);
            id = readAccelChipID();

            #ifdef DEBUG_BMI088
                std::cout << "Accel CHIP_ID: 0x"<< std::hex << int(id) << std::dec << "\n";
            #endif

            if (id != BMI088_ACC_ID)
            {
                std::cerr << "Accel not detected\n";
                spiClose(spi_gyro);
                spiClose(spi_acc);
                gpioTerminate();
            }

            
        }

    bool readGyro(double& gx_dps, double& gy_dps, double& gz_dps) {
        int16_t gx_raw, gy_raw, gz_raw;

        if (!ReadGyroRaw(gx_raw, gy_raw, gz_raw)) {
            return false;  // SPI transfer failed
        }

        // Convert raw signed 16-bit values to scaled rad per second
        gx_dps = static_cast<double>(gx_raw) * gyro_scale;
        gy_dps = static_cast<double>(gy_raw) * gyro_scale;
        gz_dps = static_cast<double>(gz_raw) * gyro_scale;

        return true;
    }
    bool readAccel(double& ax_g, double& ay_g, double& az_g) {
        int16_t ax_raw, ay_raw, az_raw;

        if (!readAccelRaw(ax_raw, ay_raw, az_raw)) {
            return false;  // SPI transfer failed
        }

        // Convert raw signed 16-bit values to scaled g
        ax_g = static_cast<double>(ax_raw) * acc_scale;
        ay_g = static_cast<double>(ay_raw) * acc_scale;
        az_g = static_cast<double>(az_raw) * acc_scale;

        return true;
    }
    bool ReadGyroRaw(int16_t& gx, int16_t& gy, int16_t& gz){
        char tx[7] = {0};
        char rx[7] = {0};

        tx[0] = BMI088_GYR_DATA_X_L | 0x80; // read, auto-increment

        if(spiXfer(spi_gyro, reinterpret_cast<char*>(tx), reinterpret_cast<char*>(rx), 7) != 7){
            return false;
        }

        gx = (int16_t)((rx[2] << 8) | rx[1]);
        gy = (int16_t)((rx[4] << 8) | rx[3]);
        gz = (int16_t)((rx[6] << 8) | rx[5]);

        return true;
    }
    bool readAccelRaw(int16_t& ax, int16_t& ay, int16_t& az){
        uint8_t tx[7] = {0};
        uint8_t rx[7] = {0};

        // Start burst read at ACC_X_LSB (0x12), set bit 7 for read command
        tx[0] = 0x12 | 0x80;

        // The rest of tx can stay 0 (dummy bytes sent by master)

        if (spiXfer(spi_acc, reinterpret_cast<char*>(tx), reinterpret_cast<char*>(rx), 7) != 7)
        {
            return false;  // Transfer failed
        }

        // rx[0] = dummy byte → discard
        // rx[1] = ACC_X_LSB
        // rx[2] = ACC_X_MSB
        // rx[3] = ACC_Y_LSB
        // rx[4] = ACC_Y_MSB
        // rx[5] = ACC_Z_LSB
        // rx[6] = ACC_Z_MSB

        az = static_cast<int16_t>((rx[1] << 8) | rx[0]);  // MSB * 256 + LSB
        ay = static_cast<int16_t>((rx[3] << 8) | rx[2]);
        ax = static_cast<int16_t>((rx[5] << 8) | rx[4]);

        return true;
    }
    void lowPowerMode() {
        #ifdef DEBUG_BMI088
            std::cout << "Putting BMI088 into low-power mode...\n";
        #endif
        // 1. Accelerometer: suspend mode
        spiWrite8(spi_acc, BMI088_ACC_PWR_CTRL, 0x00);
        gpioDelay(5000);  // Give time for mode change

        // 2. Gyroscope: Deep suspend mode
        spiWrite8(spi_gyro, BMI088_GYRO_LPM1, 0x20); 
        gpioDelay(5000);

        #ifdef DEBUG_BMI088
            std::cout << "BMI088 low-power mode activated.\n";
        #endif
    }
        void wakeUp(u_int8_t accel_range, double odr, AccelOversampling osr,
                GyroRange range, GyroBandwidth bandwidth) {
            #ifdef DEBUG_BMI088
                std::cout << "Waking up BMI088...\n";
            #endif
            
            spiWrite8(spi_gyro, BMI088_GYRO_LPM1, 0x00); 
            gpioDelay(5000);

            // Re-initialize gyro
            initGyro(range, bandwidth);
            
            // Re-initialize accelerometer
            initAccelerometer(accel_range, odr, osr);
            #ifdef DEBUG_BMI088
                std::cout << "BMI088 awakened.\n";
            #endif
    }
    void setAccelRange(uint8_t range_g) {
        uint8_t reg_value;

            switch (range_g) {
                case 3:  reg_value = 0x00; break;
                case 6:  reg_value = 0x01; break;
                case 12: reg_value = 0x02; break;
                case 24: reg_value = 0x03; break;
                default:
                #ifdef DEBUG_BMI088
                            std::cout << "[BMI088] Invalid accel range requested: " << int(range_g) << "g. Valid: 3,6,12,24\n";
                #endif
                    reg_value = 0x04;
            }
        
            spiWrite8(spi_acc, BMI088_ACC_range, reg_value);  // ACC_RANGE register
            gpioDelay(10000);  // Allow setting to take effect
        
        #ifdef DEBUG_BMI088
            std::cout << "[BMI088] Accelerometer range set to +/-" << int(range_g) << "g (reg 0x41 = 0x"
                        << std::hex << int(reg_value) << std::dec << ")\n";
        #endif
        acc_scale = getAccelScaleFactor_g(reg_value);
    }

    void setAccelODR_and_Filter(double odr_hz, AccelOversampling osr) {
        // Map ODR in Hz to acc_odr bits [3:0]
        uint8_t acc_odr;
        if      (odr_hz == 12.5) acc_odr = 0x05;
        else if (odr_hz == 25)   acc_odr = 0x06;
        else if (odr_hz == 50)   acc_odr = 0x07;
        else if (odr_hz == 100)  acc_odr = 0x08;
        else if (odr_hz == 200)  acc_odr = 0x09;
        else if (odr_hz == 400)  acc_odr = 0x0A;
        else if (odr_hz == 800)  acc_odr = 0x0B;
        else if (odr_hz == 1600) acc_odr = 0x0C;
        else {
        #ifdef DEBUG_BMI088
            std::cout << "[BMI088] Invalid ODR requested: " << odr_hz << " Hz\n";
        #endif
        }

        // acc_bwp from enum (bits [7:4])
        uint8_t acc_bwp = static_cast<uint8_t>(osr);
        uint8_t reg_value = (acc_bwp << 4) | acc_odr;

        spiWrite8(spi_acc, BMI088_ACC_CONF, reg_value);
        gpioDelay(10000);  

        #ifdef DEBUG_BMI088
            std::cout << "[BMI088] ACC_CONF set: ODR = " << odr_hz << " Hz, "
                   << "Filter = " << (osr == AccelOversampling::OSR4 ? "OSR4" :
                                       osr == AccelOversampling::OSR2 ? "OSR2" : "Normal")
                   << " (reg 0x40 = 0x" << std::hex << int(reg_value) << std::dec << ")\n";
        #endif
    }

    void setGyroRange(GyroRange range) {
        spiWrite8(spi_gyro, BMI088_GYR_RANGE, static_cast<uint8_t>(range));
        gpioDelay(5000);
    }

    void setGyroBandwidth(GyroBandwidth bw) {
        spiWrite8(spi_gyro, BMI088_GYR_BANDWIDTH, static_cast<uint8_t>(bw));
        gpioDelay(5000);
    }
    ~BMI088(){
        #ifdef DEBUG_BMI088
            std::cout << "Closing BMI088...\n";
        #endif

        lowPowerMode();

        spiClose(spi_gyro);
        spiClose(spi_acc);
    }
    private:

    void initGyro(GyroRange range, GyroBandwidth bw) {
        spiWrite8(spi_gyro, BMI088_GYRO_LPM1, 0x00); 
        gpioDelay(5000);

        setGyroRange(range);
        setGyroBandwidth(bw);

        gyro_scale = getGyroScaleFactor();
        gpioDelay(2000);

        // ── interrupt setup ──────────────────────────────
        //spiWrite8(spi_gyro, 0x15, 0x80);      // register 0x15: GYRO_INT_CTRL
        //gpioDelay(2000);

        //spiWrite8(spi_gyro, 0x16, 0x00);      // register 0x16: INT3_INT4_IO_CONF       
        //spiWrite8(spi_gyro, 0x18, 0x01);      // register 0x18: INT3_INT4_IO_MAP      
        //gpioDelay(5000);

        #ifdef DEBUG_BMI088
            std::cout << "INT_CTRL = 0x%02X\n" << spiRead8(spi_gyro, 0x15) << "\n";
            std::cout << "INT_MAP  = 0x%02X\n" << spiRead8(spi_gyro, 0x18) << "\n";
            std::cout << "INT_CONF = 0x%02X\n" << spiRead8(spi_gyro, 0x16) << "\n";
        #endif
        // ───────────────────────────────────────────────────────────
    }
    void initAccelerometer(uint8_t accel_range, double accel_odr, AccelOversampling osr){

        spiWrite8(spi_acc, BMI088_ACC_PWR_CTRL, 0x04); // ACC_PWR_CTRL: on
        gpioDelay(5000);

        spiWrite8(spi_acc, BMI088_ACC_PWR_CONF, 0x00); // ACC_PWR_CONF: active
        gpioDelay(5000);

        setAccelRange(accel_range); 
        setAccelODR_and_Filter(accel_odr, osr);
    }

double getAccelScaleFactor_g(uint8_t reg_value) {

    // Apply the datasheet formula using only the valid range code
    double scale = 1.5 * (1 << (reg_value + 1)) / 32768.0;

    #ifdef DEBUG_BMI088
        std::cout << "[BMI088] Accel scale factor = " << scale << " g/LSB\n";
    #endif

    return scale;
}

    double getGyroScaleFactor(){
        uint8_t reg = spiRead8(spi_gyro, BMI088_GYR_RANGE);
        double scale;

        switch (reg & 0x07) { 
        case 0x00:
            scale = 2000.0 / 32768.0;   // ±2000 °/s = 16.384 LSB/°/s
            break;
        case 0x01:
            scale = 1000.0 / 32768.0;   // ±1000 °/s = 32.768 LSB/°/s
            break;
        case 0x02:
            scale = 500.0 / 32768.0;    // ±500 °/s  = 65.536 LSB/°/s
            break;
        case 0x03:
            scale = 250.0 / 32768.0;    // ±250 °/s  = 131.072 LSB/°/s
            break;
        case 0x04:
            scale = 125.0 / 32768.0;    // ±125 °/s  = 262.144 LSB/°/s
            break;
        default:
        #ifdef DEBUG_BMI088
                    std::cout << "[BMI088] Invalid GYR_RANGE value: 0x" << std::hex << int(reg) << std::dec << "\n";
        #endif
            scale = 2000.0 / 32768.0;  
            break;
    }

    #ifdef DEBUG_BMI088
        std::cout << "[BMI088] Gyro scale factor = " << scale << " deg/s per LSB (range reg = 0x"
                    << std::hex << int(reg) << std::dec << ")\n";
    #endif

    return scale * PI/180.0; // convert to rad/s
    }

    uint8_t readAccelChipID(){
        uint8_t tx[3] = {
            static_cast<uint8_t>(BMI088_ACC_CHIP_ID | 0x80), // CHIP_ID + read
            0x00,
            0x00
        };
        uint8_t rx[3] = {0};

        spiXfer(spi_acc, reinterpret_cast<char*>(tx), reinterpret_cast<char*>(rx), 3);

        return rx[2]; // rx[1] is dummy, rx[2] is CHIP_ID
    }

    void spiWrite8(const int spi, const uint8_t reg, const uint8_t val){
        char tx[2] = {static_cast<char>(reg & 0x7F), static_cast<char>(val)};
        spiWrite(spi, tx, 2);
    }

    uint8_t spiRead8(const int spi, uint8_t reg){
        char tx[2] = {static_cast<char>(reg | 0x80), // read
            0x00
        };
        char rx[2] = {0};

        spiXfer(spi, tx, rx, 2);
        return static_cast<uint8_t>(rx[1]);
    }




    double acc_scale; // m/s
    double gyro_scale; // rad/s
    
    int spi_gyro = -1;
    int spi_acc = -1;


    // --------------- Configuration ---------------
    static constexpr unsigned SPI_BAUD  = 5000000;   
    static constexpr unsigned SPI_FLAGS = 0x100;     // AUX SPI (SPI1), mode 0
    static constexpr unsigned CS_GYR    = 1;         // /dev/spidev1.1
    static constexpr unsigned CS_ACC    = 0;
    // --------------- BMI088 Gyro Registers ---------------
    static constexpr uint8_t BMI088_GYR_CHIP_ID   = 0x00;
    static constexpr uint8_t BMI088_GYR_RANGE     = 0x0F;
    static constexpr uint8_t BMI088_GYR_BANDWIDTH = 0x10;
    static constexpr uint8_t BMI088_GYR_DATA_X_L  = 0x02;
    static constexpr uint8_t BMI088_GYRO_LPM1     = 0x11;
    static constexpr uint8_t FIFO_CONFIG_1        = 0x3E;
    static constexpr uint8_t GYRO_INT_STAT_1      = 0x0A;
    // --------------- BMI088 Accel Registers ---------------
    static constexpr uint8_t BMI088_ACC_CHIP_ID  = 0x00;
    static constexpr uint8_t BMI088_ACC_CONF     = 0x40; // set oversampling and ODR
    static constexpr uint8_t BMI088_ACC_range    = 0x41; 
    static constexpr uint8_t BMI088_ACC_PWR_CONF = 0x7C; 
    static constexpr uint8_t BMI088_ACC_PWR_CTRL = 0x7D;
    static constexpr uint8_t ACC_STATUS          = 0x03;
    // Expected IDs
    static constexpr uint8_t BMI088_ACC_ID = 0x1E;
    static constexpr uint8_t BMI088_GYR_ID = 0x0F;

};