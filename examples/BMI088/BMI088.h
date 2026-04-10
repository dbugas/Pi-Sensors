#pragma once

#include <pigpio.h>
#include <iostream>
#include <cstdint>
#include <vector>
#include <cstring>
#include <cmath>
#include <chrono>
#include <atomic>

#include "gpio.h"
#define PI 3.141592653589793
#define DEBUG_BMI088 // Uncomment for debug output

class BMI088 : public gpio {
    public:
        enum class AccelOversampling : uint8_t {
            OSR4   = 0x08,  // Strongest filtering (lowest bandwidth, least noise)
            OSR2   = 0x09,  // Medium filtering
            Normal = 0x0A   // Lightest filtering (highest bandwidth) – recommended for high ODR
        };

        enum class AccelODR : uint8_t {
            ODR_12_5Hz  = 0x05,
            ODR_25Hz    = 0x06,
            ODR_50Hz    = 0x07,
            ODR_100Hz   = 0x08,
            ODR_200Hz   = 0x09,
            ODR_400Hz   = 0x0A,
            ODR_800Hz   = 0x0B,
            ODR_1600Hz  = 0x0C
        };

        enum class AccelRange : uint8_t {
            G_3   = 0x00, 
            G_6   = 0x01, 
            G_12  = 0x02, 
            G_24  = 0x03  
        };

        enum class GyroRange : uint8_t {
            DPS_2000 = 0x00,
            DPS_1000 = 0x01,
            DPS_500  = 0x02,
            DPS_250  = 0x03,
            DPS_125  = 0x04
        };

        enum class GyroBandwidth : uint8_t{
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

            init_gpio();

            spi_gyro = spiOpenBus(CS_GYR, SPI_BAUD, SPI_FLAGS);
            if (spi_gyro < 0)
            {
                std::cerr << "[BMI088] SPI gyro open failed\n";
            }
            spi_acc = spiOpenBus(CS_ACC, SPI_BAUD, SPI_FLAGS);

            if (spi_acc < 0)
            {
                std::cerr << "[BMI088] SPI accel open failed\n";
            }
            // Verify gyro identity
            initGyro(BMI088::GyroRange::DPS_1000, BMI088::GyroBandwidth::ODR_1000Hz_BW_116Hz);
            uint8_t id = spiRead8(spi_gyro, BMI088_GYR_CHIP_ID);

            #ifdef DEBUG_BMI088
                std::cout << "[BMI088] Gyro CHIP_ID: 0x" << std::hex << int(id) << std::dec << "\n";
            #endif

            if (id != BMI088_GYR_ID)
            {
                std::cerr << "[BMI088] Gyro not detected\n";
            }

            // Verify accel identity/needed for startup with spi
            id = readAccelChipID();
            initAccelerometer(BMI088::AccelRange::G_6, BMI088::AccelODR::ODR_800Hz, BMI088::AccelOversampling::Normal);

            #ifdef DEBUG_BMI088
                std::cout << "[BMI088] Accel CHIP_ID: 0x"<< std::hex << int(id) << std::dec << "\n";
            #endif

            if (id != BMI088_ACC_ID)
            {
                std::cerr << "[BMI088] Accel not detected\n";
            }

            initCalibrationMatrix();
        }
        BMI088(AccelRange accel_range, AccelODR accel_odr, AccelOversampling accel_osr, 
            GyroRange gyro_range, GyroBandwidth gyro_bandwidth){
            init_gpio();

            spi_gyro = spiOpenBus(CS_GYR, SPI_BAUD, SPI_FLAGS);
            if (spi_gyro < 0)
            {
                std::cerr << "[BMI088] SPI gyro open failed\n";
            }
            spi_acc = spiOpenBus(CS_ACC, SPI_BAUD, SPI_FLAGS);

            if (spi_acc < 0)
            {
                std::cerr << "[BMI088] SPI accel open failed\n";
            }
            // Verify gyro identity
            initGyro(gyro_range, gyro_bandwidth);
            uint8_t id = spiRead8(spi_gyro, BMI088_GYR_CHIP_ID);

            #ifdef DEBUG_BMI088
                std::cout << "[BMI088] Gyro CHIP_ID: 0x" << std::hex << int(id) << std::dec << "\n";
            #endif

            if (id != BMI088_GYR_ID)
            {
                std::cerr << "[BMI088] Gyro not detected\n";
                spiCloseBus(spi_gyro);
                spiCloseBus(spi_acc);
            }

            // Verify accel identity/needed for startup with spi
            id = readAccelChipID();
            initAccelerometer(accel_range, accel_odr, accel_osr);

            #ifdef DEBUG_BMI088
                std::cout << "[BMI088] Accel CHIP_ID: 0x"<< std::hex << int(id) << std::dec << "\n";
            #endif

            if (id != BMI088_ACC_ID)
            {
                std::cerr << "[BMI088] Accel not detected\n";
                spiCloseBus(spi_gyro);
                spiCloseBus(spi_acc);
            }

            initCalibrationMatrix();
        }

    bool readGyro(double& gx_dps, double& gy_dps, double& gz_dps) {
        int16_t gx_raw, gy_raw, gz_raw;

        if (!ReadGyroRaw(gx_raw, gy_raw, gz_raw)) {
            return false;  // SPI transfer failed
        }

        // Convert raw signed 16-bit values to scaled rad per second
        gx_dps = static_cast<double>(gx_raw) * gyro_scale - Offset_gyro[0];
        gy_dps = static_cast<double>(gy_raw) * gyro_scale - Offset_gyro[1];
        gz_dps = static_cast<double>(gz_raw) * gyro_scale - Offset_gyro[2];

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

        if(spiTransfer(spi_gyro, reinterpret_cast<char*>(tx), reinterpret_cast<char*>(rx), 7) != 7){
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

        if (spiTransfer(spi_acc, reinterpret_cast<char*>(tx), reinterpret_cast<char*>(rx), 7) != 7)
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
    bool readAccelCalibrated(double& ax, double& ay, double& az){

        double ax_raw = 0.0, ay_raw = 0.0, az_raw = 0.0;
        if(!readAccel(ax_raw, ay_raw, az_raw)) return false;

        double dx = ax_raw - Offset[0];
        double dy = ay_raw - Offset[1];
        double dz = az_raw - Offset[2];

        ax = calibMatrix[0][0] * dx + calibMatrix[0][1] * dy + calibMatrix[0][2] * dz;
        ay = calibMatrix[1][0] * dx + calibMatrix[1][1] * dy + calibMatrix[1][2] * dz;
        az = calibMatrix[2][0] * dx + calibMatrix[2][1] * dy + calibMatrix[2][2] * dz;

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
        void wakeUp(AccelRange accel_range, AccelODR odr, AccelOversampling osr,
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
    void setAccelRange(AccelRange range) {
        uint8_t reg_value = static_cast<uint8_t>(range); 

        spiWrite8(spi_acc, BMI088_ACC_range, reg_value);  
        gpioDelay(10000);  // Allow setting to take effect

    #ifdef DEBUG_BMI088
        // Map back to human-readable g-value for debug print
        int range_g;
        switch (range) {
            case AccelRange::G_3:  range_g = 3;   break;
            case AccelRange::G_6:  range_g = 6;   break;
            case AccelRange::G_12: range_g = 12;  break;
            case AccelRange::G_24: range_g = 24;  break;
            default:               range_g = -1;  // Should never happen with enum class
        }

        std::cout << "[BMI088] Accelerometer range set to +/-" << range_g << "g "
                  << "(reg 0x41 = 0x" << std::hex << static_cast<int>(reg_value) << std::dec << ")\n";
    #endif

        acc_scale = getAccelScaleFactor_g(reg_value);
    }

    void setAccelODR_and_Filter(AccelODR odr, AccelOversampling osr) {
        uint8_t acc_odr = static_cast<uint8_t>(odr);          // e.g. ODR_100Hz → 0x08
        uint8_t acc_bwp = static_cast<uint8_t>(osr);          // e.g. Normal → 0x0A

        uint8_t reg_value = (acc_bwp << 4) | acc_odr;         // Bits [7:4] = bwp, [3:0] = odr

        spiWrite8(spi_acc, BMI088_ACC_CONF, reg_value);       // Register 0x40
        gpioDelay(10000);                                     // Settling time

        #ifdef DEBUG_BMI088
        // Map ODR enum back to Hz string for readable debug
        const char* odr_str;
        switch (odr) {
            case AccelODR::ODR_12_5Hz:  odr_str = "12.5"; break;
            case AccelODR::ODR_25Hz:    odr_str = "25";   break;
            case AccelODR::ODR_50Hz:    odr_str = "50";   break;
            case AccelODR::ODR_100Hz:   odr_str = "100";  break;
            case AccelODR::ODR_200Hz:   odr_str = "200";  break;
            case AccelODR::ODR_400Hz:   odr_str = "400";  break;
            case AccelODR::ODR_800Hz:   odr_str = "800";  break;
            case AccelODR::ODR_1600Hz:  odr_str = "1600"; break;
            default:                    odr_str = "??";   break;  // Should never hit
        }

        // Filter name from enum
        const char* filter_str;
        switch (osr) {
            case AccelOversampling::OSR4:   filter_str = "OSR4";   break;
            case AccelOversampling::OSR2:   filter_str = "OSR2";   break;
            case AccelOversampling::Normal: filter_str = "Normal"; break;
            default:                        filter_str = "??";     break;
        }

        std::cout << "[BMI088] ACC_CONF set: ODR = " << odr_str << " Hz, "
                  << "Filter = " << filter_str
                  << " (reg 0x40 = 0x" << std::hex << static_cast<int>(reg_value) << std::dec << ")\n";
        #endif
    }

    void setGyroRange(GyroRange range) {
        uint8_t val = static_cast<uint8_t>(range);
        spiWrite8(spi_gyro, BMI088_GYR_RANGE, val);
        gpioDelay(5000);
    }

    void setGyroBandwidth(GyroBandwidth bw) {
        uint8_t val = static_cast<uint8_t>(bw);
        spiWrite8(spi_gyro, BMI088_GYR_BANDWIDTH, val);
        gpioDelay(5000);
    }
    ~BMI088(){
        #ifdef DEBUG_BMI088
            std::cout << "Closing BMI088...\n";
        #endif

        lowPowerMode();

        spiCloseBus(spi_gyro);
        spiCloseBus(spi_acc);

        close_gpio();
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
            std::cout << "[BMI088] INT_CTRL = 0x%02X\n" << spiRead8(spi_gyro, 0x15) << "\n";
            std::cout << "[BMI088] INT_MAP  = 0x%02X\n" << spiRead8(spi_gyro, 0x18) << "\n";
            std::cout << "[BMI088] INT_CONF = 0x%02X\n" << spiRead8(spi_gyro, 0x16) << "\n";
        #endif
        // ───────────────────────────────────────────────────────────
    }
    void initAccelerometer(AccelRange accel_range, AccelODR accel_odr, AccelOversampling osr){

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

        spiTransfer(spi_acc, reinterpret_cast<char*>(tx), reinterpret_cast<char*>(rx), 3);
        spiTransfer(spi_acc, reinterpret_cast<char*>(tx), reinterpret_cast<char*>(rx), 3);
        
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

        spiTransfer(spi, tx, rx, 2);
        return static_cast<uint8_t>(rx[1]);
    }

    void initCalibrationMatrix()
    {
        // R matrix
        constexpr double R[3][3] = {
            {0.71255252 ,-0.13878161, 0.68775619},
            {-0.69090930, 0.03182244, 0.72224073},
            {0.12211981 , 0.98981160, 0.07321034}
        };

        // S diagonal scaling
        constexpr double S_diag[3][3] = {{1.10766793, 0.00000000, 0.00000000},
                                         {0.00000000, 1.00533049, 0.00000000},
                                         {0.00000000, 0.00000000, 0.87716575}};
        double temp[3][3] = {0};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                temp[i][j] = 0.0;
                for (int k = 0; k < 3; ++k) {
                    temp[i][j] += S_diag[i][k] * R[j][k];  
                }
            }
        }

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                calibMatrix[i][j] = 0.0;
                for (int k = 0; k < 3; ++k) {
                    calibMatrix[i][j] += R[i][k]*temp[k][j];  
                }
            }
        }
        #ifdef DEBUG_BMI088
        std::cout << "[BMI088] Calibration matrix (R*S*R^T):\n";
        for (int i = 0; i < 3; ++i) {
            std::cout << "  ";
            for (int j = 0; j < 3; ++j) {
                std::cout << calibMatrix[i][j] << (j<2 ? ", " : "");
            }
            std::cout << "\n";
        }
        #endif
    }


    double acc_scale; // m/s
    double gyro_scale; // rad/s
    
    int spi_gyro = -1;
    int spi_acc = -1;


    double calibMatrix[3][3] = {0};
    static constexpr double Offset[3] = {
        0.00474523,    
        0.01032585,
        -0.05219814
    };
    static constexpr double Offset_gyro[3] = {
        0.00262801,    
        -0.00188499,
        -0.00057151
    };
    // --------------- Configuration ---------------
    static constexpr unsigned SPI_BAUD  = 500000;   
    static constexpr unsigned SPI_FLAGS = 0x00;     // 0x100 AUX SPI (SPI1), mode 0
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