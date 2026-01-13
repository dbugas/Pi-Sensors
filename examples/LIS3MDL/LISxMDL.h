#pragma once

#include <pigpio.h>
#include <iostream>
#include <cstdint>

#define DEBUG_LIS3MDL  // Uncomment for debug output

class LISxMDL {
public:
    // ---------------- ODR (CTRL_REG1 bits 7 + [4:2]) ----------------
    enum class ODR : uint8_t {
        Hz_0_625 = 0x00,
        Hz_1_25  = 0x04,
        Hz_2_5   = 0x08,
        Hz_5     = 0x0C,
        Hz_10    = 0x10,
        Hz_20    = 0x14,
        Hz_40    = 0x18,
        Hz_80    = 0x1C,

        // FAST_ODR enabled (bit 7)
        Hz_155   = 0x80 | 0x00,
        Hz_300   = 0x80 | 0x04,
        Hz_560   = 0x80 | 0x08,
        Hz_1000  = 0x80 | 0x0C
    };

    // ---------------- Performance Modes ----------------
    // CTRL_REG1 OM[6:5] (XY), CTRL_REG4 OMZ[3:2] (Z)
    enum class PerformanceMode : uint8_t {
        LowPower     = 0x00,
        Medium       = 0x20,
        High         = 0x40,
        UltraHigh    = 0x60
    };

    enum class ZPerformanceMode : uint8_t {
        LowPower     = 0x00,
        Medium       = 0x04,
        High         = 0x08,
        UltraHigh    = 0x0C
    };

    // ---------------- Full-scale (CTRL_REG2 bits [6:5]) ----------------
    enum class FullScale : uint8_t {
        Gauss_4  = 0x00,
        Gauss_8  = 0x20,
        Gauss_12 = 0x40,
        Gauss_16 = 0x60
    };

    // ---------------- Constructor ----------------
    LISxMDL(FullScale range, ODR odr) {

        if (gpioInitialise() < 0) {
            std::cerr << "pigpio initialisation failed." << std::endl;
        }

        handle_ = i2cOpen(1, LIS3MDL_I2C_ADDR, 0);
        if (handle_ < 0) {
            std::cerr << "[LIS3MDL] Failed to open I2C device\n";
        }

        uint8_t id = i2cReadByteData(handle_, WHO_AM_I_REG);
        if (id != EXPECTED_WHO_AM_I) {
            std::cerr << "[LIS3MDL] Wrong WHO_AM_I: 0x"
                      << std::hex << int(id) << std::dec << "\n";
            i2cClose(handle_);
            handle_ = -1;
        }

        init(range, odr);
    }

    // ---------------- Destructor ----------------
    ~LISxMDL() {
        if (handle_ >= 0) {
            i2cWriteByteData(handle_, CTRL_REG3, 0x03); // power-down
            i2cClose(handle_);
        }
    }

    // ---------------- Raw Read ----------------
    bool readRaw(int16_t& mx, int16_t& my, int16_t& mz) {
        if (handle_ < 0) return false;

        char buf[6];
        i2cWriteByte(handle_, OUT_X_L);
        if (i2cReadDevice(handle_, buf, 6) != 6) return false;

        mx = int16_t(buf[1] << 8 | buf[0]);
        my = int16_t(buf[3] << 8 | buf[2]);
        mz = int16_t(buf[5] << 8 | buf[4]);

        return true;
    }

    // ---------------- Read in Gauss ----------------
    bool read_gauss(double& mx, double& my, double& mz) {
        int16_t rx, ry, rz;
        if (!readRaw(rx, ry, rz)) return false;

        mx = rx / scale;
        my = ry / scale;
        mz = rz / scale;

        return true;
    }

    bool dataReady() const {
        int status = i2cReadByteData(handle_, STATUS_REG);
        if (status < 0) {
            return false;
        }
        return status & 0x08; // ZYXDA
    }
private:
    // ---------------- Initialization ----------------
    void init(FullScale range, ODR odr) {
        uint8_t odr_val = static_cast<uint8_t>(odr);
        bool fast = odr_val & 0x80;
        uint8_t odr_bits = odr_val & 0x1C;

        PerformanceMode xy_mode = fast ? PerformanceMode::High
                                       : PerformanceMode::UltraHigh;

        ZPerformanceMode z_mode = fast ? ZPerformanceMode::High
                                       : ZPerformanceMode::UltraHigh;

        uint8_t ctrl1 = static_cast<uint8_t>(xy_mode) | odr_bits | (fast ? 0x80 : 0x00);
        i2cWriteByteData(handle_, CTRL_REG1, ctrl1);

        i2cWriteByteData(handle_, CTRL_REG2, static_cast<uint8_t>(range));
        i2cWriteByteData(handle_, CTRL_REG3, 0x00); // continuous

        uint8_t ctrl4 = static_cast<uint8_t>(z_mode);
        i2cWriteByteData(handle_, CTRL_REG4, ctrl4);

        uint8_t ctrl5 = 0x40;
        i2cWriteByteData(handle_, CTRL_REG5, ctrl5); // block data update

        scale = getLSBperGauss();

    #ifdef DEBUG_LIS3MDL
        std::cout << "[LIS3MDL] CTRL_REG1 value: " << std::hex << int(ctrl1) << std::dec << "\n";
        std::cout << "[LIS3MDL] CTRL_REG2 value: " << std::hex << int(static_cast<uint8_t>(range)) << std::dec << "\n";
        std::cout << "[LIS3MDL] CTRL_REG3 value: " << std::hex << int(0x00) << std::dec << "\n";
        std::cout << "[LIS3MDL] CTRL_REG4 value: " << std::hex << int(ctrl4) << std::dec << "\n";
        std::cout << "[LIS3MDL] CTRL_REG5 value: " << std::hex << int(ctrl5) << std::dec << "\n";

        std::cout << "[LIS3MDL] scale = " << scale << "\n";
        std::cout << "[LIS3MDL] Initialized\n";
    #endif
    }

    // ---------------- Scaling ----------------
    double getLSBperGauss() const {
        uint8_t fs = i2cReadByteData(handle_, CTRL_REG2) & 0x60;
        switch (fs) {
            case 0x00: return 6842.0;
            case 0x20: return 3421.0;
            case 0x40: return 2281.0;
            case 0x60: return 1711.0;
            default:   return 1711.0;
        }
    }

    double scale;
    // ---------------- Registers ----------------
    static constexpr uint8_t LIS3MDL_I2C_ADDR  = 0x1C;
    static constexpr uint8_t WHO_AM_I_REG      = 0x0F;
    static constexpr uint8_t EXPECTED_WHO_AM_I = 0x3D;

    static constexpr uint8_t CTRL_REG1  = 0x20;
    static constexpr uint8_t CTRL_REG2  = 0x21;
    static constexpr uint8_t CTRL_REG3  = 0x22;
    static constexpr uint8_t CTRL_REG4  = 0x23;
    static constexpr uint8_t CTRL_REG5  = 0x24;
    static constexpr uint8_t STATUS_REG = 0x27;

    static constexpr uint8_t OUT_X_L   = 0x28 | 0x80;

    int handle_ = -1;
};
