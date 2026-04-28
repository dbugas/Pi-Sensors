#pragma once

#include <pigpio.h>
#include <iostream>
#include <cstdint>
#include <Eigen/Dense>
#include <stdexcept>

#include "gpio.h"
//#define DEBUG_LIS2MDL  // Uncomment for debug output

class LIS2MDL : public gpio  {
public:
    enum class ODR : uint8_t {
        ODR_10HZ  = 0b00,  // 10 Hz (default)
        ODR_20HZ  = 0b01,  // 20 Hz
        ODR_50HZ  = 0b10,  // 50 Hz
        ODR_100HZ = 0b11   // 100 Hz
    };

    // ---------------- Constructor ----------------
    LIS2MDL(ODR odr) {

        init_gpio();
        handle_ = spiOpenBus(CS, SPI_BAUD, SPI_FLAGS, CS);
        if (handle_ < 0) {
            std::cerr << "[LIS2MDL] Failed to open device\n";
        }

        writeRegister(CFG_REG_C, 0x34);   // 4WSPI + I2C_DIS + BDU           

        uint8_t id = readRegister(WHO_AM_I_REG);
        if (id != EXPECTED_WHO_AM_I) {
            std::cerr << "[LIS2MDL] Wrong WHO_AM_I: 0x" << std::hex << int(id) << std::dec << " (expected 0x40)\n";
            throw std::runtime_error("LIS2MDL not found!");
        }
        #ifdef DEBUG_LIS2MDL
            std::cout << "[LIS2MDL] WHO_AM_I: 0x"  << std::hex << int(id) << std::dec << " (expected 0x40)\n";
        #endif

        uint8_t reg = 0;
        uint8_t tempComp = 1;
        reg |= tempComp << 7;
        reg |= static_cast<uint8_t>(odr) << 2;
        reg |= 0b00000000; // continuous mode, high resolution mode
        writeRegister(CFG_REG_A, reg);
        reg = 0b00000010;   // OFF_CANC=1, LPF=0, last 2 bits
        writeRegister(CFG_REG_B, reg);
        writeRegister(INT_CTRL_REG, 0b00000000); // disable interrupt

        gpioDelay(5000);
    }
    LIS2MDL(ODR odr, Eigen::Matrix3d Cali_mat) : calibMatrix(Cali_mat) {

        init_gpio();
        handle_ = spiOpenBus(CS, SPI_BAUD, SPI_FLAGS, CS);
        if (handle_ < 0) {
            std::cerr << "[LIS2MDL] Failed to open device\n";
        }

        writeRegister(CFG_REG_C, 0x34);   // 4WSPI + I2C_DIS + BDU           

        uint8_t id = readRegister(WHO_AM_I_REG);
        if (id != EXPECTED_WHO_AM_I) {
            std::cerr << "[LIS2MDL] Wrong WHO_AM_I: 0x" << std::hex << int(id) << std::dec << " (expected 0x40)\n";
            throw std::runtime_error("LIS2MDL not found!");
        }
        #ifdef DEBUG_LIS2MDL
            std::cout << "[LIS2MDL] WHO_AM_I: 0x"  << std::hex << int(id) << std::dec << " (expected 0x40)\n";
        #endif

        uint8_t reg = 0;
        uint8_t tempComp = 1;
        reg |= tempComp << 7;
        reg |= static_cast<uint8_t>(odr) << 2;
        reg |= 0b00000000; // continuous mode, high resolution mode
        writeRegister(CFG_REG_A, reg);
        reg = 0b00000010;   // OFF_CANC=1, LPF=0, last 2 bits
        writeRegister(CFG_REG_B, reg);
        writeRegister(INT_CTRL_REG, 0b00000000); // disable interrupt

        gpioDelay(5000);
    }
    // ---------------- Destructor ----------------
    ~LIS2MDL() {
        if (handle_ >= 0) {
            writeRegister(CFG_REG_A, 0b10000011); // power-down
            spiCloseBus(handle_);
        }
    }

    // ---------------- Raw Read ----------------
    bool readRaw(int16_t& mx, int16_t& my, int16_t& mz) {
        if (handle_ < 0) return false;

        uint8_t tx[7] = {OUT_X_L | 0xC0, 0, 0, 0, 0, 0, 0};  // 0xC0 = read + auto-increment
        uint8_t rx[7] = {0};

        if (spiTransfer(handle_, reinterpret_cast<char*>(tx), reinterpret_cast<char*>(rx), 7u, CS) != 7) {
            return false;
        }

        mx = int16_t(rx[2] << 8 | rx[1]);
        my = int16_t(rx[4] << 8 | rx[3]);
        mz = int16_t(rx[6] << 8 | rx[5]);

        return true;
    }

    // ---------------- Read in Gauss ----------------
    bool read_gauss(double& mx, double& my, double& mz) {
        int16_t rx, ry, rz;
        if (!readRaw(rx, ry, rz)) return false;

        mx = rx * scale;
        my = ry * scale;
        mz = rz * scale;

        return true;
    }

    bool readCalibrated(double& mx, double& my, double& mz) {
        double rx, ry, rz;
        if (!read_gauss(rx, ry, rz)) return false;

        Eigen::Vector3d m_raw(rx, ry, rz);
        m_raw = calibMatrix*m_raw;

        mx = m_raw(0);
        my = m_raw(1);
        mz = m_raw(2);

        return true;
    }

    bool dataReady() {
        int status = readRegister(STATUS_REG);
        return (status & 0x08) != 0;
    }

    // Hard iron offsets are applied in hardware instead of software
    void setHardIronOffsets(float offset_x, float offset_y, float offset_z) {

        // Convert to signed 16-bit integer (two's complement)
        int16_t raw_x = static_cast<int16_t>(round(offset_x / float(scale)));
        int16_t raw_y = static_cast<int16_t>(round(offset_y / float(scale)));
        int16_t raw_z = static_cast<int16_t>(round(offset_z / float(scale)));

        uint8_t tx[7];
        // Little Endian
        tx[0] = OFFSET_X_REG_L;                             // Starting register: OFFSET_X_REG_L (write bit is 0)
        tx[1] = static_cast<uint8_t>(raw_x & 0xFF);         // X low
        tx[2] = static_cast<uint8_t>((raw_x >> 8) & 0xFF);  // X high
        tx[3] = static_cast<uint8_t>(raw_y & 0xFF);         // Y low
        tx[4] = static_cast<uint8_t>((raw_y >> 8) & 0xFF);  // Y high
        tx[5] = static_cast<uint8_t>(raw_z & 0xFF);         // Z low
        tx[6] = static_cast<uint8_t>((raw_z >> 8) & 0xFF);  // Z high

        spiTransfer(handle_, reinterpret_cast<char*>(tx), nullptr, 7u, CS);
        #ifdef DEBUG_LIS2MDL
            std::printf("[LIS2MDL] Hard-iron offsets set: X=%.3f g, Y=%.3f g, Z=%.3f g\n", 
                        offset_x, offset_y, offset_z);
            std::printf("[LIS2MDL] Failed to write hard-iron offsets\n");
        #endif

    }
private:

    uint8_t readRegister(uint8_t reg) {
        if (handle_ < 0) return 0;

        uint8_t tx[2] = {static_cast<uint8_t>(reg | 0x80), 0x00};
        uint8_t rx[2] = {0};

        spiTransfer(handle_, reinterpret_cast<char*>(tx), reinterpret_cast<char*>(rx), 2u, CS);

        #ifdef DEBUG_LIS2MDL
            std::printf("[LIS2MDL] Read reg 0x%02X → rx[0]=0x%02X, rx[1]=0x%02X (result=%d)\n", 
                        reg, rx[0], rx[1], result);
        #endif

        return rx[1];
    }

    void writeRegister(uint8_t reg, uint8_t value) {
        uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7F), value};
        
        spiTransfer(handle_, reinterpret_cast<char*>(tx), nullptr, 2u, CS);
        gpioDelay(1000);
    }

    Eigen::Matrix3d calibMatrix = Eigen::Matrix3d::Zero();
    double scale = 0.0015; // gauss

    static constexpr unsigned SPI_BAUD  = 6000000;   
    static constexpr unsigned SPI_FLAGS = 0;
    static constexpr unsigned CS        = 23;

    // ---------------- Registers ----------------
    static constexpr uint8_t LIS3MDL_I2C_ADDR  = 0x1C;
    static constexpr uint8_t WHO_AM_I_REG      = 0x4F;
    static constexpr uint8_t EXPECTED_WHO_AM_I = 0x40;

    static constexpr uint8_t CFG_REG_A  = 0x60;
    static constexpr uint8_t CFG_REG_B  = 0x61;
    static constexpr uint8_t CFG_REG_C  = 0x62;
    static constexpr uint8_t INT_CTRL_REG  = 0x63;
    static constexpr uint8_t STATUS_REG = 0x67;
    static constexpr uint8_t OFFSET_X_REG_L = 0x45;

    static constexpr uint8_t OUT_X_L    = 0x68;

    int handle_ = -1;
};
