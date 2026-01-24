#pragma once

#include <pigpio.h>
#include <iostream>
#include <cstdint>

//#define DEBUG_LIS3MDL  // Uncomment for debug output

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
        // Values include the specific OM (Performance Mode) bits needed for each speed:
        Hz_1000  = 0x02 | 0x00, // Low Power (OM=00)
        Hz_560   = 0x02 | 0x20, // Medium (OM=01)
        Hz_300   = 0x02 | 0x40, // High (OM=10)
        Hz_155   = 0x02 | 0x60  // Ultra-High (OM=11)
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
        initCalibrationMatrix();
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
    bool readCalibrated(double& mx, double& my, double& mz)
    {
        double rx, ry, rz;
        if (!read_gauss(rx, ry, rz)) return false;

        double dx = rx - hardIronOffset[0];
        double dy = ry - hardIronOffset[1];
        double dz = rz - hardIronOffset[2];

        mx = calibMatrix[0][0] * dx + calibMatrix[0][1] * dy + calibMatrix[0][2] * dz;
        my = calibMatrix[1][0] * dx + calibMatrix[1][1] * dy + calibMatrix[1][2] * dz;
        mz = calibMatrix[2][0] * dx + calibMatrix[2][1] * dy + calibMatrix[2][2] * dz;

        return true;
    }

    bool dataReady() const {
        int status = i2cReadByteData(handle_, STATUS_REG);
        if (status < 0) {
            return false;
        }
        return status & 0x08;
    }
private:
    // ---------------- Initialization ----------------
void init(FullScale range, ODR odr) {
    uint8_t odr_val = static_cast<uint8_t>(odr);
    
    // Check if bit 1 (FAST_ODR) is set
    bool fast = (odr_val & 0x02) != 0;
    
    uint8_t ctrl1 = 0;
    uint8_t ctrl4 = 0;

    if (fast) {
        // In Fast Mode, we use the Enum value directly (contains OM and FAST bits)
        ctrl1 = odr_val;

        // Synchronize Z-axis: Extract OM bits (6:5) and shift to Z-pos (3:2)
        uint8_t om_bits = (odr_val & 0x60);
        ctrl4 = (om_bits >> 3); 
    } else {
        // Standard Mode (<= 80Hz): Default to Ultra-High Performance (0x60)
        // and include the DO bits from the Enum
        ctrl1 = 0x60 | odr_val; 
        ctrl4 = 0x0C; // Z-axis Ultra-High (bits 3:2)
    }

    // Write to device
    i2cWriteByteData(handle_, CTRL_REG1, ctrl1);
    i2cWriteByteData(handle_, CTRL_REG2, static_cast<uint8_t>(range));
    i2cWriteByteData(handle_, CTRL_REG3, 0x00); // Continuous mode
    i2cWriteByteData(handle_, CTRL_REG4, ctrl4);
    i2cWriteByteData(handle_, CTRL_REG5, 0x40); // Block Data Update (BDU)

    scale = getLSBperGauss();

#ifdef DEBUG_LIS3MDL
    std::printf("[LIS3MDL] CTRL1: 0x%02X, CTRL4: 0x%02X\n", ctrl1, ctrl4);
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

    void initCalibrationMatrix()
    {
        // R matrix
        constexpr double R[3][3] = {
            {-0.63130273, 0.09122449, 0.77015255},
            {-0.76346217, 0.10144869, -0.63783515},
            {0.13631715, 0.99064941, -0.00560161}
        };

        // S diagonal scaling
        constexpr double S_diag[3][3] = {{2.20677668,0.0,0.0 },
                                         {0.0, 2.13826103, 0.0},
                                         {0.0, 0.0, 2.05586751}};

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                calibMatrix[i][j] = 0.0;
                for (int k = 0; k < 3; ++k) {
                    calibMatrix[i][j] += S_diag[i][k] * R[j][k];  
                }
            }
        }

        #ifdef DEBUG_LIS3MDL
        std::cout << "[LIS3MDL] Calibration matrix (S*R^T):\n";
        for (int i = 0; i < 3; ++i) {
            std::cout << "  ";
            for (int j = 0; j < 3; ++j) {
                std::cout << calibMatrix[i][j] << (j<2 ? ", " : "");
            }
            std::cout << "\n";
        }
        #endif
    }
    double calibMatrix[3][3] = {0};
    static constexpr double hardIronOffset[3] = {
        0.03628057,    
        0.02168861,
        0.15742409
    };
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
