#pragma once

#include <iostream>
#include <pigpio.h>
#include <unistd.h> // For usleep
#include <stdexcept>
#include <cmath>
#include <chrono>

#include "gpio.h"
//#define DEBUG_DPS

class DPS310 : public gpio {
    private:
        int handle_;
    
        // Scaling factors (kP/kT) for pressure and temperature based on their oversampling rates
        double pressure_scale_factor_;
        double temp_scale_factor_;
    
        // Last compensated temperature value
        float last_temperature_;
        double t_raw = 0.0;
    
        // calibration coefficients
        struct Coefficients {
            int32_t c0, c1, c00, c01, c10, c11, c20, c21, c30;
        } coeffs_;
    
        // Register definitions
        static constexpr uint8_t REG_PROD_ID = 0x0D; // Product ID register
        static constexpr uint8_t REG_PSR_B2 = 0x00;  // Pressure data, MSB
        static constexpr uint8_t REG_TMP_B2 = 0x03;  // Temperature data, MSB
        static constexpr uint8_t REG_PRS_CFG = 0x06; // Pressure config
        static constexpr uint8_t REG_TMP_CFG = 0x07; // Temperature config
        static constexpr uint8_t REG_MEAS_CFG = 0x08;// Measurement config
        static constexpr uint8_t REG_CFG_REG = 0x09; // Configuration register
        static constexpr uint8_t REG_COEFF = 0x10;   // Calibration coefficients start
    
        int32_t twosComplement(int32_t val, uint8_t bits) {
            if (val & ((uint32_t)1u << (bits - 1))) {
                val -= (uint32_t)1u << bits;
            }
            return static_cast<int32_t>(val);;
        }
    
        // Function to read multiple registers over I2C
        bool readRegisters(uint8_t reg, char* buffer, int length) {
            int result = i2cReadI2CBlockData(handle_, reg, buffer, length);
            if (result < 0) {
                std::cerr << "[DPS310] Failed to read registers starting at 0x" << std::hex << (int)reg << std::endl;
                return false;
            }
            return true;
        }
    
        // Function to read calibration coefficients
        bool readCalibrationCoefficients() {
            char coeff_data[18];
            if (!readRegisters(REG_COEFF, coeff_data, 18)) {
                return false;
            }
    
            coeffs_.c0 = ((uint16_t)coeff_data[0] << 4) | (((uint16_t)coeff_data[1] >> 4) & 0x0F);
            coeffs_.c0 = twosComplement(coeffs_.c0, 12);
    
            coeffs_.c1 = twosComplement((((uint16_t)coeff_data[1] & 0x0F) << 8) | coeff_data[2], 12);
            coeffs_.c00 = ((uint32_t)coeff_data[3] << 12) | ((uint32_t)coeff_data[4] << 4) |
                         (((uint32_t)coeff_data[5] >> 4) & 0x0F);
            coeffs_.c00 = twosComplement(coeffs_.c00, 20);
    
            coeffs_.c10 = (((uint32_t)coeff_data[5] & 0x0F) << 16) | ((uint32_t)coeff_data[6] << 8) |
                         (uint32_t)coeff_data[7];
            coeffs_.c10 = twosComplement(coeffs_.c10, 20);
            
            coeffs_.c01 = twosComplement(((uint16_t)coeff_data[8] << 8) | (uint16_t)coeff_data[9], 16);
            coeffs_.c11 = twosComplement(((uint16_t)coeff_data[10] << 8) | (uint16_t)coeff_data[11], 16);
            coeffs_.c20 = twosComplement(((uint16_t)coeff_data[12] << 8) | (uint16_t)coeff_data[13], 16);
            coeffs_.c21 = twosComplement(((uint16_t)coeff_data[14] << 8) | (uint16_t)coeff_data[15], 16);
            coeffs_.c30 = twosComplement(((uint16_t)coeff_data[16] << 8) | (uint16_t)coeff_data[17], 16);
    
            #ifdef DEBUG_DPS
                std::cout << "[DPS310] Calibration Coefficients:" << std::endl;
                std::cout << "c0: "   << coeffs_.c0 << std::endl;
                std::cout << "c1: "   << coeffs_.c1 << std::endl;
                std::cout << "c00: "  << coeffs_.c00 << std::endl;
                std::cout << "c01: "  << coeffs_.c01 << std::endl;
                std::cout << "c10: "  << coeffs_.c10 << std::endl;
                std::cout << "c11: "  << coeffs_.c11 << std::endl;
                std::cout << "c20: "  << coeffs_.c20 << std::endl;
                std::cout << "c21: "  << coeffs_.c21 << std::endl;
                std::cout << "c30: "  << coeffs_.c30 << std::endl;
            #endif
    
            return true;
        }
        // Helper function to get the maximum measurement rate (Hz) based on oversampling rate (Table 17)
        int getMaxMeasurementRate(int oversampling_rate) {
            switch (oversampling_rate) {
                case 1:   return 128;
                case 2:   return 128;
                case 4:   return 128;
                case 8:   return 64;
                case 16:  return 32;
                case 32:  return 16;
                case 64:  return 8;
                case 128: return 4;
                default:
                    throw std::invalid_argument("[DPS310] Invalid oversampling rate! Must be 1, 2, 4, 8, 16, 32, 64, or 128.");
            }
        }
    
        // Helper function to get measurement time (ms) based on oversampling rate
        double getMeasurementTime(int oversampling_rate) {
            switch (oversampling_rate) {
                case 1:   return 3.6;
                case 2:   return 5.2;
                case 4:   return 8.4;
                case 8:   return 14.8;
                case 16:  return 27.6;
                case 32:  return 53.2;
                case 64:  return 104.4;
                case 128: return 206.8;
                default:
                    throw std::invalid_argument("[DPS310] Invalid oversampling rate! Must be 1, 2, 4, 8, 16, 32, 64, or 128.");
            }
        }
    
        // Helper function to check if the combination of measurement rates and oversampling rates is valid
        void checkMeasurementConstraint(int pressure_measurement_rate, int pressure_oversampling_rate,
            int temp_measurement_rate, int temp_oversampling_rate) {
            // Step 1: Check maximum rate limits from Table 17
            int max_pressure_rate = getMaxMeasurementRate(pressure_oversampling_rate);
            int max_temp_rate = getMaxMeasurementRate(temp_oversampling_rate);
    
            if (pressure_measurement_rate > max_pressure_rate) {
            throw std::invalid_argument(
                "[DPS310] Pressure measurement rate " + std::to_string(pressure_measurement_rate) +
                " Hz exceeds maximum allowed rate of " + std::to_string(max_pressure_rate) +
                " Hz for oversampling rate " + std::to_string(pressure_oversampling_rate) + "x (Table 17)."
            );
            }
    
            if (temp_measurement_rate > max_temp_rate) {
            throw std::invalid_argument(
                "[DPS310] Temperature measurement rate " + std::to_string(temp_measurement_rate) +
                " Hz exceeds maximum allowed rate of " + std::to_string(max_temp_rate) +
                " Hz for oversampling rate " + std::to_string(temp_oversampling_rate) + "x (Table 17)."
            );
            }
    
            // Step 2: Check total time constraint
            double pressure_measurement_time = getMeasurementTime(pressure_oversampling_rate); // in ms per measurement
            double temp_measurement_time = getMeasurementTime(temp_oversampling_rate);         // in ms per measurement
    
            // Calculate total time in 1 second (1000 ms):
            // - pressure_measurement_rate is in Hz (measurements per second)
            // - pressure_measurement_time is in ms per measurement
            // So, pressure_measurement_rate * pressure_measurement_time gives total time for pressure measurements in 1 second (in ms)
            double pressure_total_time = pressure_measurement_rate * pressure_measurement_time;
            double temp_total_time = temp_measurement_rate * temp_measurement_time;
    
            double total_time = pressure_total_time + temp_total_time;
    
            #ifdef DEBUG_DPS
                std::cout << "[DPS310] Measurement Constraint Check:\n"
                    << "Pressure: Rate = " << pressure_measurement_rate
                    << " Hz, Oversampling = " << pressure_oversampling_rate
                    << "x, Max Rate = " << max_pressure_rate
                    << " Hz, Measurement Time = " << pressure_measurement_time
                    << " ms, Total Time = " << pressure_total_time << " ms\n"
                    << "Temperature: Rate = " << temp_measurement_rate
                    << " Hz, Oversampling = " << temp_oversampling_rate
                    << "x, Max Rate = " << max_temp_rate
                    << " Hz, Measurement Time = " << temp_measurement_time
                    << " ms, Total Time = " << temp_total_time << " ms\n"
                    << "Total Time in 1 second = " << total_time << " ms\n";
            #endif
            // Constraint: Total time for all measurements in 1 second must be <= 1000 ms
            if (total_time > 1000.0) {
                throw std::invalid_argument(
                    "[DPS310] Invalid combination of measurement rates and oversampling rates.\n"
                    "Pressure: Rate = " + std::to_string(pressure_measurement_rate) +
                    " Hz, Oversampling = " + std::to_string(pressure_oversampling_rate) +
                    "x, Total Time = " + std::to_string(pressure_total_time) + " ms\n" +
                    "Temperature: Rate = " + std::to_string(temp_measurement_rate) +
                    " Hz, Oversampling = " + std::to_string(temp_oversampling_rate) +
                    "x, Total Time = " + std::to_string(temp_total_time) + " ms\n" +
                    "Total time (" + std::to_string(total_time) +
                    " ms) exceeds 1000 ms. Must satisfy (Pressure Rate × Pressure Measurement Time) + "
                    "(Temperature Rate × Temperature Measurement Time) ≤ 1000 ms."
                );
            }
        }
    
    
        // Helper function to convert measurement rate (Hz) to register bits
        uint8_t getMeasurementRateBits(int rate) {
            switch (rate) {
                case 1:  return 0x00; // 000
                case 2:  return 0x10; // 001
                case 4:  return 0x20; // 010
                case 8:  return 0x30; // 011
                case 16: return 0x40; // 100
                case 32: return 0x50; // 101
                case 64: return 0x60; // 110
                case 128: return 0x70; // 111
                default:
                    throw std::invalid_argument("[DPS310] Invalid measurement rate! Must be 1, 2, 4, 8, 16, 32, 64, or 128 Hz.");
            }
        }
    
        // Helper function to convert oversampling rate to register bits and scaling factor
        uint8_t getOversamplingBits(int rate, double& scale_factor, bool& enable_shift) {
            switch (rate) {
                case 1:   scale_factor = 524288;  enable_shift = false; return 0x00; // 000
                case 2:   scale_factor = 1572864; enable_shift = false; return 0x01; // 001
                case 4:   scale_factor = 3670016; enable_shift = false; return 0x02; // 010
                case 8:   scale_factor = 7864320; enable_shift = false; return 0x03; // 011
                case 16:  scale_factor = 253952;  enable_shift = true;  return 0x04; // 100
                case 32:  scale_factor = 516096;  enable_shift = true;  return 0x05; // 101
                case 64:  scale_factor = 1040384; enable_shift = true;  return 0x06; // 110
                case 128: scale_factor = 2088960; enable_shift = true;  return 0x07; // 111
                default:
                    throw std::invalid_argument("[DPS310] Invalid oversampling rate! Must be 1, 2, 4, 8, 16, 32, 64, or 128.");
            }
        }
    
    public:
        // Constructor with separate measurement and oversampling rates for pressure and temperature
        DPS310(int pressure_measurement_rate = 128, int pressure_oversampling_rate = 16,
               int temp_measurement_rate = 128, int temp_oversampling_rate = 16)
            : handle_(-1), pressure_scale_factor_(0.0), temp_scale_factor_(0.0) {
    
            // Check if the combination of measurement rates and oversampling rates is valid
            checkMeasurementConstraint(pressure_measurement_rate, pressure_oversampling_rate,
                                       temp_measurement_rate, temp_oversampling_rate);
    
            
            // Step 2: Open I2C connection to DPS310
            handle_ = i2cOpenBus(1, 0x77); // I2C bus 1, address 0x77
            if (handle_ < 0) {
                gpioTerminate();
                throw std::runtime_error("[DPS310] Failed to open I2C device!");
            }
    
            // Step 3: Check product ID to verify sensor is responding
            char prod_id;
            if (!readRegisters(REG_PROD_ID, &prod_id, 1)) {
                i2cCloseBus(handle_);
                gpioTerminate();
                throw std::runtime_error("[DPS310] Failed to read product ID!");
            }
            if (prod_id != 0x10) {
                i2cCloseBus(handle_);
                gpioTerminate();
                throw std::runtime_error("[DPS310] Invalid product ID: 0x" + std::to_string(static_cast<int>(prod_id)) +
                                        ", expected 0x10");
            }
            #ifdef DEBUG_DPS
                std::cout << "[DPS310] Product ID verified: 0x" << std::hex << static_cast<int>(prod_id) << std::endl;
                std::cout << std::dec; // Reset to decimal
            #endif
            // Step 4: Read calibration coefficients
            if (!readCalibrationCoefficients()) {
                i2cCloseBus(handle_);
                gpioTerminate();
                throw std::runtime_error("[DPS310] Failed to read calibration coefficients!");
            }
    
            // Step 5: Configure the DPS310 with specified measurement and oversampling rates
            // Configure pressure
            uint8_t pressure_meas_rate_bits = getMeasurementRateBits(pressure_measurement_rate);
            bool pressure_enable_shift;
            uint8_t pressure_oversampling_bits = getOversamplingBits(pressure_oversampling_rate,
                                                                    pressure_scale_factor_,
                                                                    pressure_enable_shift);
    
            // Configure temperature
            uint8_t temp_meas_rate_bits = getMeasurementRateBits(temp_measurement_rate);
            bool temp_enable_shift;
            uint8_t temp_oversampling_bits = getOversamplingBits(temp_oversampling_rate,
                                                                 temp_scale_factor_,
                                                                 temp_enable_shift);
    
            // Combine measurement rate and oversampling bits for pressure and temperature
            uint8_t prs_cfg_value = pressure_meas_rate_bits | pressure_oversampling_bits;
            uint8_t tmp_cfg_value = temp_meas_rate_bits | temp_oversampling_bits; // Bit 7 = 0 for internal sensor
    
            if (i2cWriteByte(handle_, REG_PRS_CFG, prs_cfg_value) < 0) {
                i2cCloseBus(handle_);
                gpioTerminate();
                throw std::runtime_error("[DPS310] Failed to configure pressure settings!");
            }
            if (i2cWriteByte(handle_, REG_TMP_CFG, tmp_cfg_value) < 0) {
                i2cCloseBus(handle_);
                gpioTerminate();
                throw std::runtime_error("[DPS310] Failed to configure temperature settings!");
            }
    
            // Configure bit shifting based on oversampling rates
            uint8_t cfg_reg_value = 0x00;
            if (pressure_enable_shift) {
                cfg_reg_value |= 0x04; // Set bit 2 (PRS_SHIFT_EN)
            }
            if (temp_enable_shift) {
                cfg_reg_value |= 0x08; // Set bit 3 (TMP_SHIFT_EN)
            }
            if (i2cWriteByte(handle_, REG_CFG_REG, cfg_reg_value) < 0) {
                i2cCloseBus(handle_);
                gpioTerminate();
                throw std::runtime_error("[DPS310] Failed to configure bit shifting!");
            }
    
            // Enable continuous measurement for both pressure and temperature
            if (i2cWriteByte(handle_, REG_MEAS_CFG, 0x07) < 0) {
                i2cCloseBus(handle_);
                gpioTerminate();
                throw std::runtime_error("[DPS310] Failed to enable continuous measurement!");
            }
    
            // Step 6: Wait for sensor to stabilize
            usleep(100000); // 100 ms delay
            #ifdef DEBUG_DPS
                std::cout << "[DPS310] initialized successfully with:\n"
                          << "Pressure measurement rate: " << pressure_measurement_rate
                          << " Hz, oversampling rate: " << pressure_oversampling_rate << "x\n"
                          << "Temperature measurement rate: " << temp_measurement_rate
                          << " Hz, oversampling rate: " << temp_oversampling_rate << "x" << std::endl;
            #endif
        }
    
        // Destructor (handles cleanup)
        ~DPS310() {
            if (handle_ >= 0) {
                i2cCloseBus(handle_);
            }
            #ifdef DEBUG_DPS
                std::cout << "[DPS310] cleanup completed." << std::endl;
            #endif
        }
    
        // Function to check if a new pressure or temperature measurement is available
        bool  isMeasurementReady(bool& pressure_ready, bool& temp_ready) {
            char meas_cfg;
            if (!readRegisters(REG_MEAS_CFG, &meas_cfg, 1)) {
                std::cerr << "[DPS310] Failed to read measurement configuration register!" << std::endl;
                return false;
            }
    
            // Check if pressure (bit 4) and temperature (bit 5) measurements are ready
            pressure_ready = (meas_cfg & (1 << 4)) != 0;
    
            temp_ready = (meas_cfg & (1 << 5)) != 0;
    
            //std::cout << "Measurement readiness - Pressure ready: " << (pressure_ready ? "Yes" : "No")
                //         << ", Temperature ready: " << (temp_ready ? "Yes" : "No") << std::endl;
            // Only require pressure to be ready to proceed
            return pressure_ready;
        }

        // Function to read and compensate pressure and temperature data
        bool readData(double& pressure, double& temperature, bool temp_ready, bool pressure_ready) {
            bool success = false;
        
            if (temp_ready) {
                char temp_data_raw[3];
                if (i2cReadI2CBlockData(handle_, REG_TMP_B2, temp_data_raw, 3) < 0) {
                    std::cerr << "[DPS310] Failed to read temp data!\n";
                } else {
                    uint8_t d[3] = {static_cast<uint8_t>(temp_data_raw[0]),
                                    static_cast<uint8_t>(temp_data_raw[1]),
                                    static_cast<uint8_t>(temp_data_raw[2])};
                    int32_t raw = (int32_t(d[0]) << 16) | (int32_t(d[1]) << 8) | d[2];
                    raw = twosComplement(raw, 24);
                    
                    double Traw_sc = static_cast<double>(raw) / temp_scale_factor_;
                    last_temperature_ = static_cast<float>(coeffs_.c0 * 0.5 + coeffs_.c1 * Traw_sc) + 27.0;
                    
                    t_raw = Traw_sc;
                    success = true;
                }
            }
    
            if(pressure_ready)
            {
                // Read 3 bytes of pressure data
                char pressure_data_raw[3];
                if (i2cReadI2CBlockData(handle_, REG_PSR_B2, pressure_data_raw, 3) < 0) {
                    std::cerr << "[DPS310] Failed to read pressure data!" << std::endl;
                    return false;
                }
                // Safely cast each byte to uint8_t for bitwise work
                uint8_t pres_data[3] = {
                    static_cast<uint8_t>(pressure_data_raw[0]),
                    static_cast<uint8_t>(pressure_data_raw[1]),
                    static_cast<uint8_t>(pressure_data_raw[2])
                };
                int32_t raw_pressure = (static_cast<int32_t>(pres_data[0]) << 16) |
                (static_cast<int32_t>(pres_data[1]) << 8)  |
                (static_cast<int32_t>(pres_data[2]));
                raw_pressure = twosComplement(raw_pressure, 24);

                double p_raw = static_cast<double>(raw_pressure) / pressure_scale_factor_;

                pressure = static_cast<double>(coeffs_.c00) +
                           p_raw * (static_cast<double>(coeffs_.c10) + p_raw * (static_cast<double>(coeffs_.c20) + p_raw * static_cast<double>(coeffs_.c30))) +
                           t_raw * static_cast<double>(coeffs_.c01) +
                           t_raw * p_raw * (static_cast<double>(coeffs_.c11) + p_raw * static_cast<double>(coeffs_.c21));

                success = true;
            }

            temperature = last_temperature_;
            return success;
        }
        // Updated function to calculate altitude and altitude change with temperature compensation
        bool calculateAltitudeChange(const double pressure, const double temperature, double& altitude, double& altitude_change) {
            // Constants for the hypsometric equation
            const double P0 = 101325.0; // Standard sea-level pressure in Pa
            const double R = 8.314; // Universal gas constant (J/mol*K)
            const double g = 9.80665; // Gravitational acceleration (m/s^2)
            const double M = 0.0289644; // Molar mass of dry air (kg/mol)
    
            // Convert temperature to Kelvin
            double T = temperature + 273.15;
    
            // Check for valid temperature and pressure
            if (T <= 0.0 || pressure <= 0.0) {
                std::cerr << "[DPS310]Invalid temperature (" << temperature << " °C) or pressure (" << pressure << " Pa) for altitude calculation!" << std::endl;
                return false;
            }
    
            // Calculate the scale height factor
            double scale_factor = (R * T) / (g * M);
    
            // Calculate altitude using the hypsometric equation
            double last_altitude = scale_factor * std::log(P0 / pressure);
            altitude_change = last_altitude - altitude;
            altitude = last_altitude;
    
            return true;
        }
    };