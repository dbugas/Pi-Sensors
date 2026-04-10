# pragma once

#include <iostream>
#include <pigpio.h>
#include <unistd.h> // For usleep
#include <stdexcept>
#include <cmath>
#include <chrono>
#include <thread>

#include "../../header/gpio.h"

#define DEBUG_DPS368

class DPS368 : public gpio {
    public:
        enum class MeasurementRate : uint8_t
        {
            Hz1   = 0b000,
            Hz2   = 0b001,
            Hz4   = 0b010,
            Hz8   = 0b011,
            Hz16  = 0b100,
            Hz32  = 0b101,
            Hz64  = 0b110,
            Hz128 = 0b111
        };

        enum class OversamplingRate : uint8_t
        {
            OSR1   = 0b0000,
            OSR2   = 0b0001,
            OSR4   = 0b0010,
            OSR8   = 0b0011,
            OSR16  = 0b0100,
            OSR32  = 0b0101,
            OSR64  = 0b0110,
            OSR128 = 0b0111
        };

        DPS368(MeasurementRate prs_rate,OversamplingRate prs_osr,
            MeasurementRate tmp_rate, OversamplingRate tmp_osr){
            init_gpio();

            handle = spiOpen(CS, SPI_BAUD, SPI_FLAGS);

            uint8_t id;
            if (!readRegister(REG_PROD_ID, &id)) return;

            if (id != 0x10) {
                std::cout << "DPS368 not found! ID: " << (int)id << std::endl;
                throw std::runtime_error("DPS368 not found!");
            }

            checkMeasurementConstraint(prs_rate, prs_osr, tmp_rate, tmp_osr);
            std::this_thread::sleep_for(std::chrono::microseconds(100000));

            readCalibrationCoefficients();
            setSensorConfig( prs_rate,  prs_osr, tmp_rate,  tmp_osr);
        }
        bool  isMeasurementReady(bool& pressure_ready, bool& temp_ready) {
            uint8_t meas_cfg;
            if (!readRegister(REG_MEAS_CFG, &meas_cfg)) {
                std::cerr << "Failed to read measurement configuration register!" << std::endl;
                return false;
            }
    
            // Check if pressure (bit 4) and temperature (bit 5) measurements are ready
            pressure_ready = (meas_cfg & (1 << 4)) != 0;
            temp_ready = (meas_cfg & (1 << 5)) != 0;
    
            #ifdef DEBUG_DPS368
                std::cout << "[DPS368] Measurement readiness - Pressure ready: " << (pressure_ready ? "Yes" : "No")
                         << ", Temperature ready: " << (temp_ready ? "Yes" : "No") << std::endl;
            #endif
            return pressure_ready || temp_ready;
        }


        bool readCalibrated(float& pressure_Pa, float& temperature_C)
        {
            bool pressure_ready = false;
            bool temp_ready = false;
            if(!isMeasurementReady(pressure_ready, temp_ready)) return false;

            int32_t Praw, Traw;
            if (!readRaw(Praw, Traw))
                return false;
        
            // Apply scaling
            float Praw_sc = (float)Praw / pressure_scale_factor_;
            float Traw_sc = (float)Traw / temp_scale_factor_;
        
            // Temperature (C)
            temperature_C = coeffs_.c0 * 0.5f + coeffs_.c1 * Traw_sc;
        
            // Pressure (Pa)
            pressure_Pa =
                coeffs_.c00 + Praw_sc * (coeffs_.c10 + Praw_sc * (coeffs_.c20 + Praw_sc *  coeffs_.c30)) 
                + Traw_sc * coeffs_.c01 + Traw_sc * Praw_sc * (coeffs_.c11 + Praw_sc * coeffs_.c21);
                
            return true;
        }

        ~DPS368() {
            enterLowPowerMode();
            if (handle >= 0) {
                spiClose(handle);
            }
            close_gpio();
        }
    private:

        int handle = -1;
        static constexpr unsigned SPI_BAUD  = 6000000;   
        static constexpr unsigned SPI_FLAGS = 3;
        static constexpr unsigned CS        = 1;

        // Register definitions
        static constexpr uint8_t REG_PROD_ID = 0x0D; // Product ID register
        static constexpr uint8_t REG_PSR_B2 = 0x00;  // Pressure data, MSB
        static constexpr uint8_t REG_TMP_B2 = 0x03;  // Temperature data, MSB
        static constexpr uint8_t REG_PRS_CFG = 0x06; // Pressure config
        static constexpr uint8_t REG_TMP_CFG = 0x07; // Temperature config
        static constexpr uint8_t REG_MEAS_CFG = 0x08;// Measurement config
        static constexpr uint8_t REG_CFG_REG = 0x09; // Configuration register
        static constexpr uint8_t REG_COEFF = 0x10;   // Calibration coefficients start

        double pressure_scale_factor_;
        double temp_scale_factor_;

        struct Coefficients {
            int32_t c0, c1, c00, c01, c10, c11, c20, c21, c30;
        } coeffs_;

        int32_t twosComplement(int32_t val, uint8_t bits) {
            if (val & ((uint32_t)1u << (bits - 1))) {
                val -= (uint32_t)1u << bits;
            }
            return static_cast<int32_t>(val);;
        }

        bool setSensorConfig(MeasurementRate prs_rate, OversamplingRate prs_osr,
                         MeasurementRate tmp_rate, OversamplingRate tmp_osr)
        {
            // Write config registers
            uint8_t prs_cfg = (static_cast<uint8_t>(prs_rate) << 4) |
                               static_cast<uint8_t>(prs_osr);

            uint8_t tmp_cfg = (static_cast<uint8_t>(tmp_rate) << 4) |
                               static_cast<uint8_t>(tmp_osr);

            writeRegister(REG_PRS_CFG, prs_cfg);
            writeRegister(REG_TMP_CFG, tmp_cfg);
            // Handle SHIFT_EN
            uint8_t cfg = 0b0;

            if (static_cast<uint8_t>(prs_osr) >= static_cast<uint8_t>(OversamplingRate::OSR16))
                cfg |= (1 << 2);

            if (static_cast<uint8_t>(tmp_osr) >= static_cast<uint8_t>(OversamplingRate::OSR16))
                cfg |= (1 << 3);

            writeRegister(REG_CFG_REG, cfg);

            // Store scale factors
            pressure_scale_factor_ = getScaleFactor(prs_osr);
            temp_scale_factor_ = getScaleFactor(tmp_osr);

            writeRegister(REG_MEAS_CFG,0b00000111); // continuous pressure and temperature measurment

            return true;
        }
        void writeRegister(uint8_t reg, uint8_t value)
        {
            char tx[2];
            tx[0] = reg & 0x7F;
            tx[1] = value;
        
            spiWrite(handle, tx, 2);
        }
        bool readRegister(uint8_t reg, uint8_t* value)
        {
            char tx[2];
            char rx[2];

            tx[0] = reg | 0x80;
            tx[1] = 0x00;

            int result = spiXfer(handle, tx, rx, 2);

            if (result < 0)
                return false;

            *value = static_cast<uint8_t>(rx[1]);
            return true;
        }
        bool readRegisters(int spi, uint8_t startReg, uint8_t* buffer, int length)
        {
            std::vector<char> tx(length + 1, 0);
            std::vector<char> rx(length + 1, 0);
        
            tx[0] = startReg | 0x80; // read bit
        
            int result = spiXfer(spi, tx.data(), rx.data(), length + 1);
        
            if (result < 0) return false;
        
            for (int i = 0; i < length; i++)
            {
                buffer[i] = static_cast<uint8_t>(rx[i + 1]);
            }
        
            return true;
        }

        bool readCalibrationCoefficients()
        {
            uint8_t coeff_data[18];
        
            if (!readRegisters(handle, REG_COEFF, coeff_data, 18))
            {
                return false;
            }
        
            coeffs_.c0 = ((uint16_t)coeff_data[0] << 4) | (((uint16_t)coeff_data[1] >> 4) & 0x0F);
            coeffs_.c0 = twosComplement(coeffs_.c0, 12);
            coeffs_.c1 = (((uint16_t)coeff_data[1] & 0x0F) << 8) |(uint16_t)coeff_data[2];
            coeffs_.c1 = twosComplement(coeffs_.c1, 12);
            coeffs_.c00 = ((uint32_t)coeff_data[3] << 12) |((uint32_t)coeff_data[4] << 4) |(((uint32_t)coeff_data[5] >> 4) & 0x0F);
            coeffs_.c00 = twosComplement(coeffs_.c00, 20);
            coeffs_.c10 = (((uint32_t)coeff_data[5] & 0x0F) << 16) |((uint32_t)coeff_data[6] << 8) |(uint32_t)coeff_data[7];
            coeffs_.c10 = twosComplement(coeffs_.c10, 20);
            coeffs_.c01 = twosComplement(((uint16_t)coeff_data[8] << 8) | (uint16_t)coeff_data[9], 16);
            coeffs_.c11 = twosComplement( ((uint16_t)coeff_data[10] << 8) | (uint16_t)coeff_data[11], 16);
            coeffs_.c20 = twosComplement(((uint16_t)coeff_data[12] << 8) | (uint16_t)coeff_data[13], 16);
            coeffs_.c21 = twosComplement(((uint16_t)coeff_data[14] << 8) | (uint16_t)coeff_data[15], 16);
            coeffs_.c30 = twosComplement(((uint16_t)coeff_data[16] << 8) | (uint16_t)coeff_data[17], 16);
            
            #ifdef DEBUG_DPS368
                std::cout << "[DPS368] Calibration Coefficients:\n";
                std::cout << "c0:  " << coeffs_.c0  << "\n";
                std::cout << "c1:  " << coeffs_.c1  << "\n";
                std::cout << "c00: " << coeffs_.c00 << "\n";
                std::cout << "c01: " << coeffs_.c01 << "\n";
                std::cout << "c10: " << coeffs_.c10 << "\n";
                std::cout << "c11: " << coeffs_.c11 << "\n";
                std::cout << "c20: " << coeffs_.c20 << "\n";
                std::cout << "c21: " << coeffs_.c21 << "\n";
                std::cout << "c30: " << coeffs_.c30 << "\n";
            #endif
            return true;
        }
        float getScaleFactor(OversamplingRate osr)
        {
            switch (osr)
            {
                case OversamplingRate::OSR1:   return 524288.0f;
                case OversamplingRate::OSR2:   return 1572864.0f;
                case OversamplingRate::OSR4:   return 3670016.0f;
                case OversamplingRate::OSR8:   return 7864320.0f;
                case OversamplingRate::OSR16:  return 253952.0f;
                case OversamplingRate::OSR32:  return 516096.0f;
                case OversamplingRate::OSR64:  return 1040384.0f;
                case OversamplingRate::OSR128: return 2088960.0f;
                default: return 1.0f;
            }
        }

        bool readRaw(int32_t& Praw, int32_t& Traw)
        {
            uint8_t buffer[6];
        
            // Read pressure (0x00–0x02) + temperature (0x03–0x05)
            if (!readRegisters(handle, REG_PSR_B2, buffer, 6))
                return false;
        
            // Pressure (24-bit signed)
            Praw = ((int32_t)buffer[0] << 16) |
                   ((int32_t)buffer[1] << 8)  |
                   (int32_t)buffer[2];
        
            if (Praw & 0x800000) Praw |= 0xFF000000;
        
            // Temperature (24-bit signed)
            Traw = ((int32_t)buffer[3] << 16) |
                   ((int32_t)buffer[4] << 8)  |
                   (int32_t)buffer[5];
        
            if (Traw & 0x800000) Traw |= 0xFF000000;
        
            return true;
        }

        void enterLowPowerMode() {
            // MEAS_CTRL = 000 = Standby / Idle mode
            writeRegister(REG_MEAS_CFG, 0x00);

            #ifdef DEBUG_DPS368
                std::cout << "[DPS368] Entered Low Power Standby mode" << std::endl;
            #endif
        }

         void checkMeasurementConstraint(MeasurementRate prs_rate,OversamplingRate prs_osr,
                                            MeasurementRate tmp_rate, OversamplingRate tmp_osr) {
            // Step 1: Check maximum rate limits from Table 17 of data sheet
            int pressure_measurement_rate  = toHz(prs_rate);
            int temp_measurement_rate      = toHz(tmp_rate);                            
            int pressure_oversampling_rate = toOversampling(prs_osr);
            int temp_oversampling_rate     = toOversampling(tmp_osr);  

            int max_pressure_rate = getMaxMeasurementRate(pressure_oversampling_rate);
            int max_temp_rate = getMaxMeasurementRate(temp_oversampling_rate);
    
            if (pressure_measurement_rate > max_pressure_rate) {
            throw std::invalid_argument(
                "[DPS368] Pressure measurement rate " + std::to_string(pressure_measurement_rate) +
                " Hz exceeds maximum allowed rate of " + std::to_string(max_pressure_rate) +
                " Hz for oversampling rate " + std::to_string(pressure_oversampling_rate) + "x (Table 17)."
            );
            }
    
            if (temp_measurement_rate > max_temp_rate) {
            throw std::invalid_argument(
                "[DPS368] Temperature measurement rate " + std::to_string(temp_measurement_rate) +
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
    
            // Debug output to show the calculated times
            #ifdef DEBUG_DPS368
                std::cout << "[DPS368] Measurement Constraint Check:\n"
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
                    "[DPS368] Invalid combination of measurement rates and oversampling rates.\n"
                    "Pressure: Rate = " + std::to_string(pressure_measurement_rate) +
                    " Hz, Oversampling = " + std::to_string(pressure_oversampling_rate) +
                    "x, Total Time = " + std::to_string(pressure_total_time) + " ms\n" +
                    "Temperature: Rate = " + std::to_string(temp_measurement_rate) +
                    " Hz, Oversampling = " + std::to_string(temp_oversampling_rate) +
                    "x, Total Time = " + std::to_string(temp_total_time) + " ms\n" +
                    "Total time (" + std::to_string(total_time) +
                    " ms) exceeds 1000 ms. Must satisfy (Pressure Rate × Pressure Measurement Time) + "
                    "(Temperature Rate × Temperature Measurement Time) <= 1000 ms."
                );
            }
        }

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
                throw std::invalid_argument("Invalid oversampling rate! Must be 1, 2, 4, 8, 16, 32, 64, or 128.");
        }
    }
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
                throw std::invalid_argument("Invalid oversampling rate! Must be 1, 2, 4, 8, 16, 32, 64, or 128.");
        }
    }

    static int toHz(MeasurementRate rate) {
        switch (rate) {
            case MeasurementRate::Hz1:   return 1;
            case MeasurementRate::Hz2:   return 2;
            case MeasurementRate::Hz4:   return 4;
            case MeasurementRate::Hz8:   return 8;
            case MeasurementRate::Hz16:  return 16;
            case MeasurementRate::Hz32:  return 32;
            case MeasurementRate::Hz64:  return 64;
            case MeasurementRate::Hz128: return 128;
            default:
                throw std::invalid_argument("Invalid MeasurementRate enum value");
        }
    }

    static int toOversampling(OversamplingRate osr) {
        switch (osr) {
            case OversamplingRate::OSR1:   return 1;
            case OversamplingRate::OSR2:   return 2;
            case OversamplingRate::OSR4:   return 4;
            case OversamplingRate::OSR8:   return 8;
            case OversamplingRate::OSR16:  return 16;
            case OversamplingRate::OSR32:  return 32;
            case OversamplingRate::OSR64:  return 64;
            case OversamplingRate::OSR128: return 128;
            default:
                throw std::invalid_argument("Invalid OversamplingRate enum value");
        }
    }

};