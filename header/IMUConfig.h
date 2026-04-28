#pragma once

#include <string>
#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "Parser.h"
#include "BMI088.h"
#include "DPS368.h"
#include "LIS2MDL.h"

struct IMUSettings {
    BMI088::AccelRange        accel_scale = BMI088::AccelRange::G_12;
    BMI088::AccelODR          accel_odr   = BMI088::AccelODR::ODR_800Hz;
    BMI088::AccelOversampling accel_osr   = BMI088::AccelOversampling::Normal;
    int accel_timer_us = 1250;

    BMI088::GyroRange         gyro_scale = BMI088::GyroRange::DPS_1000;
    BMI088::GyroBandwidth     gyro_odr   = BMI088::GyroBandwidth::ODR_1000Hz_BW_116Hz;
    int gyro_timer_us = 1000;

    DPS368::MeasurementRate   PressureODR = DPS368::MeasurementRate::Hz16;
    DPS368::OversamplingRate  PressureOSR = DPS368::OversamplingRate::OSR16;
    DPS368::MeasurementRate   TempODR = DPS368::MeasurementRate::Hz1;
    DPS368::OversamplingRate  TempOSR = DPS368::OversamplingRate::OSR1;
    int dps_timer_us = 62000;

    LIS2MDL::ODR magODR = LIS2MDL::ODR::ODR_100HZ;
    int mag_timer_us = 10000;

    int quat_timer_us = 1000;
    int delay_us = 50;
};

class IMUConfig {
public:
    enum class PerformanceMode {
        Ultra,      // Highest ODR + widest bandwidth (~200–500 Hz cutoff). Very low latency, captures fastest dynamics & vibrations, higher noise & power.
        High,       // High ODR + wide bandwidth (~100–230 Hz cutoff). Excellent for most dynamic applications (drones, robotics).
        Medium,     // Moderate ODR + bandwidth (~40–145 Hz cutoff). Balanced response for general motion, lower noise & CPU load.
        Low,        // Lowest ODR + narrowest bandwidth (~10–40 Hz cutoff) Lowest noise & power, suited for slow/static or battery-critical apps.
        Custom      // Customized IMU setting set in the IMUConfig file
    };

    explicit IMUConfig(Parser& parser_) : parser(parser_) {}

    IMUSettings loadfromfile() {
        IMUSettings settings;
        std::string value;
        // Accel scale
        parser.loadVar("accel_scale", value);
        if (auto e = findEntry(accelScaleTable, value)) {
            settings.accel_scale = *e;
        }
        // Accel OSR
        parser.loadVar("accel_osr", value);
        if (auto e = findEntry(accelOSRTable, value)) {
            settings.accel_osr = *e;
        }
        // Accel ODR
        parser.loadVar("accel_odr", value);
        if (auto e = findEntry(accelOdrTable, value)) {
            settings.accel_odr = e->odr;
            settings.accel_timer_us = e->timer;
        }
        // Gyro scale
        parser.loadVar("gyro_range", value);
        if (auto e = findEntry(gyroScaleTable, value)) {
            settings.gyro_scale = *e;
        }
        // Gyro ODR
        parser.loadVar("gyro_bw", value);
        if (auto e = findEntry(gyroOdrTable, value)) {
            settings.gyro_odr = e->odr;
            settings.gyro_timer_us = e->timer;
        }
        // DPS pressure ODR
        parser.loadVar("pressureRate", value);
        if (auto e = findEntry(dpsOdrTable, value)) {
            settings.PressureODR = e->odr;
            settings.dps_timer_us = e->timer;
        }
        // DPS pressure OSR
        parser.loadVar("pressureOSR", value);
        if (auto e = findEntry(dpsOSRTable, value)) {
            settings.PressureOSR = *e;
        }
        // DPS temp ODR
        parser.loadVar("tempRate", value);
        if (auto e = findEntry(dpsOdrTable, value)) {
            settings.TempODR = e->odr;
        }
        // DPS temp OSR
        parser.loadVar("tempOSR", value);
        if (auto e = findEntry(dpsOSRTable, value)) {
            settings.TempOSR = *e;
        }
        // LIS2MDL ODR
        parser.loadVar("Mag_ODR", value);
        if (auto e = findEntry(magOdrTable, value)) {
            settings.magODR = e->odr;
            settings.mag_timer_us = e->timer;
        }

        settings.quat_timer_us = std::min({settings.accel_timer_us, settings.gyro_timer_us, settings.mag_timer_us });
        settings.delay_us = std::round((double)std::min({settings.quat_timer_us, settings.dps_timer_us}) / 10.0);

        return settings;
    }

    IMUSettings load(PerformanceMode mode) {
        IMUSettings settings;
        switch(mode){
                case PerformanceMode::Ultra:
                    settings.accel_scale = BMI088::AccelRange::G_24;   
                    settings.accel_odr   = BMI088::AccelODR::ODR_1600Hz;
                    settings.accel_osr   = BMI088::AccelOversampling::Normal;
                    settings.gyro_scale  = BMI088::GyroRange::DPS_2000;
                    settings.gyro_odr    = BMI088::GyroBandwidth::ODR_2000Hz_BW_532Hz;

                    settings.PressureODR = DPS368::MeasurementRate::Hz64;
                    settings.PressureOSR = DPS368::OversamplingRate::OSR8;
                    settings.TempODR     = DPS368::MeasurementRate::Hz1;
                    settings.TempOSR     = DPS368::OversamplingRate::OSR1;

                    settings.magODR   = LIS2MDL::ODR::ODR_100HZ;

                    settings.accel_timer_us = 625;
                    settings.gyro_timer_us  = 500;
                    settings.mag_timer_us   = 10000;
                    settings.dps_timer_us   = 14100;
                    settings.quat_timer_us  = 625;
                    settings.delay_us       = 50;
                    break;

                case PerformanceMode::High:
                    settings.accel_scale = BMI088::AccelRange::G_12;   
                    settings.accel_odr   = BMI088::AccelODR::ODR_800Hz;
                    settings.accel_osr   = BMI088::AccelOversampling::Normal;
                    settings.gyro_scale  = BMI088::GyroRange::DPS_1000;
                    settings.gyro_odr    = BMI088::GyroBandwidth::ODR_1000Hz_BW_116Hz;

                    settings.PressureODR = DPS368::MeasurementRate::Hz32;
                    settings.PressureOSR = DPS368::OversamplingRate::OSR16;
                    settings.TempODR     = DPS368::MeasurementRate::Hz1;
                    settings.TempOSR     = DPS368::OversamplingRate::OSR1;

                    settings.magODR   = LIS2MDL::ODR::ODR_100HZ;

                    settings.accel_timer_us = 1250;
                    settings.gyro_timer_us  = 1000;
                    settings.mag_timer_us   = 10000;
                    settings.dps_timer_us   = 27600;
                    settings.quat_timer_us  = 1000;
                    settings.delay_us       = 50;
                    break;

                case PerformanceMode::Medium:
                    settings.accel_scale = BMI088::AccelRange::G_6;   
                    settings.accel_odr   = BMI088::AccelODR::ODR_400Hz;
                    settings.accel_osr   = BMI088::AccelOversampling::Normal;
                    settings.gyro_scale  = BMI088::GyroRange::DPS_1000;
                    settings.gyro_odr    = BMI088::GyroBandwidth::ODR_400Hz_BW_47Hz;

                    settings.PressureODR = DPS368::MeasurementRate::Hz32;
                    settings.PressureOSR = DPS368::OversamplingRate::OSR16;
                    settings.TempODR     = DPS368::MeasurementRate::Hz1;
                    settings.TempOSR     = DPS368::OversamplingRate::OSR1;

                    settings.magODR   = LIS2MDL::ODR::ODR_50HZ;

                    settings.accel_timer_us = 2500;
                    settings.gyro_timer_us  = 2500;
                    settings.mag_timer_us   = 20000;
                    settings.dps_timer_us   = 27600;
                    settings.quat_timer_us  = 2500;
                    settings.delay_us       = 250;
                    break;

                case PerformanceMode::Low:
                    settings.accel_scale = BMI088::AccelRange::G_3;   
                    settings.accel_odr   = BMI088::AccelODR::ODR_100Hz;
                    settings.accel_osr   = BMI088::AccelOversampling::OSR2;
                    settings.gyro_scale  = BMI088::GyroRange::DPS_250;
                    settings.gyro_odr    = BMI088::GyroBandwidth::ODR_200Hz_BW_64Hz;

                    settings.PressureODR = DPS368::MeasurementRate::Hz8;
                    settings.PressureOSR = DPS368::OversamplingRate::OSR64;
                    settings.TempODR     = DPS368::MeasurementRate::Hz1;
                    settings.TempOSR     = DPS368::OversamplingRate::OSR1;

                    settings.magODR   = LIS2MDL::ODR::ODR_20HZ;

                    settings.accel_timer_us = 10000;
                    settings.gyro_timer_us  = 5000;
                    settings.mag_timer_us   = 50000;
                    settings.dps_timer_us   = 1011000;
                    settings.quat_timer_us  = 5000;
                    settings.delay_us       = 500;
                    break;
                case PerformanceMode::Custom:
                        settings = loadfromfile();
                    break;
            }

        return settings;
    }

private:
    Parser& parser;

    // Accel scale
    static constexpr std::pair<const char*, BMI088::AccelRange> accelScaleTable[] = {
        {"24", BMI088::AccelRange::G_24},
        {"12", BMI088::AccelRange::G_12},
        {"6",  BMI088::AccelRange::G_6},
        {"3",  BMI088::AccelRange::G_3}
    };

    static constexpr std::pair<const char*, BMI088::AccelOversampling> accelOSRTable[] = {
        {"Normal", BMI088::AccelOversampling::Normal},
        {"OSR2",   BMI088::AccelOversampling::OSR2},
        {"OSR4",   BMI088::AccelOversampling::OSR4}
    };

    struct AccelOdrConfig {
        BMI088::AccelODR odr;
        int timer;
    };

    static constexpr std::pair<const char*, AccelOdrConfig> accelOdrTable[] = {
        {"1600", {BMI088::AccelODR::ODR_1600Hz, 625}},
        {"800",  {BMI088::AccelODR::ODR_800Hz, 1250}},
        {"400",  {BMI088::AccelODR::ODR_400Hz, 2500}},
        {"200",  {BMI088::AccelODR::ODR_200Hz, 5000}},
        {"100",  {BMI088::AccelODR::ODR_100Hz, 10000}}
    };

    // Gyro
    static constexpr std::pair<const char*, BMI088::GyroRange> gyroScaleTable[] = {
        {"2000", BMI088::GyroRange::DPS_2000},
        {"1000", BMI088::GyroRange::DPS_1000},
        {"500",  BMI088::GyroRange::DPS_500},
        {"250",  BMI088::GyroRange::DPS_250},
        {"125",  BMI088::GyroRange::DPS_125}
    };

    struct GyroOdrConfig {
        BMI088::GyroBandwidth odr;
        int timer;
    };

    static constexpr std::pair<const char*, GyroOdrConfig> gyroOdrTable[] = {
        {"2000_532", {BMI088::GyroBandwidth::ODR_2000Hz_BW_532Hz, 500}},
        {"2000_230", {BMI088::GyroBandwidth::ODR_2000Hz_BW_230Hz, 500}},
        {"1000_116", {BMI088::GyroBandwidth::ODR_1000Hz_BW_116Hz, 1000}},
        {"400_47",   {BMI088::GyroBandwidth::ODR_400Hz_BW_47Hz, 2500}},
        {"200_23",   {BMI088::GyroBandwidth::ODR_200Hz_BW_23Hz, 5000}},
        {"100_12",   {BMI088::GyroBandwidth::ODR_100Hz_BW_12Hz, 10000}},
        {"200_64",   {BMI088::GyroBandwidth::ODR_200Hz_BW_64Hz, 5000}},
        {"100_32",   {BMI088::GyroBandwidth::ODR_100Hz_BW_32Hz, 10000}}
    };

    // DPS368
    struct DPSOdrConfig {
        DPS368::MeasurementRate odr;
        int timer;
    };

    static constexpr std::pair<const char*, DPSOdrConfig> dpsOdrTable[] = {
        {"1",   {DPS368::MeasurementRate::Hz1, 1000000}},
        {"2",   {DPS368::MeasurementRate::Hz2, 500000}},
        {"4",   {DPS368::MeasurementRate::Hz4, 250000}},
        {"8",   {DPS368::MeasurementRate::Hz8, 125000}},
        {"16",  {DPS368::MeasurementRate::Hz16, 62500}},
        {"32",  {DPS368::MeasurementRate::Hz32, 31250}},
        {"64",  {DPS368::MeasurementRate::Hz64, 15625}},
        {"128", {DPS368::MeasurementRate::Hz128, 7813}}
    };

    static constexpr std::pair<const char*, DPS368::OversamplingRate> dpsOSRTable[] = {
        {"1",   DPS368::OversamplingRate::OSR1},
        {"2",   DPS368::OversamplingRate::OSR2},
        {"4",   DPS368::OversamplingRate::OSR4},
        {"8",   DPS368::OversamplingRate::OSR8},
        {"16",  DPS368::OversamplingRate::OSR16},
        {"32",  DPS368::OversamplingRate::OSR32},
        {"64",  DPS368::OversamplingRate::OSR64},
        {"128", DPS368::OversamplingRate::OSR128},
    };

    // LIS2MDL
    struct MagOdrConfig {
        LIS2MDL::ODR odr;
        int timer;
    };

    static constexpr std::pair<const char*, MagOdrConfig> magOdrTable[] = {
        {"10",  {LIS2MDL::ODR::ODR_10HZ, 100000}},
        {"20",  {LIS2MDL::ODR::ODR_20HZ, 50000}},
        {"50",  {LIS2MDL::ODR::ODR_50HZ, 20000}},
        {"100", {LIS2MDL::ODR::ODR_100HZ, 10000}}

    };

    template <typename T, size_t N>
    static constexpr const T* findEntry(const std::pair<const char*, T> (&table)[N], const std::string& key) {
        for (size_t i = 0; i < N; ++i) {
            if (key == table[i].first) {
                std::cout << "key found " << table[i].first << "\n";
                return &table[i].second;
            }
        }
        return nullptr;
    }
};