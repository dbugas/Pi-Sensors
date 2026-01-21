#pragma once

#include <memory>
#include <chrono>
#include <thread>
#include <iostream>
#include <atomic>

#include "BMI088.h"
#include "dps310.h"
#include "LISxMDL.h"
#include "vqf.h"
#include "timer.h"
#include "DataBuffer.h"

struct AccelData {
    double x, y, z;             // m/s
    uint64_t timestamp_us = 0;
};

struct GyroData {
    double x, y, z;             // rad/s
    uint64_t timestamp_us = 0;
};

struct MagData {
    double x, y, z;             // Calibrated unitless, norm ~1
    uint64_t timestamp_us = 0;
};

struct BaroData {
    double pressure_Pa;
    double temperature_C;
    double altitude_m;         // relative to reference
    uint64_t timestamp_us = 0;
};
struct QuatData {
    double q[4];              // quaternion [w, x, y, z]
    uint64_t timestamp_us = 0;
};

class IMU{
    public:
        // Performance mode controls sensor data rate, so sensor accuracy is the inverse.
        enum class PerformanceMode {
            Ultra,      // Fastest response (~0.3–1 ms effective rise time / very high bandwidth). For ultra-agile dynamics & vibration-heavy environments.
            High,       // Fast response (~1–3 ms effective rise time / high bandwidth). Standard for demanding real-time tracking.
            Medium,     // Moderate response (~5–10 ms effective rise time / medium bandwidth). Good compromise for general-purpose use.
            Low         // Slow response (~20–50 ms effective rise time / low bandwidth). Best for low-noise, low-power, slow-changing applications.
        };

        IMU(PerformanceMode mode, bool use_mag, bool use_barometer){

            int pressureRate;
            int pressureOversampling;
            int tempRate;
            int tempOversampling;
            BMI088::AccelRange accel_scale;
            BMI088::AccelODR accel_odr;
            BMI088::AccelOversampling accel_osr;
            BMI088::GyroRange gyro_range;
            BMI088::GyroBandwidth gyro_bw;
            LISxMDL::FullScale mag_scale;
            LISxMDL::ODR mag_odr;

            // in microseconds
            int bmi_accel_timer_val;
            int bmi_gyro_timer_val;
            int mag_timer_val;      
            int dps_timer_val;      
            int quat_timer_val;     

            switch(mode){
                case PerformanceMode::Ultra:
                    accel_scale = BMI088::AccelRange::G_24;   
                    accel_odr   = BMI088::AccelODR::ODR_1600Hz;
                    accel_osr   = BMI088::AccelOversampling::Normal;
                    gyro_range  = BMI088::GyroRange::DPS_2000;
                    gyro_bw     = BMI088::GyroBandwidth::ODR_2000Hz_BW_230Hz;

                    pressureRate         = 64;
                    pressureOversampling = 8;
                    tempRate             = 1;
                    tempOversampling     = 1;

                    mag_scale = LISxMDL::FullScale::Gauss_12;
                    mag_odr   = LISxMDL::ODR::Hz_80;

                    bmi_accel_timer_val = 625;
                    bmi_gyro_timer_val  = 500;
                    mag_timer_val       = 12500;
                    dps_timer_val       = 15625;
                    quat_timer_val      = 625;
                    break;

                case PerformanceMode::High:
                    accel_scale = BMI088::AccelRange::G_6;   
                    accel_odr   = BMI088::AccelODR::ODR_800Hz;
                    accel_osr   = BMI088::AccelOversampling::Normal;
                    gyro_range  = BMI088::GyroRange::DPS_1000;
                    gyro_bw     = BMI088::GyroBandwidth::ODR_1000Hz_BW_116Hz;

                    pressureRate         = 32;
                    pressureOversampling = 16;
                    tempRate             = 1;
                    tempOversampling     = 1;

                    mag_scale = LISxMDL::FullScale::Gauss_8;
                    mag_odr   = LISxMDL::ODR::Hz_80;

                    bmi_accel_timer_val = 1250;
                    bmi_gyro_timer_val  = 1000;
                    mag_timer_val       = 12500;
                    dps_timer_val       = 32000;
                    quat_timer_val      = 1000;
                    break;

                case PerformanceMode::Medium:
                    accel_scale = BMI088::AccelRange::G_6;   
                    accel_odr   = BMI088::AccelODR::ODR_400Hz;
                    accel_osr   = BMI088::AccelOversampling::Normal;
                    gyro_range  = BMI088::GyroRange::DPS_1000;
                    gyro_bw     = BMI088::GyroBandwidth::ODR_400Hz_BW_47Hz;

                    pressureRate         = 32;
                    pressureOversampling = 16;
                    tempRate             = 1;
                    tempOversampling     = 1;

                    mag_scale = LISxMDL::FullScale::Gauss_4;
                    mag_odr   = LISxMDL::ODR::Hz_40;

                    bmi_accel_timer_val = 2500;
                    bmi_gyro_timer_val  = 2500;
                    mag_timer_val       = 25000;
                    dps_timer_val       = 32000;
                    quat_timer_val      = 2500;
                    break;

                case PerformanceMode::Low:
                    accel_scale = BMI088::AccelRange::G_3;   
                    accel_odr   = BMI088::AccelODR::ODR_100Hz;
                    accel_osr   = BMI088::AccelOversampling::OSR2;
                    gyro_range  = BMI088::GyroRange::DPS_250;
                    gyro_bw     = BMI088::GyroBandwidth::ODR_200Hz_BW_64Hz;

                    pressureRate         = 8;
                    pressureOversampling = 32;
                    tempRate             = 1;
                    tempOversampling     = 1;

                    mag_scale = LISxMDL::FullScale::Gauss_4;
                    mag_odr   = LISxMDL::ODR::Hz_20;

                    bmi_accel_timer_val = 10000;
                    bmi_gyro_timer_val  = 5000;
                    mag_timer_val       = 50000;
                    dps_timer_val       = 1.25e5;
                    quat_timer_val      = 5000;
                    break;
            }

            bmi088_ = std::make_unique<BMI088>(accel_scale, accel_odr, accel_osr, gyro_range, gyro_bw);
            if(use_barometer) dps310_ = std::make_unique<DPS310>(pressureRate, pressureOversampling, tempRate, tempOversampling);
            if(use_mag)       mag_    = std::make_unique<LISxMDL>(mag_scale, mag_odr);

            vqf = std::make_unique<VQF>((double)bmi_gyro_timer_val*1e-6, 
                                        (double)bmi_accel_timer_val*1e-6, 
                                        (double)mag_timer_val*1e-6);
            bmi_accel_timer             = std::make_unique<Timer>(std::chrono::microseconds(bmi_accel_timer_val));
            bmi_gyro_timer              = std::make_unique<Timer>(std::chrono::microseconds(bmi_gyro_timer_val));
            quat_timer                  = std::make_unique<Timer>(std::chrono::microseconds(quat_timer_val));
            if(use_mag) mag_timer       = std::make_unique<Timer>(std::chrono::microseconds(mag_timer_val-100));
            if(use_barometer) {
                dps_timer = std::make_unique<Timer>(std::chrono::microseconds(dps_timer_val-100));

                int counter = 0;
                double avg = 0.0;

                dps_timer->start();
                while(counter < 10){
                   if(update_baro()) {
                       counter ++;
                       const BaroData* data = latest_baro();
                       avg += data->altitude_m;
                   }
                    std::this_thread::sleep_for(std::chrono::microseconds(10));
                }
                altitude0 = avg /10.0;
            }

            bmi_accel_timer->start();
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            bmi_gyro_timer->start();
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            if(use_mag) mag_timer->start();
            quat_timer->start();
        }

        bool update_accel() {
            if (!bmi_accel_timer->check()) return false;

            AccelData* slot = accel_buffer_.prepare_write();

            if (bmi088_->readAccel(slot->x, slot->y, slot->z)) {
                slot->timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();

                accel_buffer_.commit();
                has_new_accel_.store(true, std::memory_order_release);

                return true;
            }
            return false;
        }

        bool update_gyro() {
            if (!bmi_gyro_timer->check()) return false;

            GyroData* slot = gyro_buffer_.prepare_write();

            if (bmi088_->readGyro(slot->x, slot->y, slot->z)) {
                slot->timestamp_us =  std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();

                gyro_buffer_.commit();
                has_new_gyro_.store(true, std::memory_order_release);
                return true;
            }
            return false;
        }

        bool update_mag() {
            if(!mag_timer->check()) return false;

            if (mag_->dataReady()){
                MagData* slot = mag_buffer_.prepare_write();
                if (mag_->readCalibrated(slot->x, slot->y, slot->z)) {
                    slot->timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();

                    mag_buffer_.commit();
                    has_new_mag_.store(true, std::memory_order_release);
                    return true;
                }
            }
            else mag_timer->set(true);
            
            return false;
        }
        bool update_baro() {
            if(!dps_timer->check()) return false;

            if (dps310_->isMeasurementReady(ispressureRDY, istempRDY)){

                BaroData* slot = baro_buffer_.prepare_write();

                bool ok = dps310_->readData(slot->pressure_Pa, slot->temperature_C,
                                            istempRDY, ispressureRDY);

                if (ok) {
                    dps310_->calculateAltitudeChange(slot->pressure_Pa, slot->temperature_C,
                                                    slot->altitude_m, altitude_change);
                    slot->altitude_m -= altitude0;  // relative altitude
                    slot->timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();

                    baro_buffer_.commit();
                    has_new_baro_.store(true, std::memory_order_release);
                    return true;
                }
            }
            else dps_timer->set(true);
 
            return false;
        }

        // ── Reader interface ────────────────
        // All these are lock-free and very fast

        const AccelData* latest_accel() const noexcept {

            return accel_buffer_.get_latest();
        }

        const GyroData* latest_gyro() const noexcept {
            return gyro_buffer_.get_latest();
        }

        const MagData* latest_mag() const noexcept {
            return mag_buffer_.get_latest();
        }
        const BaroData* latest_baro() const noexcept {
            return baro_buffer_.get_latest();
        }
        bool get_latest_accel_and_consume(AccelData& out) noexcept {
            if (!has_new_accel_.exchange(false, std::memory_order_acq_rel)) {
                return false;  // no new data
            }

            const auto* latest = accel_buffer_.get_latest();
            if (latest) {
                out = *latest;  // copy the data
                return true;
            }
            return false;
        }

        bool get_latest_gyro_and_consume(GyroData& out) noexcept {
            if (!has_new_gyro_.exchange(false, std::memory_order_acq_rel)) {
                return false;
            }
            const auto* latest = gyro_buffer_.get_latest();
            if (latest) {
                out = *latest;
                return true;
            }
            return false;
        }
        
        bool get_latest_mag_and_consume(MagData& out) noexcept {
            if (!has_new_mag_.exchange(false, std::memory_order_acq_rel)) {
                return false;
            }
            const auto* latest = mag_buffer_.get_latest();
            if (latest) {
                out = *latest;
                return true;
            }
            return false;
        }

        bool get_latest_baro_and_consume(BaroData& out) noexcept {
            if(!has_new_baro_.exchange(false, std::memory_order_acq_rel)) {
                return false;
            }
            const auto* latest = baro_buffer_.get_latest();
            if(latest) {
                out = *latest;
                return true;
            }
            return false;
        }
        bool get_latest_quat_and_consume(QuatData& out) noexcept {
            if(!has_new_quat_.exchange(false, std::memory_order_acq_rel)){
                return false;
            }
            const auto* latest = quat_buffer_.get_latest();
            if(latest){
                out = *latest;
                return true;
            }
            return false;
        }

        // Starts thread to read all IMU sensors (BMI088, DPS310, LISxMDL)
        void start_sensor_thread() {
            running = true;
        
            try {
                sensor_update_thread = std::thread([this] {
                    update10DOF();
                });
            } catch(const std::exception& e) {
                std::cerr << "Failed to create start_sensor_thread: " << e.what() << "\n";
                running = false;
            }
        }
        void stop_sensor_threads() {
            if(!running) {
                return;
            }

            running = false;

            if(sensor_update_thread.joinable()) {
                sensor_update_thread.join();
            }
            if(quat_update_thread.joinable()){
                quat_update_thread.join();
            }

        }

        void update_quat_thread(){

            try{
                quat_update_thread = std::thread([this] {
                    while(running){

                        update_ori();
                        std::this_thread::sleep_for(std::chrono::microseconds(50));
                    }
                });
            }
            catch(const std::exception& e) {
                std::cerr << "Failed to create update_quat_thread: " << e.what() << "\n";
            }
        }

        // Uses vqf to update quaternion with gyroscope, accelerometer, and optionally magnetometer data
        void update_ori(){

            if(get_latest_gyro_and_consume(gydat)){
                vqf_real_t gyro[3] = {gydat.x, gydat.y, gydat.z};
                vqf->updateGyr(gyro);
            }
            if(get_latest_accel_and_consume(accdat)){
                vqf_real_t accel[3] = {accdat.x, accdat.y, accdat.z};
                vqf->updateAcc(accel);
            }
            if(mag_ != nullptr){
                if(get_latest_mag_and_consume(magdat)){
                    vqf_real_t mag[3] = {magdat.x, magdat.y, magdat.z};
                    vqf->updateMag(mag);
                }
            }
            if(!quat_timer->check()) return;

            if(mag_ != nullptr){
                QuatData* slot = quat_buffer_.prepare_write();
                vqf->getQuat9D(slot->q);
                slot->timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                
                quat_buffer_.commit();
                has_new_quat_.store(true, std::memory_order_release);
            }
            else{
                QuatData* slot = quat_buffer_.prepare_write();
                vqf->getQuat6D(slot->q);
                slot->timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                
                quat_buffer_.commit();
                has_new_quat_.store(true, std::memory_order_release);
            }
        }

        ~IMU(){

            stop_sensor_threads();
            dps310_.reset();
            bmi088_.reset();
            mag_.reset();
            vqf.reset();
            bmi_accel_timer->stop();
            bmi_gyro_timer->stop();
            mag_timer->stop();
            dps_timer->stop();
            quat_timer->stop();

        }
    private:

        //  read all sensors in the imu until running = false
        void update10DOF(){
            while(running){

                update_gyro();
                update_accel();
                if(mag_ != nullptr) update_mag();
                if(dps310_ != nullptr) update_baro();

                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }
        }

        std::unique_ptr<VQF> vqf;

        DataBuffer<AccelData, 3> accel_buffer_;
        DataBuffer<GyroData, 3>  gyro_buffer_;
        DataBuffer<MagData, 3>   mag_buffer_;
        DataBuffer<BaroData, 3>  baro_buffer_;
        DataBuffer<QuatData, 3>  quat_buffer_;

        AccelData accdat;
        GyroData gydat;
        MagData magdat;

        std::unique_ptr<DPS310> dps310_;
        std::unique_ptr<BMI088> bmi088_;
        std::unique_ptr<LISxMDL> mag_;

        std::unique_ptr<Timer> bmi_gyro_timer;
        std::unique_ptr<Timer> bmi_accel_timer;
        std::unique_ptr<Timer> mag_timer;
        std::unique_ptr<Timer> dps_timer;
        std::unique_ptr<Timer> quat_timer;

        std::atomic<bool> has_new_accel_{false};
        std::atomic<bool> has_new_gyro_{false};
        std::atomic<bool> has_new_mag_{false};
        std::atomic<bool> has_new_baro_{false};
        std::atomic<bool> has_new_quat_{false};

        // dps310 values
        bool ispressureRDY = false;
        bool istempRDY = false;
        double altitude_change = 0.0;
        double altitude0 = 0.0;

        // bmi088 values
        std::atomic<bool> isgyroRDY = false;
        std::atomic<bool> isaccelRDY = false;
        
        // sensor readings
        std::atomic<bool> running = false;
        std::thread sensor_update_thread;
        std::thread quat_update_thread;
};