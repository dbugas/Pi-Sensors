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
        IMU( int pressureRate = 32, int pressureOversampling = 16,
        int tempRate = 1, int tempOversampling = 1,
        int accel_scale = 6, double accel_odr = 800.0,
        BMI088::GyroRange gyro_range =  BMI088::GyroRange::DPS_1000, 
        BMI088::GyroBandwidth gyro_bw = BMI088::GyroBandwidth::ODR_1000Hz_BW_116Hz,
        LISxMDL::FullScale mag_scale = LISxMDL::FullScale::Gauss_8,
        LISxMDL::ODR mag_odr = LISxMDL::ODR::Hz_80) : vqf(1e-3, 1.25e-3, 0.0125) {

            dps310_ = std::make_unique<DPS310>(pressureRate, pressureOversampling, tempRate, tempOversampling);
            bmi088_ = std::make_unique<BMI088>(accel_scale, accel_odr, BMI088::AccelOversampling::Normal, 
                                                gyro_range, gyro_bw);
            mag_ = std::make_unique<LISxMDL>(mag_scale, mag_odr);

            bmi_accel_timer = std::make_unique<Timer>(std::chrono::microseconds(1250));
            bmi_gyro_timer = std::make_unique<Timer>(std::chrono::microseconds(1000));
            mag_timer = std::make_unique<Timer>(std::chrono::microseconds(31500));
            dps_timer = std::make_unique<Timer>(std::chrono::milliseconds(12));
            quat_timer = std::make_unique<Timer>(std::chrono::milliseconds(5));

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

            bmi_accel_timer->start();
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            bmi_gyro_timer->start();
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            mag_timer->start();
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
            } catch (const std::exception& e) {
                std::cerr << "Failed to create start_sensor_thread: " << e.what() << "\n";
                running = false;
            }
        }
        void stop_sensor_threads() {
            if (!running) {
                return;
            }

            running = false;

            if (sensor_update_thread.joinable()) {
                sensor_update_thread.join();
            }
            if(quat_update_thread.joinable()){
                quat_update_thread.join();
            }

        }

        void update_quat_thread(bool use_mag){

            try {
                quat_update_thread = std::thread([this, use_mag] {
                    while(running){

                        update_ori(use_mag);
                        std::this_thread::sleep_for(std::chrono::microseconds(100));
                    }
                });
            }
            catch (const std::exception& e) {
                std::cerr << "Failed to create update_quat_thread: " << e.what() << "\n";
            }
        }

        // Uses vqf to update quaternion with gyroscope, accelerometer, and optionally magnetometer data
        void update_ori(bool use_mag){

            if(get_latest_gyro_and_consume(gydat)){
                vqf_real_t gyro[3] = {gydat.x, gydat.y, gydat.z};
                vqf.updateGyr(gyro);
            }
            if(get_latest_accel_and_consume(accdat)){
                vqf_real_t accel[3] = {accdat.x, accdat.y, accdat.z};
                vqf.updateAcc(accel);
            }
            if(use_mag){
                if(get_latest_mag_and_consume(magdat)){
                    vqf_real_t mag[3] = {magdat.x, magdat.y, magdat.z};
                    vqf.updateMag(mag);
                }
            }
            if(!quat_timer->check()) return;

            if(!use_mag){
                QuatData* slot = quat_buffer_.prepare_write();
                vqf.getQuat6D(slot->q);
                slot->timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                
                quat_buffer_.commit();
                has_new_quat_.store(true, std::memory_order_release);
            }
            else{
                QuatData* slot = quat_buffer_.prepare_write();
                vqf.getQuat9D(slot->q);
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
                update_mag();
                update_baro();

                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }

        VQF vqf;

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