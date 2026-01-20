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
    double x, y, z;
    uint64_t timestamp_us = 0;
};

struct GyroData {
    double x, y, z;
    uint64_t timestamp_us = 0;
};

struct MagData {
    double x, y, z;
    uint64_t timestamp_us = 0;
};

struct BaroData {
    double pressure_Pa;
    double temperature_C;
    double altitude_m;         // relative to reference
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
        LISxMDL::ODR mag_odr = LISxMDL::ODR::Hz_80) {

            dps310_ = std::make_unique<DPS310>(pressureRate, pressureOversampling, tempRate, tempOversampling);
            bmi088_ = std::make_unique<BMI088>(accel_scale, accel_odr, BMI088::AccelOversampling::Normal, 
                                                gyro_range, gyro_bw);
            mag_ = std::make_unique<LISxMDL>(mag_scale, mag_odr);

            bmi_accel_timer = std::make_unique<Timer>(std::chrono::microseconds(1250));
            bmi_gyro_timer = std::make_unique<Timer>(std::chrono::microseconds(1000));
            mag_timer = std::make_unique<Timer>(std::chrono::microseconds(31500));
            dps_timer = std::make_unique<Timer>(std::chrono::milliseconds(12));

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
            if (!has_new_baro_.exchange(false, std::memory_order_acq_rel)) {
                return false;
            }
            const auto* latest = baro_buffer_.get_latest();
            if (latest) {
                out = *latest;
                return true;
            }
            return false;
        }

        // Starts thread to read all IMU sensors (BMI088, DPS310, LISxMDL)
        void start_sensor_thread() {
            running = true;
            std::cout << "start_sensor_thread() called → launching thread\n" << std::flush;
        
            try {
                sensor_update_thread = std::thread([this] {
                    std::cout << "Thread is now running!\n" << std::flush;
                    update10DOF();
                });
            } catch (const std::exception& e) {
                std::cerr << "Failed to create thread: " << e.what() << "\n";
                running = false;
            }
        }
        void stop_sensor_thread() {
            if (!running) {
                return;
            }

            running = false;

            if (sensor_update_thread.joinable()) {
                sensor_update_thread.join();
            }

        }

        ~IMU(){

            dps310_.reset();
            bmi088_.reset();
            mag_.reset();
            bmi_accel_timer->stop();
            bmi_gyro_timer->stop();
            stop_sensor_thread();

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

        DataBuffer<AccelData, 3> accel_buffer_;
        DataBuffer<GyroData, 3>  gyro_buffer_;
        DataBuffer<MagData, 3>   mag_buffer_;
        DataBuffer<BaroData, 3>  baro_buffer_;

        std::unique_ptr<DPS310> dps310_;
        std::unique_ptr<BMI088> bmi088_;
        std::unique_ptr<LISxMDL> mag_;

        std::unique_ptr<Timer> bmi_gyro_timer;
        std::unique_ptr<Timer> bmi_accel_timer;
        std::unique_ptr<Timer> mag_timer;
        std::unique_ptr<Timer> dps_timer;

        std::atomic<bool> has_new_accel_{false};
        std::atomic<bool> has_new_gyro_{false};
        std::atomic<bool> has_new_mag_{false};
        std::atomic<bool> has_new_baro_{false};

        // dps310 values
        bool ispressureRDY = false;
        bool istempRDY = false;
        double pressure;
        double temp;
        double altitude;
        double altitude_change = 0.0;
        double altitude0 = 0.0;

        // bmi088 values
        std::atomic<bool> isgyroRDY = false;
        std::atomic<bool> isaccelRDY = false;
        
        // sensor readings
        std::atomic<bool> running = false;
        std::thread sensor_update_thread;
};