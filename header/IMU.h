#pragma once

#include <memory>
#include <chrono>
#include <thread>
#include <iostream>
#include <atomic>

#include "BMI088.h"
#include "dps310.h"
#include "LISxMDL.h"
#include "PCA9685.h"
#include "ads1115.h"
#include "vqf.h"
#include "timer.h"
#include "DataBuffer.h"
#include "gpio.h"
#include "srKF.h"

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
    double vertical_speed_mps;
    uint64_t timestamp_us = 0;
};
struct QuatData {
    double q[4];               // quaternion [w, x, y, z]
    uint64_t timestamp_us = 0;
};

class IMU : protected gpio {
    public:
    enum class PerformanceMode {
        Ultra,      // Highest ODR + widest bandwidth (~200–500 Hz cutoff). Very low latency, captures fastest dynamics & vibrations, higher noise & power.
        High,       // High ODR + wide bandwidth (~100–230 Hz cutoff). Excellent for most dynamic applications (drones, robotics).
        Medium,     // Moderate ODR + bandwidth (~40–145 Hz cutoff). Balanced response for general motion, lower noise & CPU load.
        Low         // Lowest ODR + narrowest bandwidth (~10–40 Hz cutoff) Lowest noise & power, suited for slow/static or battery-critical apps.
    };

        IMU(PerformanceMode mode, bool use_mag, bool use_barometer){

            init_gpio();
            int pressureRate = 16;
            int pressureOversampling = 16;
            int tempRate = 1;
            int tempOversampling = 1;
            BMI088::AccelRange accel_scale = BMI088::AccelRange::G_12;
            BMI088::AccelODR accel_odr = BMI088::AccelODR::ODR_800Hz;
            BMI088::AccelOversampling accel_osr = BMI088::AccelOversampling::Normal;
            BMI088::GyroRange gyro_range = BMI088::GyroRange::DPS_1000;
            BMI088::GyroBandwidth gyro_bw = BMI088::GyroBandwidth::ODR_1000Hz_BW_116Hz;
            LISxMDL::FullScale mag_scale  =LISxMDL::FullScale::Gauss_8;
            LISxMDL::ODR mag_odr = LISxMDL::ODR::Hz_80;

            // in microseconds
            int bmi_accel_timer_val = 1250;
            int bmi_gyro_timer_val  = 1000;
            int mag_timer_val       = 12500;
            int dps_timer_val       = 32000;
            int quat_timer_val      = 1000;    
            delay = 50;

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

                    mag_scale = LISxMDL::FullScale::Gauss_8;
                    mag_odr   = LISxMDL::ODR::Hz_155;

                    bmi_accel_timer_val = 625;
                    bmi_gyro_timer_val  = 500;
                    mag_timer_val       = 6451;
                    dps_timer_val       = 14100;
                    quat_timer_val      = 1000;
                    delay = 50;
                    break;

                case PerformanceMode::High:
                    accel_scale = BMI088::AccelRange::G_12;   
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
                    dps_timer_val       = 27600;
                    quat_timer_val      = 1000;
                    delay = 50;
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
                    dps_timer_val       = 27600;
                    quat_timer_val      = 2500;
                    delay = 250;
                    break;

                case PerformanceMode::Low:
                    accel_scale = BMI088::AccelRange::G_3;   
                    accel_odr   = BMI088::AccelODR::ODR_100Hz;
                    accel_osr   = BMI088::AccelOversampling::OSR2;
                    gyro_range  = BMI088::GyroRange::DPS_250;
                    gyro_bw     = BMI088::GyroBandwidth::ODR_200Hz_BW_64Hz;

                    pressureRate         = 8;
                    pressureOversampling = 64;
                    tempRate             = 1;
                    tempOversampling     = 1;

                    mag_scale = LISxMDL::FullScale::Gauss_4;
                    mag_odr   = LISxMDL::ODR::Hz_20;

                    bmi_accel_timer_val = 10000;
                    bmi_gyro_timer_val  = 5000;
                    mag_timer_val       = 50000;
                    dps_timer_val       = 1011000;
                    quat_timer_val      = 5000;
                    delay = 500;
                    break;
            }

            bmi088_ = std::make_unique<BMI088>(accel_scale, accel_odr, accel_osr, gyro_range, gyro_bw);
            if(use_barometer) dps310_ = std::make_unique<DPS310>(pressureRate, pressureOversampling, tempRate, tempOversampling);
            if(use_mag)       mag_    = std::make_unique<LISxMDL>(mag_scale, mag_odr);

            vqf = std::make_unique<VQF>((double)bmi_gyro_timer_val*1e-6, 
                                        (double)bmi_accel_timer_val*1e-6, 
                                        (double)mag_timer_val*1e-6);
            bmi_accel_timer       = std::make_unique<Timer>(std::chrono::microseconds(bmi_accel_timer_val));
            bmi_gyro_timer        = std::make_unique<Timer>(std::chrono::microseconds(bmi_gyro_timer_val));
            quat_timer            = std::make_unique<Timer>(std::chrono::microseconds(quat_timer_val));
            if(use_mag) mag_timer = std::make_unique<Timer>(std::chrono::microseconds(mag_timer_val));
            if(use_barometer) {
                dps_timer = std::make_unique<Timer>(std::chrono::microseconds(dps_timer_val));

                int counter = 0;
                int timeout = 0;
                double avg = 0.0;

                dps_timer->start(true);
                while(counter < 10){
                    if(!dps_timer->check()) {
                        timeout += 5000;
                        continue;
                    }
                    if (dps310_->isMeasurementReady(ispressureRDY, istempRDY)){
                            double pressure_Pa = 0.0;
                            double temperature_C = 0.0;
                            double altitude_m = 0.0;
                            bool ok = dps310_->readData(pressure_Pa, temperature_C,
                                                        istempRDY, ispressureRDY);
                            if(ok){
                                dps310_->calculateAltitudeChange(pressure_Pa, temperature_C,
                                                    altitude_m, altitude_change);
                                avg += altitude_m;
                                counter++;
                            }
                    }
                    else dps_timer->set(true);
                    
                    if(timeout > dps_timer_val*2000000){
                        std::cout << "DPS310 setup failed \n";
                        dps_timer->stop();
                        dps310_.reset();
                        break;
                    }
                    
                    std::this_thread::sleep_for(std::chrono::microseconds(5000));
                }
                altitude0 = avg /10.0;

                EKF = std::make_unique<srKF>(
                    Eigen::Matrix4d{{1,0,0,0}, {0,0.5,0,0}, {0,0,0.1,0}, {0,0,0,0.05}},
                    Eigen::Matrix<double,1,1>{{10.0}},
                    compute_process_noise_approx((double)bmi_accel_timer_val*1e-6, 0.01, 0.0001),
                    Eigen::Vector4d{altitude0,0,0,0.01},
                    (double)bmi_accel_timer_val*1e-6
                );

            }

            bmi_accel_timer->start(true);
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            bmi_gyro_timer->start(true);
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            if(use_mag) mag_timer->start(true);
            quat_timer->start(true);
        }

        bool update_accel_raw() {
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

        bool update_accel() {
            if (!bmi_accel_timer->check()) return false;

            AccelData* slot = accel_buffer_.prepare_write();

            if (bmi088_->readAccelCalibrated(slot->x, slot->y, slot->z)) {
                slot->timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();

                accel_buffer_.commit();
                has_new_accel_.store(true, std::memory_order_release);
                
                if(dps310_){
                    // Update EKF with new accel data
                    const QuatData* quat = latest_quat();
                    Eigen::Quaterniond q(quat->q[0], quat->q[1], quat->q[2], quat->q[3]);
                    Eigen::Vector3d a(slot->x, slot->y, slot->z);
                    a = q * a; 
                    EKF->KF_predict_alt((a(2) - 1.0)*9.80665); // use Z linear accel in world frame

                    BaroData* baro_slot = baro_buffer_.prepare_write();
                    baro_slot->altitude_m = EKF->xhat(0) - altitude0;  
                    baro_slot->vertical_speed_mps = EKF->xhat(1);
                    baro_buffer_.commit();
                    has_new_baro_.store(true, std::memory_order_release);
                }
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

        bool update_mag_raw() {
            if(!mag_timer->check()) return false;

            if (mag_->dataReady()){
                MagData* slot = mag_buffer_.prepare_write();
                if (mag_->read_gauss(slot->x, slot->y, slot->z)) {
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
                    EKF->KF_update_alt(Eigen::VectorXd::Constant(1, slot->pressure_Pa), slot->temperature_C + 273.15);
                    EKF->SageHusa_update_alt();
                    //dps310_->calculateAltitudeChange(slot->pressure_Pa, slot->temperature_C,
                    //                                slot->altitude_m, altitude_change);
                    //std::cout << "altitude EKF: " << EKF->xhat(0) << " m" << " Vertical Vel: " << EKF->xhat(1) << " m/s" << " accel: " << EKF->xhat(2) << " m/s²" << " Accel Bias: " << EKF->xhat(3) << " m/s²\n";
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
        const QuatData* latest_quat() const noexcept {
            return quat_buffer_.get_latest();
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

        void update_quat_thread() {
            try {
                quat_update_thread = std::thread([this] {
                    // Set real-time priority for this thread specifically

                    #ifdef __linux__
                        pin_thread_to_core(3);
                        sched_param param{};
                        param.sched_priority = 99; 
                        pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
                    #endif
                
                    //bool wait_type = delay <= 100;
                    //const auto interval = std::chrono::microseconds(delay);
                    //auto next_deadline = std::chrono::steady_clock::now() + interval;
                
                    while (running) {
                        update_quat();
                        if(vqf->getRestDetected()){
                            altitude0 = EKF->xhat(0);
                        }
                        //if(wait_type) hybrid_wait(next_deadline, interval);
                        std::this_thread::sleep_for(std::chrono::microseconds(delay));
                    }
                });
            } catch (const std::exception& e) {
                std::cerr << "Failed to create update_quat_thread: " << e.what() << "\n";
            }
        }

        // Uses vqf to update quaternion with gyroscope, accelerometer, and optionally magnetometer data
        void update_quat(){

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
            else { 
                QuatData* slot = quat_buffer_.prepare_write();
                vqf->getQuat6D(slot->q);
                slot->timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
                
                quat_buffer_.commit();
                has_new_quat_.store(true, std::memory_order_release);
            }
        }

        void update_imu(){

            update_gyro();
            update_accel();
            if (mag_ != nullptr) update_mag();
            if (dps310_ != nullptr) update_baro();
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

            close_gpio();
        }
    private:

    void pin_thread_to_core(int core_id) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(core_id, &cpuset);

        pthread_t current_thread = pthread_self();
        if (pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset) != 0) {
            std::cerr << "Error pinning thread to core " << core_id << "\n";
        }
    }

    void hybrid_wait(std::chrono::steady_clock::time_point& next_deadline, 
                     std::chrono::microseconds interval) {

        // Spin Phase
        while (std::chrono::steady_clock::now() < next_deadline) {
            asm volatile("yield");
        }

        // Update deadline
        next_deadline += interval;

        auto post_now = std::chrono::steady_clock::now();
        if (post_now > next_deadline + std::chrono::microseconds(50)) {
            next_deadline = post_now + interval;
        }
    }

        //  read all sensors in the imu until running = false
    void update10DOF() {
        // interval calculated from your microsecond delay
        //const auto interval = std::chrono::microseconds(delay);
        //auto next_deadline = std::chrono::steady_clock::now() + interval;

        #ifdef __linux__
            pin_thread_to_core(2);
            sched_param param{};
            param.sched_priority = 99; 
            pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        #endif

        //bool wait_type = delay <= 100;
        while (running) {
            update_imu();
            //if(wait_type) hybrid_wait(next_deadline, interval);
            std::this_thread::sleep_for(std::chrono::microseconds(delay));
        }
    }

    Eigen::Matrix4d compute_process_noise_approx(double dt, double q_accel, double q_bias) {
        Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();

        if (dt <= 0.0) {
            return Q;  // avoid negative / zero dt issues
        }

        const double dt2 = dt * dt;
        const double dt3 = dt2 * dt;
        const double dt4 = dt3 * dt;
        const double dt5 = dt4 * dt;

        Q(0, 0) = (dt4 / 20.0 * q_accel + dt * q_bias)*250.0;
        Q(0, 1) = (dt3 / 8.0 * q_accel + dt * q_bias) * 10.0;
        Q(0, 2) = (dt2 / 6.0 * q_accel + dt * q_bias) * 10.0;

        Q(1, 0) = Q(0, 1);
        Q(1, 1) = (dt2 / 3.0 * q_accel + dt * q_bias) * 10.0;
        Q(1, 2) = (dt / 2.0 * q_accel + dt * q_bias) * 10.0;

        Q(2, 0) = Q(0, 2);
        Q(2, 1) = Q(1, 2);
        Q(2, 2) = dt * q_accel;

        Q(3, 3) =  q_bias;

        return Q;
    }

        std::unique_ptr<VQF> vqf;
        std::unique_ptr<srKF> EKF;

        DataBuffer<AccelData, 3> accel_buffer_;
        DataBuffer<GyroData, 3>  gyro_buffer_;
        DataBuffer<MagData, 2>   mag_buffer_;
        DataBuffer<BaroData, 2>  baro_buffer_;
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
        int delay = 50;

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