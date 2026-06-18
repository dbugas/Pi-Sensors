#pragma once

#include <memory>
#include <array>
#include <chrono>
#include <thread>
#include <iostream>
#include <atomic>
#include <string>
#include <stdexcept>

#include "BMI088.h"
#include "DPS368.h"
#include "LIS2MDL.h"
#include "PCA9685.h"
#include "vqf.h"
#include "timer.h"
#include "DataBuffer.h"
#include "gpio.h"
#include "srKF.h"
#include "Parser.h"
#include "IMUConfig.h"
#include "imu_config.h"

struct PWMval {
    std::array<uint16_t, 16> pwm_Val{};
    uint8_t start;
    uint8_t count;
};

struct AccelData {
    double x, y, z;             // m/s^2
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
        IMU(IMUConfig::PerformanceMode mode, bool use_mag, bool use_barometer){

            init_gpio();
            IMUSettings settings;
            Eigen::Matrix3d calibMatrix = Eigen::Matrix3d::Zero();
            Eigen::Vector3d Offset = Eigen::Vector3d::Zero();

            parser = std::make_unique<Parser>(SOURCE_DIR "/IMUconfig.cfg");  
            parser->loadVar("calibMatrix_acc", calibMatrix);
            parser->loadVar("offset_acc", Offset);
            
            IMUConfig config(*parser);
            settings = config.load(mode);
            delay = settings.delay_us;

            bmi088_ = std::make_unique<BMI088>(settings.accel_scale, settings.accel_odr, settings.accel_osr, 
                                                settings.gyro_scale, settings.gyro_odr, calibMatrix, Offset);

            parser->loadVar("calibMatrix_mag", calibMatrix);
            parser->loadVar("offset_mag", Offset);

            if(use_barometer) dps368_ = std::make_unique<DPS368>(settings.PressureODR, settings.PressureOSR, settings.TempODR, settings.TempOSR);
            if(use_mag) {
                mag_ = std::make_unique<LIS2MDL>(settings.magODR, calibMatrix);
                mag_->setHardIronOffsets(Offset(0),Offset(1),Offset(2));
            }

            vqf = std::make_unique<VQF>((double)settings.gyro_timer_us*1e-6, 
                                        (double)settings.accel_timer_us*1e-6, 
                                        (double)settings.mag_timer_us*1e-6);
            vqf->setTauAcc(0.4);                 

            bmi_accel_timer       = std::make_unique<Timer>(std::chrono::microseconds(settings.accel_timer_us));
            bmi_gyro_timer        = std::make_unique<Timer>(std::chrono::microseconds(settings.gyro_timer_us));
            quat_timer            = std::make_unique<Timer>(std::chrono::microseconds(settings.quat_timer_us));
            if(use_mag) mag_timer = std::make_unique<Timer>(std::chrono::microseconds(settings.mag_timer_us));
            if(use_barometer) {
                dps_timer = std::make_unique<Timer>(std::chrono::microseconds(settings.dps_timer_us));

                int counter = 0;
                int timeout = 0;
                double avg = 0.0;
                double pressure_Pa = 0.0;
                double temperature_C = 0.0;
                double altitude_m = 0.0;

                dps_timer->start(true);
                while(counter < 10){
                    if(!dps_timer->check()) continue;

                    if (dps368_->readCalibrated_ifnew(pressure_Pa, temperature_C)){
                        altitude_m = dps368_->calculateAltitude(pressure_Pa, temperature_C);
                        avg += altitude_m;
                        counter++;
                    }
                    else dps_timer->set(true); // jitter compensation
                    
                    if(timeout > 100){
                        std::cout << "DPS368 setup failed \n";
                        dps_timer->stop();
                        dps368_.reset();
                        break;
                    }
                    
                    std::this_thread::sleep_for(std::chrono::microseconds(5000));
                }
                altitude0 = avg /10.0;

                BaroData* baro_slot = baro_buffer_.prepare_write();
                baro_slot->pressure_Pa = pressure_Pa;
                baro_slot->temperature_C = temperature_C;
                baro_slot->altitude_m = altitude0;
                baro_slot->vertical_speed_mps = 0.0;
                baro_buffer_.commit();

                EKF = std::make_unique<srKF>(
                    Eigen::Matrix4d{{1,0,0,0}, {0,0.5,0,0}, {0,0,0.1,0}, {0,0,0,0.05}},
                    Eigen::Matrix<double,1,1>{{10.0}},
                    compute_process_noise_approx((double)settings.accel_timer_us*1e-6, 0.01, 0.0001),
                    Eigen::Vector4d{altitude0,0,0,0.01},
                    (double)settings.accel_timer_us*1e-6
                );

            }

            bmi_accel_timer->start(true);
            std::this_thread::sleep_for(std::chrono::microseconds(509));
            bmi_gyro_timer->start(true);
            std::this_thread::sleep_for(std::chrono::microseconds(491));
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

                vqf_real_t accel[3] = {slot->x*9.80665, slot->y*9.80665, slot->z*9.80665};
                vqf->updateAcc(accel);

                slot->timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();

                accel_buffer_.commit();
                has_new_accel_.store(true, std::memory_order_release);
                
                if(dps368_){
                    // Update EKF with new accel data
                    const QuatData* quat = latest_quat();
                    const BaroData* barodat_ = latest_baro();

                    Eigen::Quaterniond q(quat->q[0], quat->q[1], quat->q[2], quat->q[3]);
                    Eigen::Vector3d a(slot->x, slot->y, slot->z);
                    a = q * a; 
                    EKF->KF_predict_alt((a(2) - 1.0)*9.80665); // use Z linear accel in world frame

                    BaroData* baro_slot = baro_buffer_.prepare_write();
                    baro_slot->altitude_m = EKF->xhat(0) - altitude0;  
                    baro_slot->vertical_speed_mps = EKF->xhat(1);
                    baro_slot->pressure_Pa = barodat_->pressure_Pa;
                    baro_slot->temperature_C = barodat_->temperature_C;

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

                vqf_real_t gyro[3] = {slot->x, slot->y, slot->z};
                vqf->updateGyr(gyro);

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

                    vqf_real_t mag[3] = {slot->x, slot->y, slot->z};
                    vqf->updateMag(mag);

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

            if (dps368_->isMeasurementReady(ispressureRDY, istempRDY)){

                BaroData* slot = baro_buffer_.prepare_write();

                bool ok = dps368_->readCalibrated(slot->pressure_Pa, slot->temperature_C);
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
            else dps_timer->set(true); // jitter compensation
 
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
            if (!has_new_accel_.exchange(false, std::memory_order_acquire)) {
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
            if (!has_new_gyro_.exchange(false, std::memory_order_acquire)) {
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
            if (!has_new_mag_.exchange(false, std::memory_order_acquire)) {
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
            if(!has_new_baro_.exchange(false, std::memory_order_acquire)) {
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
            if(!has_new_quat_.exchange(false, std::memory_order_acquire)){
                return false;
            }
            const auto* latest = quat_buffer_.get_latest();
            if(latest){
                out = *latest;
                return true;
            }
            return false;
        }

        // PCA9685 functions

        void init_PCA9685(uint8_t addr = 0x40, int osc_freq = 25500000, float pwm_freq = 500.0f){
            pwm_ = std::make_unique<PCA9685>(1, addr);

            pwm_->begin();
            pwm_->setOscillatorFrequency(osc_freq);
            pwm_->setPWMFreq(pwm_freq);

        }

        void Set_pwm(const uint8_t start_, const uint8_t count_, const uint16_t* off_vals){
            if(!pwm_){
                std::cerr << "PCA9685 not initialized\n";
                return;
            }

            if(count_ > 8) {
                pwm_->setPWMMultiple(start_, 8, on_vals, off_vals);
                pwm_->setPWMMultiple(start_ + 9, count_ - 8, on_vals, off_vals);
            }
            else {
                 pwm_->setPWMMultiple(start_, count_, on_vals, off_vals);
            }
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
        void stop_sensor_thread() {
            if(!running) {
                return;
            }

            running = false;

            if(sensor_update_thread.joinable()) {
                sensor_update_thread.join();
            }

        }

        void update_imu(){

            update_gyro();
            update_accel();
            if (mag_ != nullptr) update_mag();
            if (dps368_ != nullptr) update_baro();
            
        }

    ~IMU() {
        if (bmi_accel_timer) bmi_accel_timer->stop();
        if (bmi_gyro_timer)  bmi_gyro_timer->stop();
        if (mag_timer)       mag_timer->stop();
        if (dps_timer)       dps_timer->stop();
        if (quat_timer)      quat_timer->stop();

        stop_sensor_thread();

        if (pwm_) {
            pwm_->setPWMMultiple(0, 8, on_vals, on_vals);
            pwm_->setPWMMultiple(8, 8, on_vals, on_vals);
            pwm_.reset();
        }

        bmi088_.reset();
        mag_.reset();
        dps368_.reset();
        vqf.reset();
        parser.reset();

        close_gpio();
    }
    private:

    // read all sensors in the imu until running = false
    void update10DOF() {

        #ifdef __linux__
            sched_param param{};
            param.sched_priority = 99; 
            pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        #endif

        while (running) {
            update_imu();
        
            //______ Update Quat ______
            if(!quat_timer->check()) continue;

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

            //______ Rest Detect ______
            //if(vqf->getRestDetected()){
            //    altitude0 = EKF->xhat(0);
            //}
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
        DataBuffer<PWMval, 2>    pwm_buffer_;

        std::unique_ptr<DPS368> dps368_;
        std::unique_ptr<BMI088> bmi088_;
        std::unique_ptr<LIS2MDL> mag_;
        std::unique_ptr<PCA9685> pwm_;

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
        std::atomic<bool> isgyroRDY{false};
        std::atomic<bool> isaccelRDY{false};

        // pwm 
        uint16_t on_vals[16] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};  

        // sensor readings
        std::atomic<bool> running = false;
        std::thread sensor_update_thread;

        std::unique_ptr<Parser> parser;
};