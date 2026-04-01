#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "IMU.h"

int main(){

    IMU imu(IMU::PerformanceMode::High, true, true);
    imu.init_PCA9685();
    uint16_t on_vals[16] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0}; 

    int counter = 0; const int max_samples = 500;
    BaroData barodat;
    GyroData gydat;
    AccelData accdat;
    MagData magdat;
    QuatData quat;
    Eigen::Vector3d rotated_acc = Eigen::Vector3d::Zero();

    const AccelData* accdatptr;
    imu.start_sensor_thread();
    std::unique_ptr<Timer> print_timer = std::make_unique<Timer>(std::chrono::milliseconds(50));
    print_timer->start();
    while(counter < max_samples){

        if(imu.get_latest_quat_and_consume(quat)){

            accdatptr = imu.latest_accel();
            Eigen::Quaterniond q(quat.q[0], quat.q[1], quat.q[2], quat.q[3]);
            Eigen::Vector3d acc_vec(accdatptr->x, accdatptr->y, accdatptr->z);
            rotated_acc = q * acc_vec;
            //std::cout << " x: " << accdatptr->x  << " y: " << accdatptr->y  << " z: " << accdatptr->z  << "\n\n";
            //std::cout << quat.timestamp_us << "\n";

        }
        
        if(imu.get_latest_gyro_and_consume(gydat)){

            //std::cout << gydat.timestamp_us << "\n";
        }

        if(print_timer->check()){
            //std::cout << " w: " << quat.q[0]  << " x: " << quat.q[1]  << " y: " << quat.q[2]  << " z: " << quat.q[3] << "\n";
            //std::cout << " x: " << accdatptr->x  << " y: " << accdatptr->y  << " z: " << accdatptr->z  << "\n\n";
            std::cout << " Baro Altitude: " << barodat.altitude_m << " m" << " Baro Pressure: " << barodat.pressure_Pa << " Pa" << " Baro Temp: " << barodat.temperature_C << " C" << " Vertical Speed: " << barodat.vertical_speed_mps << " m/s\n";
            //std::cout << "rot x: " << rotated_acc.x()  << " rot y: " << rotated_acc.y()  << " rot z: " << rotated_acc.z()  << "\n";
            
            int val = static_cast<int>((double)counter/(double)max_samples*4095.0);
            on_vals[0] = val;
            on_vals[1] = val;
            on_vals[2] = val;
            on_vals[3] = val;
            imu.Set_pwm(0, 4, on_vals);

            counter++;
        }
            
        if(imu.get_latest_baro_and_consume(barodat)){
            //counter++;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }

    imu.stop_sensor_thread(); 

    return 1;
}
