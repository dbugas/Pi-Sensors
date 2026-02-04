#include <iostream>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "IMU.h"

int main(){

    IMU imu(IMU::PerformanceMode::High, true, true);

    int counter = 0; const int max_samples = 10;
    BaroData barodat;
    GyroData gydat;
    AccelData accdat;
    MagData magdat;
    QuatData quat;
  
    const AccelData* accdatptr;
    imu.start_sensor_thread();
    imu.update_quat_thread();
    std::unique_ptr<Timer> print_timer = std::make_unique<Timer>(std::chrono::milliseconds(50));
    print_timer->start();
    while(counter < max_samples){

        if(imu.get_latest_quat_and_consume(quat)){

            accdatptr = imu.latest_accel();
            Eigen::Quaterniond q(quat.q[0], quat.q[1], quat.q[2], quat.q[3]);
            Eigen::Vector3d acc_vec(accdatptr->x, accdatptr->y, accdatptr->z);
            Eigen::Vector3d rotated_acc = q * acc_vec;
            if(print_timer->check()){
                //std::cout << " x: " << rotated_acc.x()  << " y: " << rotated_acc.y()  << " z: " << rotated_acc.z()  << "\n";
                //std::cout << " x: " << accdatptr->x  << " y: " << accdatptr->y  << " z: " << accdatptr->z  << "\n\n";
                //std::cout << " w: " << quat.q[0]  << " x: " << quat.q[1]  << " y: " << quat.q[2]  << " z: " << quat.q[3] << "\n";
                //std::cout << quat.timestamp_us << "\n";

                //std::cout << " Baro Altitude: " << barodat.altitude_m << " m" << " Baro Pressure: " << barodat.pressure_Pa << " Pa" << " Baro Temp: " << barodat.temperature_C << " C" << " Vertical Speed: " << barodat.vertical_speed_mps << " m/s\n";

                counter++;
            }
        }
        
        if(imu.get_latest_baro_and_consume(barodat)){
            //counter++;
        
        }

        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }

    imu.stop_sensor_threads(); 

    return 1;
}