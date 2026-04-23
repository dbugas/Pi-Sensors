#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "IMU.h"

int main(){

    std::string path = static_cast<std::string>(SOURCE_DIR) + static_cast<std::string>("../asdf");
    std::cout << path << "\n";
    IMU imu(IMUConfig::PerformanceMode::Custom, false, false);
    /*
    imu.init_PCA9685();
    uint16_t on_vals[16] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0}; 

    int counter = 0; const int max_samples = 0;
    BaroData barodat;
    GyroData gydat;
    AccelData accdat;
    MagData magdat;
    QuatData quat;
    Eigen::Vector3d rotated_acc = Eigen::Vector3d::Zero();

    const AccelData* accdatptr;
    //imu.start_sensor_thread();
    std::unique_ptr<Timer> print_timer = std::make_unique<Timer>(std::chrono::milliseconds(50));
    print_timer->start();
    while(counter < max_samples){
        if(imu.get_latest_accel_and_consume(accdat)){
            counter++;

            std::cout << "ax: " << accdat.x << " ay: " << accdat.y << " az: " << accdat.z << "\n";
            std::cout << "gx: " << gydat.x << " gy: " << gydat.y << " gz: " << gydat.z << "\n\n";
        }
        
        if(imu.get_latest_gyro_and_consume(gydat)){

            //std::cout << gydat.timestamp_us << "\n";
        }

        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }

    imu.stop_sensor_thread(); 
    */
    return 1;
}