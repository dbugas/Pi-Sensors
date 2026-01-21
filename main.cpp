#include <iostream>
#include <thread>
#include <chrono>

#include "IMU.h"


int main(){

    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialisation failed." << std::endl;
    }

    IMU imu(
        32,   // pressureRate
        16,   // pressureOversampling
        1,    // tempRate
        1,    // tempOversampling
        6,    // accel_scale
        800.0,// accel_odr
        BMI088::GyroRange::DPS_1000,
        BMI088::GyroBandwidth::ODR_1000Hz_BW_116Hz,
        LISxMDL::FullScale::Gauss_8,
        LISxMDL::ODR::Hz_80
    );
    int counter = 0; const int max_samples = 50;
    BaroData barodat;
    GyroData gydat;
    AccelData accdat;
    MagData magdat;
    QuatData quat;
  
    imu.start_sensor_thread();
    imu.update_quat_thread(false);
  
    while(counter < max_samples){

        if(imu.get_latest_quat_and_consume(quat)){

            counter++;
            std::cout << " w: " << quat.q[0]  << " x: " << quat.q[1]  << " y: " << quat.q[2]  << " z: " << quat.q[3] << "\n";
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    imu.stop_sensor_threads();
    gpioTerminate();

    return 1;
}