#include <iostream>
#include <thread>
#include <chrono>

#include "IMU.h"


int main(){

    //if (gpioInitialise() < 0) {
    //    std::cerr << "pigpio initialisation failed." << std::endl;
    //}

    IMU imu(IMU::PerformanceMode::Ultra, true, true);

    int counter = 0; const int max_samples = 5000;
    BaroData barodat;
    GyroData gydat;
    AccelData accdat;
    MagData magdat;
    QuatData quat;
  
    imu.start_sensor_thread();
    imu.update_quat_thread();
  
    while(counter < max_samples){

        if(imu.get_latest_quat_and_consume(quat)){

            counter++;
            std::cout << " w: " << quat.q[0]  << " x: " << quat.q[1]  << " y: " << quat.q[2]  << " z: " << quat.q[3] << "\n";
            //std::cout << quat.timestamp_us << "\n";
        }
        std::this_thread::sleep_for(std::chrono::microseconds(50));
    }

    imu.stop_sensor_threads();
    //gpioTerminate();

    return 1;
}