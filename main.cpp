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

    std::cout << "start_sensor_thread()\n";
    imu.start_sensor_thread();

    while(counter < max_samples){


        if(imu.get_latest_baro_and_consume(barodat)){
            counter++;

            std::cout << "[" << counter << "] "
                      << "Timestamp: " << barodat.timestamp_us << " us\n"
                      << "  Pressure:    " << barodat.pressure_Pa << " hPa\n"
                      << "  Temperature: " << barodat.temperature_C << " °C\n"
                      << "  Altitude:    " << barodat.altitude_m << " m (rel)\n"
                      << "  ───────────────────────────────\n";
        }
       if(imu.get_latest_gyro_and_consume(gydat)){

            std::cout << "Timestamp: " << gydat.timestamp_us << " us\n"
                      << "gx: " << gydat.x << "\n"
                      << "gy: " << gydat.y << "\n"
                      << "gz: " << gydat.z << "\n"
                      << "  ───────────────────────────────\n";
       }
       if(imu.get_latest_accel_and_consume(accdat)){

           std::cout << "Timestamp: " << accdat.timestamp_us << " us\n"
                      << "ax: " << accdat.x << "\n"
                      << "ay: " << accdat.y << "\n"
                      << "az: " << accdat.z << "\n"
                      << "  ───────────────────────────────\n";
       }
        if(imu.get_latest_mag_and_consume(magdat)){

             std::cout << "Timestamp: " << magdat.timestamp_us << " us\n"
                       << "mx: " << magdat.x << "\n"
                       << "my: " << magdat.y << "\n"
                       << "mz: " << magdat.z << "\n"
                       << "  ───────────────────────────────\n";
        }
        else{
            std::cout << "No new data yet...\n";
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    gpioTerminate();

    return 1;
}