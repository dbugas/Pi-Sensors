#include <pigpio.h>
#include <iostream>
#include <cstdint>
#include <vector>
#include <cstring>
#include <cmath>
#include <atomic>

#include "vqf.h"
#include "timer.h"
#include "BMI088.h"

// ---------- non-interrupt handling ----------

struct IMUData {
    double ax, ay, az;
    double gx, gy, gz;
    uint64_t timestamp_us;
};
static IMUData imu_buffer[2];
static std::atomic<IMUData*> latest_imu_data{&imu_buffer[0]};

int main()
{

    //BMI088 bmi(6, 800.0, BMI088::AccelOversampling::Normal, BMI088::GyroRange::DPS_1000, BMI088::GyroBandwidth::ODR_1000Hz_BW_116Hz);
    BMI088 bmi;
    int counter = 10;
    
    // ---------- non-interrupt handling ----------
    imu_buffer[0] = {};
    imu_buffer[1] = {};

    std::unique_ptr<Timer> imu_timer = std::make_unique<Timer>(std::chrono::microseconds(1250));
    std::unique_ptr<Timer> print_timer = std::make_unique<Timer>(std::chrono::milliseconds(50));

    imu_timer->set_callback([&](){
        // Choose the buffer NOT currently published
        IMUData* current = latest_imu_data.load();
        IMUData* new_sample = (current == &imu_buffer[0]) ? &imu_buffer[1] : &imu_buffer[0];

        // Read raw data
        bool ok = /*bmi.readAccelCalibrated(new_sample->ax, new_sample->ay, new_sample->az) &&*/
                 bmi.readGyro(new_sample->gx, new_sample->gy, new_sample->gz);

        if(ok) {
            new_sample->timestamp_us =
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()).count();
            latest_imu_data.store(new_sample);
        }
    });
    
    imu_timer->start();
    print_timer->start();

    while(counter < 0)
    {
        // ---------- non-interrupt handling ----------
        if(print_timer->check())
        {
            IMUData* data = latest_imu_data.load();
            std::cout << "" << data->timestamp_us << "\n";
            std::cout << "GX: " << data->gx  
                    << "  GY: " << data->gy  
                    << "  GZ: " << data->gz  << "\n";
            std::cout << "AX: " << data->ax 
                    << "  AY: " << data->ay
                    << "  AZ: " << data->az  << "\n\n";
            counter++;
        }
        
       gpioDelay(50);
    }

    imu_timer->stop();
    print_timer->stop();
    return 0;
}

// g++ -std=c++23 main.cpp vqf.cpp -lpigpio -lrt -pthread -o main
// real time: sudo chrt -r 70 ./main 
