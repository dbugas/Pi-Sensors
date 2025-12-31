#include <pigpio.h>
#include <iostream>
#include <cstdint>
#include <vector>
#include <cstring>
#include <cmath>

#include "vqf.h"
#include "timer.h"
#include "BMI088.h"

struct IMUData {
    double ax, ay, az;
    double gx, gy, gz;
    uint64_t timestamp_us;
};
static IMUData imu_buffer[2];
static std::atomic<IMUData*> latest_imu_data{&imu_buffer[0]};

int main()
{
    if (gpioInitialise() < 0)
    {
        std::cerr << "pigpio init failed\n";
        return 1;
    }

    BMI088 bmi(6, 800.0, BMI088::AccelOversampling::Normal, 
        BMI088::GyroRange::DPS_1000, BMI088::GyroBandwidth::ODR_1000Hz_BW_116Hz);
    imu_buffer[0] = {};
    imu_buffer[1] = {};

    int counter = 0;
    std::unique_ptr<Timer> imu_timer = std::make_unique<Timer>(std::chrono::microseconds(1250));
    std::unique_ptr<Timer> print_timer = std::make_unique<Timer>(std::chrono::milliseconds(10));

    imu_timer->set_callback([&](){
        // Choose the buffer NOT currently published
        IMUData* current = latest_imu_data.load();
        IMUData* new_sample = (current == &imu_buffer[0]) ? &imu_buffer[1] : &imu_buffer[0];

        // Read raw data
        bool ok = bmi.readAccel(new_sample->ax, new_sample->ay, new_sample->az) &&
                  bmi.readGyro(new_sample->gx, new_sample->gy, new_sample->gz);

        if(ok) {
            new_sample->timestamp_us =
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::steady_clock::now().time_since_epoch()
                ).count();

            latest_imu_data.store(new_sample);
        }
    });
    imu_timer->start();
    print_timer->start();

    while(counter < 10)
    {
        if(print_timer->check())
        {
            auto data = latest_imu_data.load();
            std::cout << "time stamp " << data->timestamp_us << "\n";
            std::cout << "GX: " << data->gx  
                    << "  GY: " << data->gy  
                    << "  GZ: " << data->gz  << "\n";
            std::cout << "AX: " << data->ax 
                    << "  AY: " << data->ay
                    << "  AZ: " << data->az  << "\n\n";
            counter++;
        }
    }

    return 0;
}

// g++ -std=c++23 main.cpp vqf.cpp -lpigpio -lrt -pthread -o main

