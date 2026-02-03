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

// ---------- interrupt handling ----------
std::atomic<bool> gyro_drdy{false};
std::atomic<bool> accel_drdy{false};
const int gyro_int = 22;
const int accel_int = 23;

void gyroIntCallback(int gpio, int level, uint32_t tick)
{
    if (level == PI_HIGH)   // rising edge
        gyro_drdy.store(true, std::memory_order_release);
}

void accelIntCallback(int gpio, int level, uint32_t tick)
{
    if (level == PI_HIGH)   // rising edge
        accel_drdy.store(true, std::memory_order_release);
}

void setupImuInterrupts()
{
// Gyro INT3 → GPIO 22
    gpioSetMode(gyro_int, PI_INPUT);
    gpioSetPullUpDown(gyro_int, PI_PUD_OFF);   // push-pull, active high
    gpioSetAlertFunc(gyro_int, gyroIntCallback);

    // Accel INT1 → GPIO 23
    gpioSetMode(accel_int, PI_INPUT);
    gpioSetPullUpDown(accel_int, PI_PUD_UP);   // push-pull, active high
    gpioSetAlertFunc(accel_int, accelIntCallback);
}

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
    if (gpioInitialise() < 0)
    {
        std::cerr << "pigpio init failed\n";
        return 1;
    }

    //BMI088 bmi(6, 800.0, BMI088::AccelOversampling::Normal, BMI088::GyroRange::DPS_1000, BMI088::GyroBandwidth::ODR_1000Hz_BW_116Hz);
    BMI088 bmi;
    int counter = 0;
    
    // ---------- non-interrupt handling ----------
    imu_buffer[0] = {};
    imu_buffer[1] = {};

    std::unique_ptr<Timer> imu_timer = std::make_unique<Timer>(std::chrono::microseconds(1250));
    std::unique_ptr<Timer> print_timer = std::make_unique<Timer>(std::chrono::milliseconds(10));

    imu_timer->set_callback([&](){
        // Choose the buffer NOT currently published
        IMUData* current = latest_imu_data.load();
        IMUData* new_sample = (current == &imu_buffer[0]) ? &imu_buffer[1] : &imu_buffer[0];

        // Read raw data
        bool ok = bmi.readAccelCalibrated(new_sample->ax, new_sample->ay, new_sample->az) &&
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
        /*
        // ---------- interrupt handling ----------
        if (gyro_drdy.exchange(false))
        {
            double gx, gy, gz;
            bmi.readGyro(gx, gy, gz);

            std::cout << "gyro: " << gx << " " << gy << " " << gz << "\n";
        }

        if (accel_drdy.exchange(false))
        {
            double ax, ay, az;
            bmi.readAccel(ax, ay, az);

            std::cout << "accel: " << ax << " " << ay << " " << az << "\n";
            counter++;
        }
        */

        
        // ---------- non-interrupt handling ----------
        if(print_timer->check())
        {
            //IMUData* data = latest_imu_data.load();
            //std::cout << "" << data->timestamp_us << "\n";
            //std::cout << "GX: " << data->gx  
            //        << "  GY: " << data->gy  
            //        << "  GZ: " << data->gz  << "\n";
            //std::cout << "AX: " << data->ax 
            //        << "  AY: " << data->ay
            //        << "  AZ: " << data->az  << "\n\n";
            counter++;
        }
        
       gpioDelay(50);
    }

    return 0;
}

// g++ -std=c++23 main.cpp vqf.cpp -lpigpio -lrt -pthread -O3 -o main
// real time: sudo chrt -r 70 ./main 
