#include <iostream>
#include <pigpio.h>
#include <unistd.h> 
#include <stdexcept>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <thread>

#include "dps368.h"

int main() {

    DPS368 dps(DPS368::MeasurementRate::Hz64, DPS368::OversamplingRate::OSR8,
            DPS368::MeasurementRate::Hz1, DPS368::OversamplingRate::OSR1);

    std::this_thread::sleep_for(std::chrono::microseconds(80000));

    float pressure, temperature;
    auto read_start  = std::chrono::steady_clock::now();
    if (dps.readCalibrated(pressure, temperature))
    {
        auto read_end = std::chrono::steady_clock::now();
        double read_duration_ms = std::chrono::duration<double, std::milli>(read_end - read_start).count();
        std::cout << "Pressure: " << pressure << " Pa\n";
        std::cout << "Temp: " << temperature << " °C\n";
        std::cout << "Read time: " << read_duration_ms << " ms\n";
    }
    std::this_thread::sleep_for(std::chrono::microseconds(17000));
    read_start  = std::chrono::steady_clock::now();
    if (dps.readCalibrated(pressure, temperature))
    {
        auto read_end = std::chrono::steady_clock::now();
        double read_duration_ms = std::chrono::duration<double, std::milli>(read_end - read_start).count();
        std::cout << "Pressure: " << pressure << " Pa\n";
        std::cout << "Temp: " << temperature << " °C\n";
        std::cout << "Read time: " << read_duration_ms << " ms\n";
    }

    return 0;
}
// g++ -Wall -o main main.cpp -lpigpio -lrt -O2 -ftree-vectorize