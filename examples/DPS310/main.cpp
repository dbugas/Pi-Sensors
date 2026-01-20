#include <iostream>
#include <pigpio.h>
#include <unistd.h> 
#include <stdexcept>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <thread>

#include "dps310.h"

int main() {
    try {
        if (gpioInitialise() < 0) {
            std::cerr << "pigpio initialisation failed." << std::endl;
        }
        bool temp_ready, pressure_ready = false;
        // Test case : Pressure at 64 Hz, 8x oversampling; Temperature at 4 Hz, 1x oversampling
        int pressure_meas_rate1 = 32; //13.13 // 6.96 // 2.18
        int pressure_os_rate1 = 16;
        int temp_meas_rate1 = 1;
        int temp_os_rate1 = 1;

        double altitude0 = 0.0;
        double altitude0_ave = 0.0;
        double alt_err = 0.0;
        double altitude = 0.0;
        double altitude_change = 0.0;
        double dh = 0.0;
        double pressure, temperature;
        DPS310 sensor(pressure_meas_rate1, pressure_os_rate1, temp_meas_rate1, temp_os_rate1);

        int counter = 0;
        while(counter < 10){
            if(sensor.isMeasurementReady(pressure_ready, temp_ready)){
                if (!sensor.readData(pressure, temperature,temp_ready, pressure_ready)) {
                    std::cerr << "Failed to read data. Exiting..." << std::endl;
                    return 1;
                } else counter++;
                if (sensor.calculateAltitudeChange(pressure, temperature, altitude0, altitude_change)) {
                    altitude0_ave += altitude0;
                }
            }
        }
        altitude0 = altitude0_ave/10.0;
        dh = altitude - altitude0;
        std::cout << std::fixed << std::setprecision(4)
        << " Altitude: "        << std::setw(7) << altitude << " m,"
        << "  Alt. Change: "    << std::setw(7) << altitude_change << " m,"
        << "  dh: "             << std::setw(7) << dh;
        auto start = std::chrono::high_resolution_clock::now();

        while(counter < 100) {
            if(sensor.isMeasurementReady(pressure_ready, temp_ready)){

                if (!sensor.readData(pressure, temperature, temp_ready, pressure_ready)) {
                    std::cerr << "Failed to read data. Exiting..." << std::endl;
                    return 1;
                } else counter++;
            
                if (sensor.calculateAltitudeChange(pressure, temperature, altitude, altitude_change)) {
                    dh = altitude - altitude0 ;
                    alt_err += dh;
                
                    std::cout << std::fixed << std::setprecision(4)
                              << " Altitude: "        << std::setw(7) << altitude << " m,"
                              << "  Alt. Change: "    << std::setw(7) << altitude_change << " m,"
                              << "  dh: "             << std::setw(7) << dh*100.0 << " cm";
                }
            
                std::cout << std::fixed << std::setprecision(4)
                          << "  Pressure: "     << std::setw(7) << pressure << " Pa,"
                          << "  Temp: "         << std::setw(7) << temperature << " °C"
                          << std::endl;
            }
        }

        auto end = std::chrono::high_resolution_clock::now();
        // Calculate duration
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds >(end - start);
        std::cout << "Time taken: " << duration.count()/(double)100 << " milliseconds " << std::endl;
        std::cout << "dh error: " << alt_err/100.0 << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        gpioTerminate(); // Clean up pigpio on error
        return 1;
        
    }
    
    gpioTerminate(); // Clean up pigpio once at the end
    return 0;
}
// g++ -Wall -o main main.cpp -lpigpio -lrt -O2 -ftree-vectorize