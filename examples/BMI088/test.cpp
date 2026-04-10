#include <pigpio.h>
#include <iostream>
#include <cstdint>

#include "BMI088.h"
static constexpr uint8_t BMI088_ACC_CHIP_ID  = 0x00;
static constexpr uint8_t BMI088_GYR_CHIP_ID  = 0x00;

uint8_t readAccelChipID(int spi_acc){
        uint8_t tx[3] = {
            static_cast<uint8_t>(BMI088_ACC_CHIP_ID | 0x80), // CHIP_ID + read
            0x00,
            0x00
        };
        uint8_t rx[3] = {0};

        spiXfer(spi_acc, reinterpret_cast<char*>(tx), reinterpret_cast<char*>(rx), 3);
        spiXfer(spi_acc, reinterpret_cast<char*>(tx), reinterpret_cast<char*>(rx), 3);
        
        return rx[2]; // rx[1] is dummy, rx[2] is CHIP_ID
    }

int main()
{
    BMI088 bmi;

    double ax, ay, az;
    for(int i = 0 ; i < 10; i++){
        bmi.readAccelCalibrated(ax, ay, az);
        std::cout << "ACC [g]: "  << ax << ", " << ay << ", " << az << "\n";
        gpioDelay(10000); 
    }
    /*
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialise pigpio\n";
        return 1;
    }

    const unsigned SPI_BAUD  = 10000000;
    const unsigned SPI_FLAGS = 0x00  ;   // AUX SPI (SPI1), mode 0

    int spi_acc  = spiOpen(0, SPI_BAUD,0);  // CS0 = Accelerometer
    int spi_gyro = spiOpen(1, SPI_BAUD,0);  // CS1 = Gyroscope

    if (spi_acc < 0 || spi_gyro < 0) {
        std::cerr << "Failed to open SPI handles\n";
        gpioTerminate();
        return 1;
    }

    std::cout << "Reading BMI088 Chip IDs...\n\n";

    // ---------- Gyro Chip ID (works the same as before) ----------
    {
        char tx[2] = {0x00 | 0x80, 0x00};
        char rx[2] = {0};

        if (spiXfer(spi_gyro, tx, rx, 2) == 2) {
            std::cout << "Gyro  CHIP_ID: 0x" << std::hex << (int)(uint8_t)rx[1] 
                      << std::dec << "  (expected 0x0F)\n";
        } else {
            std::cerr << "Gyro read failed\n";
        }
    }

    uint8_t accel_id;
    accel_id = readAccelChipID(spi_acc);
    std::cout << "Accel CHIP_ID: 0x" << std::hex << (int)(uint8_t)accel_id << std::dec 
    << " (expected 0x1E)\n";

    spiClose(spi_gyro);
    spiClose(spi_acc);
    gpioTerminate();

    std::cout << "\nDone.\n";
    return 0;
    */
}
// g++ -std=c++17 -O2 -o test test.cpp -lpigpio -lrt -pthread