#include <pigpio.h>
#include <iostream>
#include <unistd.h>
#include <cstdio>
#include <cstdlib>

#define LIS3MDL_I2C_ADDR    0x1C
#define WHO_AM_I_REG        0x0F
#define EXPECTED_WHO_AM_I   0x3D

#define CTRL_REG1           0x20
#define CTRL_REG3           0x22
#define CTRL_REG4           0x23

// Output data registers (auto-increment from 0x28)
#define OUT_X_L             0x28 | 0x80  // Set bit 7 for auto-increment

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialisation failed." << std::endl;
        return 1;
    }

    int handle = i2cOpen(1, LIS3MDL_I2C_ADDR, 0);  // Bus 1, address 0x1C
    if (handle < 0) {
        std::cerr << "Failed to open I2C device: error code " << handle << std::endl;
        gpioTerminate();
        return 1;
    }

    std::cout << "Successfully opened I2C device at 0x1C" << std::endl;

    // Read WHO_AM_I register to verify connection
    char who_am_i = i2cReadByteData(handle, WHO_AM_I_REG);
    std::cout << "WHO_AM_I = 0x" << std::hex << (int)(unsigned char)who_am_i << std::dec << std::endl;

    if (who_am_i != EXPECTED_WHO_AM_I) {
        std::cerr << "Unexpected device ID! Expected 0x3D for LIS3MDL." << std::endl;
        i2cClose(handle);
        gpioTerminate();
        return 1;
    }

    std::cout << "LIS3MDL detected correctly!" << std::endl;

    // Basic initialisation: continuous mode, 10 Hz ODR, high-res XY, high-res Z
    i2cWriteByteData(handle, CTRL_REG1, 0x70);  // Temp off, XY high perf, ODR = 10 Hz
    i2cWriteByteData(handle, CTRL_REG3, 0x00);  // Continuous conversion mode
    i2cWriteByteData(handle, CTRL_REG4, 0x0C);  // Z high performance mode

    std::cout << "Sensor initialised in continuous mode (10 Hz)." << std::endl;

    // Read magnetic field every second
    while (true) {
        char data[6];
        int bytes = i2cReadI2CBlockData(handle, OUT_X_L, data, 6);

        if (bytes == 6) {
            int16_t x = (int16_t)(data[0] | (data[1] << 8));
            int16_t y = (int16_t)(data[2] | (data[3] << 8));
            int16_t z = (int16_t)(data[4] | (data[5] << 8));

            std::cout << "Mag X: " << x << "  Y: " << y << "  Z: " << z << std::endl;
        } else {
            std::cerr << "Failed to read data (" << bytes << " bytes)" << std::endl;
        }

        sleep(1);
    }

    i2cClose(handle);
    gpioTerminate();
    return 0;
}
// g++ -Wall -o lis3mdl_test main.cpp -lpigpio -lrt