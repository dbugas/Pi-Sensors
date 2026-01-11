#include <pigpio.h>
#include <iostream>
#include <chrono>
#include <thread>

#include "LISxMDL.h"

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialisation failed." << std::endl;
        return 1;
    }

LISxMDL mag(
    LISxMDL::FullScale::Gauss_16,
    LISxMDL::ODR::Hz_80
    );

    int counter = 0;
    while(counter < 100){

        if(mag.dataReady()){
            double mx, my, mz;
            mag.read_gauss(mx, my, mz);

            counter ++;
            std::cout << "mx: " << mx << " my: " << my << " mz: " << mz << "\n"; 
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
    gpioTerminate();
    return 0;
}
// g++ -Wall -o main main.cpp -lpigpio -lrt -O2