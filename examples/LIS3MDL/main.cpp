#include <pigpio.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>

#include "LIS2MDL.h"

int main() {

LISxMDL mag(
    LISxMDL::ODR::ODR_100HZ
    );

    std::this_thread::sleep_for(std::chrono::microseconds(11000));
    
    int counter = 0;
    mag.setHardIronOffsets(2.0f,0.0f,0.0f);
    while(counter < 10){

        if(mag.dataReady()){
            double mx, my, mz;
            mag.read_gauss(mx, my, mz);
            double norm = std::sqrt(mx*mx + my*my + mz*mz);
            counter ++;
            std::cout << "mx: " << mx << " my: " << my << " mz: " << mz << " norm: " << norm << "\n"; 
        }
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }

    return 0;
}
// g++ -Wall -o main main.cpp -lpigpio -lrt -O2 -ftree-vectorize
