#include <pigpio.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <unistd.h> 
#include <csignal> 
#include <iomanip>

#include "LIS2MDL.h"
#include "Parser.h"

volatile bool running = true;

void signalHandler(int signal) {
    running = false;
}

int main() {

    Eigen::Matrix3d calibMatrix = Eigen::Matrix3d::Zero();
    Eigen::Vector3d offset = Eigen::Vector3d::Zero();

    Parser par("config.txt");
    par.loadVar("calibMatrix_mag", calibMatrix);
    par.loadVar("offset_mag", offset);
    std::cout << "calibMatrix: \n" << calibMatrix << "\n";
    std::cout << "offset: \n" << offset << "\n";

    LIS2MDL mag(LIS2MDL::ODR::ODR_100HZ, calibMatrix);

    std::this_thread::sleep_for(std::chrono::microseconds(11000));
    
    int counter = 0;
    mag.setHardIronOffsets(offset(0),offset(1),offset(2));
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
// g++ -Wall -o main main.cpp Parser.cpp -I /usr/include/eigen3 -lpigpio -lrt -O2 -ftree-vectorize
