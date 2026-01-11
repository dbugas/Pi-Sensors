#include <iostream>
#include <chrono>
#include <thread>

#include "PCA9685.h"

int main() {
    PCA9685 pwm(1, 0x41);
    std::cout << "enter begin\n";
    pwm.begin();
    std::cout << "enter setOscillatorFrequency\n";
    pwm.setOscillatorFrequency(6000000);
    std::cout << "enter setPWMFreq\n";
    pwm.setPWMFreq(400.0);
    //pwm.setOutputMode(true);
    
    int i = 0;
    uint16_t pwm_val = 0;
    int sign = 1;
    uint16_t on_vals[16] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};      // Example: all start at count 0
    uint16_t off_vals[16] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};  // Different duty cycles
    while (true) {

        auto start = std::chrono::high_resolution_clock::now();

        pwm.setPWMMultiple(0,4,on_vals,off_vals);
        //    for(int channel = 0; channel < 4; channel++ ){
        //        pwm.setPWM(channel, 0, pwm_val);
        //    }
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        std::cout << "Total time: " << duration << " microseconds\n";
        i++;
        pwm_val += 16*sign;
        if(pwm_val >= 4096) sign = -1, pwm_val = 4096;
        else if(pwm_val <= 0) sign = 1, pwm_val = 0;
        off_vals[0] = pwm_val;
        off_vals[1] = pwm_val;
        off_vals[2] = pwm_val;
        off_vals[3] = pwm_val;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }


    return 0;
}
// g++ main.cpp PCA9685.cpp -o main -lpigpio -lpthread -O3