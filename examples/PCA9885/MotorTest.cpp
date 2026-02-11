#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>

#include "PCA9685.h"
#include "aux.h"

int main() {
    PCA9685 pwm(1, 0x41);
    pwm.setOutputMode(true);

    std::string devicePath = "/dev/input/event0";  
    KeyboardIO keyboard(devicePath);
    if (!keyboard.isOpen()) {
        std::cerr << "Failed to open the keyboard device." << std::endl;
        return 1;
    }
    ThreadManager manager(std::thread::hardware_concurrency());

    manager.addTask(0, [&keyboard]() {
        keyboard.listenForEvents();
    });

    pwm.begin();
    pwm.setOscillatorFrequency(25500000);
    pwm.setPWMFreq(500.0);
    //pwm.setOutputMode(true);
    
    int i = 0;
    int pwm_val[16] = {2000,2000,2000,2000, 0,0,0,0, 0,0,0,0, 0,0,0,0}; 
    int channel = 0;
    int sign = 1;
    uint16_t on_vals[16] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};      // Example: all start at count 0
    uint16_t off_vals[16] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};  // Different duty cycles
   
    std::cout << "press ECS to quit\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    while (true) {

        int keyCode = keyboard.getKeyEvent();
         if(keyCode == KEY_ESC) {
                keyboard.stopListening();
                std::cout << "ESC key pressed. Exiting..." << std::endl;
                break;
        }

        if(keyCode == KEY_W){
            pwm_val[channel] += 10;
            pwm_val[channel] = std::clamp(pwm_val[channel], 0, 4096);
            pwm.setPWM(channel, 0, pwm_val[channel]);
            std::cout << "channel = " << channel << ", PWM = " << pwm_val[channel] << "\n"; 
        }
        if(keyCode == KEY_S){
            pwm_val[channel] -= 10;
            pwm_val[channel] = std::clamp(pwm_val[channel], 0, 4096);
            pwm.setPWM(channel, 0, pwm_val[channel]);
            std::cout << "channel = " << channel << ", PWM = " << pwm_val[channel] << "\n"; 
        }

        if(keyCode == KEY_0){
            channel = 0;
            std::cout << "channel = " << channel << ", PWM = " << pwm_val[channel] << "\n"; 
        }
        if(keyCode == KEY_1){
            channel = 1;
            std::cout << "channel = " << channel << ", PWM = " << pwm_val[channel] << "\n"; 
        }
        if(keyCode == KEY_2){
            channel = 2;
            std::cout << "channel = " << channel << ", PWM = " << pwm_val[channel] << "\n"; 
        }
        if(keyCode == KEY_3){
            channel = 3;
            std::cout << "channel = " << channel << ", PWM = " << pwm_val[channel] << "\n"; 
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    for(int ch = 0; i < 16; i++) {
        pwm_val[ch] = 0;
        pwm.setPWM(ch, 0, pwm_val[ch]);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    gpioTerminate();
    return 0;
}
// g++ MotorTest.cpp PCA9685.cpp -o motor_test -lpigpio -lpthread -std=c++23