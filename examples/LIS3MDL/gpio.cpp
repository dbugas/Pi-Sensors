#include "gpio.h"

bool gpio::init_pin(int pin, PIN_IO IO)
{
    if(IO == PIN_IO::OUT){
        gpioResult = gpioSetMode(pin, PI_OUTPUT);
    }
    else {
        gpioResult = gpioSetMode(pin, PI_INPUT); ;
    }

    return pigpio_Init_check();
}
bool gpio::init_pwm(int GPIO_pin, int pwm_val){

    pwm_GPIO.emplace_back(GPIO_pin);
    gpioResult = gpioPWM(GPIO_pin,0);
    if(!pigpio_Init_check()){
        std::cout << "Failed PWM init on pin " << GPIO_pin << "\n";
        return false;
    }
    gpioResult = gpioSetPWMfrequency(GPIO_pin, 500); // Set pwm_pin to 500Hz. 27
    if(!pigpio_Init_check()){
        std::cout << "Failed to set PWM frequency on pin " << GPIO_pin << "\n";
        return false;
    }
    gpioResult = gpioSetPWMrange(GPIO_pin, pwm__max);
    if(!pigpio_Init_check()){
        std::cout << "Failed to set PWM range on pin " << GPIO_pin << "\n";
        return false;
    }
    return true;
}
bool gpio::init_uart(int baud)
{
    char port[] =  "/dev/ttyS0";
    serialHandle = serOpen(port, baud, 0);
    
    gpioResult = serialHandle;
    //char msg[] = "";
    return pigpio_Init_check();
}
