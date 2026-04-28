#pragma once
#include <iostream>
#include <vector>

#include "pigpio.h"

class gpio
{
    public:
        gpio() 
        {

        }
        enum PIN_IO {
                  IN, // for inputs signals, ie switches, Hall
                  OUT // for output signals, ie LEDs
                };

        bool init_pin(unsigned pin, PIN_IO IO); // GPIO pins in only 
        bool init_uart(int baud);
        bool init_pwm(int GPIO_pin, int pwm_val = 0);
        // --- I2C Functions ---
        int i2cOpenBus(unsigned bus, unsigned addr) { return i2cOpen(bus, addr, 0); }
        void i2cCloseBus(int handle) { if (handle >= 0) i2cClose(handle); }
        int i2cWriteByte(int handle, uint8_t reg, uint8_t val) {
            return i2cWriteByteData(handle, reg, val);
        }
        int i2cReadByte(int handle, uint8_t reg) {
            return i2cReadByteData(handle, reg);
        }
        int i2cReadBlock(int handle, uint8_t reg, char* buf, unsigned n) {
            return i2cReadI2CBlockData(handle, reg, buf, n);
        }
        int i2cWriteBlock(int handle, uint8_t reg, char* buf, unsigned n) {
            return i2cWriteI2CBlockData(handle, reg, buf, n);
        }
        // --- SPI Functions ---
        int spiOpenBus(unsigned channel, unsigned baud, unsigned flags) {
            return spiOpen(channel, baud, flags);
        }
        int spiOpenBus(unsigned channel, unsigned baud, unsigned flags, unsigned cs) {
            gpioSetMode(cs, PI_OUTPUT);
            gpioWrite(cs, 0); 
            flags = flags | 0xE0; // disable hardware cs pins

            int status = spiOpen(0, baud, flags);
            gpioWrite(cs, 1); 

            return status;
        }
        void spiCloseBus(int handle) { if (handle >= 0) spiClose(handle); }
        int spiTransfer(int handle, char* tx, char* rx, unsigned n) {
            return spiXfer(handle, tx, rx, n);
        }
        int spiTransfer(int handle, const char* tx, char* rx, unsigned n, unsigned cs){
            if (cs > 31) return PI_BAD_GPIO;
        
            uint32_t mask = 1u << cs;
        
            gpioWrite_Bits_0_31_Clear(mask);
            //gpioWrite(cs, 0); 
        
            int status = spiXfer(handle, (char*)tx, rx, n);
            gpioWrite_Bits_0_31_Set(mask);
            //gpioWrite(cs, 1); 
        
            return status;
        }
        ~gpio()
        {

            if(serialHandle != -1){
                serClose(serialHandle);
            }
        }
    protected:
        void init_gpio() {
            if (gpioInitialise() < 0) {
                throw std::runtime_error("Failed to initialize pigpio");
            }
        }
        void close_gpio(){
            gpioTerminate();
        }
    private:
        std::vector<int> pwm_GPIO;
        const unsigned int pwm__max = 5000;
        
        int gpioResult;       // init check value 
        int serialHandle = -1;

        char* txData;
        int txSize;
        int rxSize;
        bool pigpio_Init_check()
        {
            if(gpioResult < 0)
            {
                switch(gpioResult)
                {
            
                case PI_INIT_FAILED: {

                    std::cout << "Initialization failed " << std::endl;
                    std::cout << "Error = " << gpioResult << std::endl;

                    return false;
                    break;
                }
                case PI_SER_OPEN_FAILED: {

                    std::cout << "Serial initialization failed " << std::endl;
                    std::cout << "Error = " << gpioResult << std::endl;

                    return false;
                    break;
                }
                case PI_NO_HANDLE: {

                    std::cout << "No serial handle " << std::endl;
                    std::cout << "Error = " << gpioResult << std::endl;

                    return false;
                    break;
                }
                case PI_BAD_GPIO: {

                    std::cout << "GPIO pin is bad " << std::endl;

                    return false;
                    break;
                }
                case PI_BAD_MODE: {

                    std::cout << "Bad mode for GPIO pin " << std::endl;

                    return false;
                    break;
                }
                case PI_BAD_DUTYCYCLE: {
                    
                    std::cout << "Bad duty cycle" << std::endl;
                    return false;
                }
                default: {

                    std::cout << "Unexpected error for GPIO pin " << std::endl;
                    std::cout << "Result = " << gpioResult << std::endl;
                    std::cout << PI_INIT_FAILED << std::endl;

                    return false;
                }
                }
            }

            return true;
        }
        int init_pin_out();
        void Serial_ISR(char* rxData, int rxSize);

};
