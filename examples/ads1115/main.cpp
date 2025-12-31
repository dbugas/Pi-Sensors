#include <iostream>
#include <pigpio.h>
#include <unistd.h>
#include <chrono>

#define ADS1115_ADDR 0x48 // Default I2C address for ADS1115
#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG 0x01
#define ADS1115_FULL_SCALE 4.096 // Full-scale voltage range
#define ADS1115_RESOLUTION 32768.0 // 16-bit resolution (signed)

// pga +/- 6.144 V | +/- 4.096 V | +/- 2.048 V | +/- 1.024 V | +/- 0.512 V | +/- 0.256 V
//         0x0000  |     0x0200  |     0x0400  |     0x0600  |     0x0800  |     0x0A00 
// Acceptable voltage at analog pins is VDD + 0.3V and GND - 0.3V

class ADS1115 {
public:
    ADS1115(int address = ADS1115_ADDR, int dataRate_ = 128) : address_(address), handle_(-1), dataRate(dataRate_) {

        std::cout << "here\n";
        handle_ = i2cOpen(bus_, address_, 0);
        if (handle_ < 0) {
            gpioTerminate();
            throw std::runtime_error("Failed to open I2C device!");
        }
        std::cout << "here\n";
        // Calibrate ADS channel 1
        int counter = 0;
        double V_avg = 0.0;
        configure(1);
        while(counter < 100){

            if(ADSready()){

                V_avg += readVoltage();
                configure(1);
                counter ++;
            }
        }
        Cal_A1 = V_avg/(double)counter;
    }
    
    ~ADS1115() {
        if (handle_ >= 0) {
            i2cClose(handle_);
        }
        std::cout << "ADS1115 cleanup completed." << std::endl;
    }
    
    void configure(int channel) {
        if (channel < 0 || channel > 3) {
            throw std::invalid_argument("Invalid channel number! Must be 0-3.");
        }
    
        // Start single-shot conversion with appropriate MUX and data rate
        // config = 0x8000 | channel | pga (+/- 4.096 V) | single shot mode | data rate | disable comparator 
        uint16_t config = 0x8000 | (channel << 12) | 0x0200 | 0x0100 | getDataRateBits(dataRate) | 0x0003;
        char config_data[3] = {
            ADS1115_REG_CONFIG,
            static_cast<char>(config >> 8),
            static_cast<char>(config & 0xFF)
        };
    
        if (i2cWriteDevice(handle_, config_data, 3) < 0) {
            throw std::runtime_error("Failed to write config!");
        }
    
    }
    bool ADSready(){
        char reg = ADS1115_REG_CONFIG;
        char read_buf[2];
        if (i2cWriteDevice(handle_, &reg, 1) < 0 || i2cReadDevice(handle_, read_buf, 2) < 0) {
            throw std::runtime_error("Failed to read config register!");
        }
        uint16_t config_read = (read_buf[0] << 8) | read_buf[1];

        return config_read & 0x8000;  //Bit 15 == 1 => conversion complete
    }
    double readVoltage() {
        char reg = ADS1115_REG_CONVERSION;
        char data[2] = {0};
        if (i2cWriteDevice(handle_, &reg, 1) < 0 || i2cReadDevice(handle_, data, 2) < 0) {
            throw std::runtime_error("Failed to read conversion result!");
        }
        int16_t result = (data[0] << 8) | data[1];
        return (result / ADS1115_RESOLUTION) * ADS1115_FULL_SCALE;
    }
    double GetBatteryVoltage() {
        double V = readChannel(0);
        V = 18.0846*V + 0.5882;
        return V;
    }
    double GetBatteryAmps() {
        double A = readChannel(1) - Cal_A1;
        A = -12.6083*A;
        return A;
    }

    double readChannel(int channel) {
        return readVoltage();
    }
    
private:
    uint16_t getDataRateBits(int dataRate) {
        switch (dataRate) {
            case 8: return (0b000 << 5);
            case 16: return (0b001 << 5);
            case 32: return (0b010 << 5);
            case 64: return (0b011 << 5);
            case 128: return (0b100 << 5);
            case 250: return (0b101 << 5);
            case 475: return (0b110 << 5);
            case 860: return (0b111 << 5);
            default: return (0b100 << 5); // Default to 128 SPS
        }
    }
    int bus_ = 1;
    int address_;
    int handle_;
    int dataRate;

    double Cal_A1 = 0.0;
};

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "pigpio initialisation failed!" << std::endl;
        return 1;
    }

    std::cout << "Starting ADS1115 test program...\n" << std::endl;

    ADS1115* ads = nullptr;

    try {
        // Create ADS1115 object at default address 0x48, 128 SPS
        // Constructor automatically calibrates channel 1 (A1)
        ads = new ADS1115(0x48, 128);

        std::cout << "ADS1115 initialized and calibrated successfully!\n" << std::endl;
        std::cout << "Time\t\tA0 (V)\tA1 (V)\tA2 (V)\tA3 (V)\tBattery (V)\tCurrent (A)\n";
        std::cout << "-----------------------------------------------------------------------------------\n";

        int reading_count = 0;

        while (true) {
            reading_count++;

            // Read all 4 channels
            ads->configure(0);
            while (!ads->ADSready()) usleep(1000);
            double v0 = ads->readVoltage();

            ads->configure(1);
            while (!ads->ADSready()) usleep(1000);
            double v1 = ads->readVoltage();

            ads->configure(2);
            while (!ads->ADSready()) usleep(1000);
            double v2 = ads->readVoltage();

            ads->configure(3);
            while (!ads->ADSready()) usleep(1000);
            double v3 = ads->readVoltage();

            double battery_voltage = ads->GetBatteryVoltage();  // Uses channel 0 with your scaling
            double current_amps    = ads->GetBatteryAmps();     // Uses channel 1 with calibration

            std::cout << reading_count << "\t"
                      << v0 << "\t"
                      << v1 << "\t"
                      << v2 << "\t"
                      << v3 << "\t"
                      << battery_voltage << "\t"
                      << current_amps << std::endl;

            usleep(500000);  // ~2 Hz update rate (adjust as needed)
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }

    // Cleanup (will call destructor)
    delete ads;
    gpioTerminate();

    return 0;
}
// g++ -Wall -o ads1115_test main.cpp -lpigpio -lrt