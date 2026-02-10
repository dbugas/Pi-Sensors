#include "PCA9685.h"
#include <pigpio.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

// Constructor
PCA9685::PCA9685(uint8_t i2c_bus, uint8_t addr) : i2c_bus_(i2c_bus), i2c_addr_(addr) {
    // Validate I2C bus and address
    if (i2c_bus > 1) {
        throw std::runtime_error("Invalid I2C bus: " + std::to_string(i2c_bus));
    }
    if (addr < 0x40 || addr > 0x7F) {
        throw std::runtime_error("Invalid I2C address: 0x" + std::to_string(addr));
    }

    // Initialize pigpio
    if (gpioInitialise() < 0) {
        throw std::runtime_error("Failed to initialize pigpio");
    }

    // Open I2C device
    i2c_handle_ = i2cOpen(i2c_bus, i2c_addr_, 0);
    if (i2c_handle_ < 0) {
        gpioTerminate();
        throw std::runtime_error("Failed to open I2C device at address 0x" + 
                                std::to_string(i2c_addr_));
    }

#ifdef ENABLE_DEBUG_OUTPUT
    std::cout << "PCA9685 initialized at address 0x" << std::hex << static_cast<int>(i2c_addr_)
              << std::dec << std::endl;
#endif
}

// Destructor
PCA9685::~PCA9685() {
    try {
        write8(PCA9685_MODE1, MODE1_SLEEP); // Put PCA9685 in sleep mode
    } catch (...) {
        // Ignore errors during cleanup
    }
    i2cClose(i2c_handle_);
    gpioTerminate();

#ifdef ENABLE_DEBUG_OUTPUT
    std::cout << "PCA9685 destructor called" << std::endl;
#endif
}

// Initialize the PCA9685
bool PCA9685::begin(uint8_t prescale) {
    setOscillatorFrequency(FREQUENCY_OSCILLATOR);
    if (prescale) {
        setExtClk(prescale);
    } else {
        setPWMFreq(100.0f); // Default to 100 Hz for LEDs
    }
    // Explicitly ensure sleep mode is off and auto-increment is on
    write8(PCA9685_MODE1, MODE1_AI);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    #ifdef ENABLE_DEBUG_OUTPUT
        std::cout << "After begin, MODE1 = 0x" << std::hex << static_cast<int>(read8(PCA9685_MODE1))
                  << std::dec << std::endl;
    #endif
    return true;
}

// Send a reset command to the PCA9685
void PCA9685::reset() {
    // Open general call address (0x00) for software reset
    int general_call_handle = i2cOpen(i2c_bus_, 0x00, 0);
    if (general_call_handle < 0) {
        throw std::runtime_error("Failed to open I2C general call address 0x00");
    }

    // Send software reset command (0x06) to general call address
    if (i2cWriteByte(general_call_handle, 0x06) < 0) {
        i2cClose(general_call_handle);
        throw std::runtime_error("Failed to send software reset command");
    }
    i2cClose(general_call_handle);

    // Wait for reset to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Verify MODE1 is in default state (0x11)
    uint8_t mode1 = read8(PCA9685_MODE1);
#ifdef ENABLE_DEBUG_OUTPUT
    std::cout << "After software reset, MODE1 = 0x" << std::hex << static_cast<int>(mode1)
              << std::dec << std::endl;
#endif
    if (mode1 != 0x11) {
        std::cerr << "Warning: MODE1 expected 0x11 after reset, got 0x" 
                  << std::hex << static_cast<int>(mode1) << std::dec << std::endl;
    }

    // Explicitly clear MODE1 to 0x00
    write8(PCA9685_MODE1, 0x00);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    #ifdef ENABLE_DEBUG_OUTPUT
        std::cout << "After clearing MODE1, MODE1 = 0x" << std::hex << static_cast<int>(read8(PCA9685_MODE1))
                  << std::dec << std::endl;
    #endif
}

// Put PCA9685 into sleep mode
void PCA9685::sleep() {
    uint8_t awake = read8(PCA9685_MODE1);
    uint8_t sleep = awake | MODE1_SLEEP;
    write8(PCA9685_MODE1, sleep);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    #ifdef ENABLE_DEBUG_OUTPUT
        std::cout << "After sleep, MODE1 = 0x" << std::hex << static_cast<int>(read8(PCA9685_MODE1))
                  << std::dec << std::endl;
    #endif
}

// Wake PCA9685 from sleep
void PCA9685::wakeup() {
    uint8_t sleep = read8(PCA9685_MODE1);
    uint8_t wakeup = (sleep & ~MODE1_SLEEP) | MODE1_AI; // Ensure AI is on
    write8(PCA9685_MODE1, wakeup);

    #ifdef ENABLE_DEBUG_OUTPUT
        std::cout << "After wakeup, MODE1 = 0x" << std::hex << static_cast<int>(read8(PCA9685_MODE1))
                  << std::dec << std::endl;
    #endif
}

// Set external clock
void PCA9685::setExtClk(uint8_t prescale) {
    uint8_t oldmode = read8(PCA9685_MODE1);
    uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP;
    write8(PCA9685_MODE1, newmode); // Go to sleep
    write8(PCA9685_MODE1, newmode | MODE1_EXTCLK); // Enable external clock
    write8(PCA9685_PRESCALE, prescale); // Set prescaler
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    write8(PCA9685_MODE1, (newmode & ~MODE1_SLEEP) | MODE1_AI); // Wake with AI

    #ifdef ENABLE_DEBUG_OUTPUT
        std::cout << "After setExtClk, MODE1 = 0x" << std::hex << static_cast<int>(read8(PCA9685_MODE1))
                  << std::dec << std::endl;
    #endif
}

// Set PWM frequency
void PCA9685::setPWMFreq(float freq) {
#ifdef ENABLE_DEBUG_OUTPUT
    std::cout << "Attempting to set freq " << freq << std::endl;
#endif
    if (freq < 24.0f) {
        freq = 24.0f; // PCA9685 minimum
    }
    if (freq > 1526.0f) {
        freq = 1526.0f; // PCA9685 maximum
    }

    float prescaleval = ((oscillator_freq_ / (freq * 4096.0f)) + 0.5f) - 1.0f;
    if (prescaleval < PCA9685_PRESCALE_MIN) {
        prescaleval = PCA9685_PRESCALE_MIN;
    }
    if (prescaleval > PCA9685_PRESCALE_MAX) {
        prescaleval = PCA9685_PRESCALE_MAX;
    }
    uint8_t prescale = static_cast<uint8_t>(prescaleval);

    #ifdef ENABLE_DEBUG_OUTPUT
        std::cout << "Final pre-scale: " << static_cast<int>(prescale) << std::endl;
    #endif
    uint8_t oldmode = read8(PCA9685_MODE1);
    uint8_t newmode = (oldmode & ~(MODE1_RESTART | MODE1_EXTCLK)) | MODE1_SLEEP;
    write8(PCA9685_MODE1, newmode); // Go to sleep
    write8(PCA9685_PRESCALE, prescale); // Set prescaler
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    write8(PCA9685_MODE1, MODE1_AI); // Wake with only AI bit set

    #ifdef ENABLE_DEBUG_OUTPUT
        std::cout << "After setPWMFreq, MODE1 = 0x" << std::hex << static_cast<int>(read8(PCA9685_MODE1))
                  << std::dec << std::endl;
    #endif
}

// Set output mode (totem pole or open drain)
void PCA9685::setOutputMode(bool totempole) {
    uint8_t oldmode = read8(PCA9685_MODE2);
    uint8_t newmode = totempole ? (oldmode | MODE2_OUTDRV) : (oldmode & ~MODE2_OUTDRV);
    write8(PCA9685_MODE2, newmode);

#ifdef ENABLE_DEBUG_OUTPUT
    std::cout << "Setting output mode: " << (totempole ? "totempole" : "open drain")
              << " by setting MODE2 to " << static_cast<int>(newmode) << std::endl;
#endif
}

// Read prescale value
uint8_t PCA9685::readPrescale() {
    return read8(PCA9685_PRESCALE);
}

// Get PWM value for a channel
uint16_t PCA9685::getPWM(uint8_t num, bool off) {
    if (num > 15) {
        throw std::runtime_error("Invalid channel: " + std::to_string(num));
    }
    uint8_t reg = PCA9685_LED0_ON_L + 4 * num + (off ? 2 : 0);
    uint8_t low = read8(reg);
    uint8_t high = read8(reg + 1);
    return (static_cast<uint16_t>(high) << 8) | low;
}

// Set PWM for a channel
uint8_t PCA9685::setPWM(uint8_t num, uint16_t on, uint16_t off) {
    if (num > 15) {
        throw std::runtime_error("Invalid channel: " + std::to_string(num));
    }

#ifdef ENABLE_DEBUG_OUTPUT
    std::cout << "Setting PWM " << static_cast<int>(num) << ": " << on << "->" << off << std::endl;
#endif

    uint8_t reg = static_cast<uint8_t>(PCA9685_LED0_ON_L + 4 * num);
    uint8_t buffer[4] = {
        static_cast<uint8_t>(on),
        static_cast<uint8_t>(on >> 8),
        static_cast<uint8_t>(off),
        static_cast<uint8_t>(off >> 8)
    };

    if (i2cWriteI2CBlockData(i2c_handle_, reg, reinterpret_cast<char *>(buffer), 4) < 0) {
        throw std::runtime_error("Failed to set PWM for channel " + std::to_string(num));
    }
    return 0;
}
void PCA9685::setPWMMultiple(uint8_t start_num, uint8_t count, const uint16_t* on_values, const uint16_t* off_values)
{
    if (start_num >= 16 || count == 0 || count > (16 - start_num)) {
        throw std::runtime_error("Invalid PCA9685 channel range");
    }

    #ifdef ENABLE_DEBUG_OUTPUT
        std::cout << "Setting PWM channels "<< static_cast<int>(start_num) << " to "<< static_cast<int>(start_num + count - 1) << '\n';
    #endif

    const uint8_t reg = static_cast<uint8_t>(PCA9685_LED0_ON_L + 4 * start_num);

    uint8_t buffer[4 * 16];
    uint8_t* p = buffer;

    for (uint8_t i = 0; i < count; ++i) {
        const uint16_t on  = on_values[i];
        const uint16_t off = off_values[i];

        p[0] = static_cast<uint8_t>(on);
        p[1] = static_cast<uint8_t>(on >> 8);
        p[2] = static_cast<uint8_t>(off);
        p[3] = static_cast<uint8_t>(off >> 8);
        p += 4;
    }

    if (i2cWriteI2CBlockData(i2c_handle_, reg, reinterpret_cast<char*>(buffer), count * 4) < 0)
    {
        throw std::runtime_error("PCA9685 I2C write failed");
    }
}

// Set PWM value with inversion option
void PCA9685::setPin(uint8_t num, uint16_t val, bool invert) {
    if (num > 15) {
        throw std::runtime_error("Invalid channel: " + std::to_string(num));
    }
    val = std::min(val, static_cast<uint16_t>(4095));
    if (invert) {
        if (val == 0) {
            setPWM(num, 4096, 0); // Fully on
        } else if (val == 4095) {
            setPWM(num, 0, 4096); // Fully off
        } else {
            setPWM(num, 0, 4095 - val);
        }
    } else {
        if (val == 4095) {
            setPWM(num, 4096, 0); // Fully on
        } else if (val == 0) {
            setPWM(num, 0, 4096); // Fully off
        } else {
            setPWM(num, 0, val);
        }
    }
}

// Set PWM based on microseconds
void PCA9685::writeMicroseconds(uint8_t num, uint16_t microseconds) {
    if (num > 15) {
        throw std::runtime_error("Invalid channel: " + std::to_string(num));
    }

#ifdef ENABLE_DEBUG_OUTPUT
    std::cout << "Setting PWM via microseconds on output " << static_cast<int>(num)
              << ": " << microseconds << std::endl;
#endif

    double pulse = microseconds;
    double pulselength = 1000000.0; // 1,000,000 us per second
    uint16_t prescale = readPrescale() + 1;
    pulselength *= prescale;
    pulselength /= oscillator_freq_;

#ifdef ENABLE_DEBUG_OUTPUT
    std::cout << pulselength << " us per bit" << std::endl;
#endif

    pulse /= pulselength;
    setPWM(num, 0, static_cast<uint16_t>(std::round(pulse)));
}

// Set oscillator frequency
void PCA9685::setOscillatorFrequency(uint32_t freq) {
    oscillator_freq_ = freq;

#ifdef ENABLE_DEBUG_OUTPUT
    std::cout << "Oscillator frequency set to " << oscillator_freq_ << std::endl;
#endif
}

// Get oscillator frequency
uint32_t PCA9685::getOscillatorFrequency() {
    return oscillator_freq_;
}

// Low-level I2C read
uint8_t PCA9685::read8(uint8_t addr) {
    int value = i2cReadByteData(i2c_handle_, addr);
    if (value < 0) {
        throw std::runtime_error("Failed to read from register 0x" + std::to_string(addr));
    }
    return static_cast<uint8_t>(value);
}

// Low-level I2C write
void PCA9685::write8(uint8_t addr, uint8_t data) {
    if (i2cWriteByteData(i2c_handle_, addr, data) < 0) {
        throw std::runtime_error("Failed to write to register 0x" + std::to_string(addr));
    }
}