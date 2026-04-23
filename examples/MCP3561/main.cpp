#include <pigpio.h>
#include <iostream>
#include <cstdint>
#include <unistd.h> // usleep
#include <iomanip>

// ==========================
// MCP3561 command builder
// ==========================

// Command types
enum CommandType : uint8_t
{
    FAST_COMMAND      = 0b00,
    STATIC_READ       = 0b01,
    INCREMENTAL_WRITE = 0b10,
    INCREMENTAL_READ  = 0b11
};

// Fast commands
enum FastCommand
{
    RESET = 0x6
};

// Register addresses (partial)
enum Register
{
    CONFIG0 = 0x1,
    CONFIG1 = 0x2,
    CONFIG2 = 0x3,
    CONFIG3 = 0x4,
    MULTIPLEXER = 0x6

};

// Read 24-bit ADC data + STATUS and convert to voltage
double readVoltage(int spi_handle, uint8_t dev_addr_bits, double vref, uint8_t gain)
{
    uint8_t cmd = dev_addr_bits | ((0x00 << 2) | CommandType::INCREMENTAL_READ);  // Incremental Read of ADC_DATA (0x00)

    char tx[4] = {(char)cmd, 0x00, 0x00, 0x00};
    char rx[4] = {0};

    spiXfer(spi_handle, tx, rx, 4);

    // rx[0] = STATUS byte
    // rx[1..3] = 24-bit signed ADC code (MSB first)
    int32_t adc_code = ((int32_t)(unsigned char)rx[1] << 16) |
                       ((int32_t)(unsigned char)rx[2] << 8)  |
                       (unsigned char)rx[3];

    // Convert from 24-bit signed to signed 32-bit (sign extend)
    if (adc_code & 0x800000) {
        adc_code |= 0xFF000000;   // Sign extend negative values
    }

    // Voltage calculation formula
    // V = (adc_code / 2^23) * (Vref / Gain)
    double voltage = (adc_code / 8388608.0) * (vref / gain);

    std::cout << "STATUS=0x" << std::hex << std::uppercase << std::setw(2) << (unsigned)(unsigned char)rx[0]
              << "  ADC_CODE=0x" << std::setw(6) << std::setfill('0') << (adc_code & 0xFFFFFF)
              << std::dec << "  Voltage = " << std::fixed << std::setprecision(6) << voltage << " V\n";

    return voltage;
}

uint8_t buildCommand(uint8_t cmdType, uint8_t addr, uint8_t devAddr = 0)
{
    return ((cmdType & 0x3) << 6) | ((addr & 0xF) << 2) | (devAddr & 0x3);
}

void rwRegister(int spi_handle, uint8_t dev_addr_bits, uint8_t reg, CommandType cmdType, uint8_t data = 0x00)
{
    uint8_t base_cmd = (reg << 2) | (uint8_t)cmdType;
    uint8_t cmd      = dev_addr_bits | base_cmd;

    if (cmdType == CommandType::INCREMENTAL_WRITE)
    {
        // Write: send Command + 1 data byte
        char tx[2] = {(char)cmd, (char)data};
        char rx[2] = {0};
        spiXfer(spi_handle, tx, rx, 2);

        std::cout << "Write DA=" << (dev_addr_bits >> 6)
                  << " Cmd=0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << (int)cmd
                  << "  Value=0x" << std::setw(2) << (int)data
                  << "  -> STATUS=0x" << std::setw(2) << (unsigned)(unsigned char)rx[0] << "\n";
    }
    else
    {
        // Read: send Command + 2 dummy bytes (or more if you want)
        char tx[3] = {(char)cmd, 0x00};
        char rx[3] = {0};
        spiXfer(spi_handle, tx, rx, 2);

        std::cout << "Read  DA=" << (dev_addr_bits >> 6)
                  << " (0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(2) << (int)dev_addr_bits << ") | "
                  << "Cmd=0x" << std::setw(2) << (int)cmd 
                  << " -> STATUS=0x" << std::setw(2) << (unsigned)(unsigned char)rx[0]
                  << "  Reg=0x" << std::setw(2) << (int)reg 
                  << "=0x" << std::setw(2) << (unsigned)(unsigned char)rx[1] << "\n";
    }

}

int main()
{
    // ==========================
    // Initialize pigpio
    // ==========================
    if (gpioInitialise() < 0)
    {
        std::cerr << "pigpio init failed\n";
        return -1;
    }

    // ==========================
    // SPI1 setup
    // ==========================
    uint32_t spiFlags = (0 | (1 << 8)); // mode 0 + SPI1 enable
    int spi_handle = spiOpen(2, 6000000, spiFlags); // channel 2 = SPI1 CE0

    if (spi_handle < 0)
    {
        std::cerr << "SPI open failed\n";
        gpioTerminate();
        return -1;
    }

    // ==========================
    // 2. Read CONFIG0 register
    // ==========================

    uint8_t dev_addr_bits = 0x01 << 6;                // DA1:DA0 in bits 7:6
    uint8_t data = 0b01100011;
    rwRegister(spi_handle, dev_addr_bits, Register::CONFIG0, CommandType::INCREMENTAL_WRITE, data);
    usleep(5000); 
    data = 0x0C;
    rwRegister(spi_handle, dev_addr_bits, Register::CONFIG1, CommandType::INCREMENTAL_WRITE, data);
    // config2 default, gain = 1
    usleep(5000); 
    data = 0x80;
    rwRegister(spi_handle, dev_addr_bits, Register::CONFIG3, CommandType::INCREMENTAL_WRITE, data);
    usleep(5000); 
    data = 0b00001000;
    rwRegister(spi_handle, dev_addr_bits, Register::MULTIPLEXER, CommandType::INCREMENTAL_WRITE, data);
    
    std::cout << "\n=== Starting continuous voltage reading ===\n";
    std::cout << "Vref = 2.5V, Gain = 1x\n\n";

    readVoltage(spi_handle, dev_addr_bits, 2.5, 1);

    // ==========================
    // Cleanup
    // ==========================
    spiClose(spi_handle);
    gpioTerminate();

    return 0;
}
// g++ -Wall -o main main.cpp -lpigpio -lrt 