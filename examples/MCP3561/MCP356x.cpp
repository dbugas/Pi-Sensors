// MCP356x.cpp — Implementation

#include "MCP356x.h"
#include <unistd.h>
#include <cmath>
#include <map>

// =============================================================================
// Register addresses
// =============================================================================
static constexpr uint8_t REG_ADCDATA = 0x0;
static constexpr uint8_t REG_CONFIG0 = 0x1;
static constexpr uint8_t REG_CONFIG1 = 0x2;
static constexpr uint8_t REG_CONFIG2 = 0x3;
static constexpr uint8_t REG_CONFIG3 = 0x4;
static constexpr uint8_t REG_IRQ     = 0x5;
static constexpr uint8_t REG_MUX     = 0x6;
static constexpr uint8_t REG_OFFSETCAL = 0x9;
static constexpr uint8_t REG_GAINCAL   = 0xA;

static constexpr uint8_t CMD_STATIC_READ = 0b01;
static constexpr uint8_t CMD_INC_WRITE   = 0b10;
static constexpr uint8_t CMD_FAST        = 0b00;

static constexpr uint8_t FC_START   = 0b1010;
static constexpr uint8_t FC_STANDBY = 0b1011;
static constexpr uint8_t FC_RESET   = 0b1110;

static constexpr uint32_t AUX_SPI_FLAG = (1u << 8);

// =============================================================================
// OSR → value lookup (for data rate calculation)
// =============================================================================
static uint32_t osrValue(ADC_OSR osr)
{
    static const uint32_t table[] = {
        32, 64, 128, 256, 512, 1024, 2048, 4096,
        8192, 16384, 20480, 24576, 40960, 49152, 81920, 98304
    };
    return table[static_cast<uint8_t>(osr) & 0x0F];
}

// =============================================================================
// gainValue lookup
// =============================================================================
double MCP356x::gainValue(ADC_Gain gain)
{
    static const double table[] = {
        1.0/3.0, 1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0
    };
    return table[static_cast<uint8_t>(gain) & 0x07];
}

// =============================================================================
// Construction / destruction
// =============================================================================
MCP356x::MCP356x(int spiChannel, uint8_t devAddr, double vref,
                 ADC_OSR osr, ADC_Gain gain, unsigned spiBaud)
    : spiChannel_(spiChannel)
    , devAddr_(devAddr & 0x03)
    , vref_(vref)
    , osr_(osr)
    , gain_(gain)
    , spiBaud_(spiBaud)
{
    // Bake OSR into CONFIG1[5:2]
    config1_ = (config1_ & 0xC3) | (osrBits(osr_) << 2);
    // Bake BOOST into CONFIG2[7:6] and GAIN into CONFIG2[5:3]
    config2_ = (config2_ & 0x07) | (boostBits(boost_) << 6) | (gainBits(gain_) << 3);
}

// Performance mode constructor
MCP356x::MCP356x(int spiChannel, uint8_t devAddr, double vref,
                 ADC_Mode mode, unsigned spiBaud)
    : spiChannel_(spiChannel)
    , devAddr_(devAddr & 0x03)
    , vref_(vref)
    , spiBaud_(spiBaud)
{
    switch (mode) {
        case ADC_Mode::Speed:
            osr_    = ADC_OSR::OSR_32;
            gain_   = ADC_Gain::GAIN_1;
            boost_  = ADC_Boost::BOOST_2;
            az_mux_ = false;
            break;

        case ADC_Mode::Default:
            osr_    = ADC_OSR::OSR_1024;
            gain_   = ADC_Gain::GAIN_1;
            boost_  = ADC_Boost::BOOST_1;
            az_mux_ = false;
            break;

        case ADC_Mode::Precision:
            osr_    = ADC_OSR::OSR_16384;
            gain_   = ADC_Gain::GAIN_16;
            boost_  = ADC_Boost::BOOST_0_5;
            az_mux_ = true;
            break;
    }

    // Bake settings into config registers
    config1_ = (config1_ & 0xC3) | (osrBits(osr_) << 2);
    config2_ = (config2_ & 0x07)
             | (boostBits(boost_) << 6)
             | (gainBits(gain_)   << 3)
             | (az_mux_ ? (1u << 2) : 0u);
}

MCP356x::~MCP356x()
{
    stop_sensor_thread();

    if (adc_timer_) adc_timer_->stop();

    if (spiHandle_ >= 0) {
        fastCmd(FC_STANDBY);
        spiClose(spiHandle_);
        spiHandle_ = -1;
    }
}

// =============================================================================
// begin()
// =============================================================================
void MCP356x::begin()
{
    if (spiHandle_ >= 0) {
        spiClose(spiHandle_);
        spiHandle_ = -1;
    }

    spiHandle_ = spiOpen(spiChannel_, spiBaud_, AUX_SPI_FLAG);
    if (spiHandle_ < 0)
        throw std::runtime_error("MCP356x: spiOpen failed (handle="
                                 + std::to_string(spiHandle_) + ")");

    fastCmd(FC_RESET);
    usleep(10'000);

    writeReg(REG_CONFIG0, config0_);
    writeReg(REG_CONFIG1, config1_);
    writeReg(REG_CONFIG2, config2_);
    writeReg(REG_CONFIG3, config3_);

    uint8_t rb = readReg8(REG_CONFIG0);
    if (rb != config0_)
        throw std::runtime_error("MCP356x: CONFIG0 readback mismatch");

    calibrate();
}

// =============================================================================
// Utility
// =============================================================================
double MCP356x::dataRateHz() const
{
    return MCLK_HZ / (4.0 * osrValue(osr_));
}

uint32_t MCP356x::periodUs() const
{
    return static_cast<uint32_t>(1e6 / dataRateHz());
}

// =============================================================================
// Configuration setters
// =============================================================================
void MCP356x::setOSR(ADC_OSR osr)
{
    osr_     = osr;
    config1_ = (config1_ & 0xC3) | (osrBits(osr_) << 2);
}

void MCP356x::setGain(ADC_Gain gain)
{
    gain_    = gain;
    config2_ = (config2_ & 0xC7) | (gainBits(gain_) << 3);
}

void MCP356x::setBoost(ADC_Boost boost)
{
    boost_   = boost;
    config2_ = (config2_ & 0x3F) | (boostBits(boost_) << 6);
}

void MCP356x::setAutoZeroMux(bool enable)
{
    // AZ_MUX is CONFIG2 bit 2
    // Note: AZ_MUX is not compatible with continuous conversion mode —
    // it is silently ignored in continuous mode per the datasheet.
    az_mux_ = enable;
    if (enable)
        config2_ |=  (1u << 2);
    else
        config2_ &= ~(1u << 2);

    // When AZ_MUX is enabled, conversion time doubles so we also
    // flag this for waitReady() to use 2x the normal timeout.
    // The IRQ polling handles this automatically since we wait for
    // DR_STATUS rather than a fixed time, but the initial sleep
    // in waitReady() needs to be doubled.
}

// =============================================================================
// Continuous mode configuration
// =============================================================================
void MCP356x::configureContinuous(uint8_t chPos, uint8_t chNeg)
{
    if (chPos > 7) throw std::invalid_argument("MCP356x: chPos must be 0–7");
    if (chNeg > 7) throw std::invalid_argument("MCP356x: chNeg must be 0–7");
    cont_mux_ = (uint8_t)((chPos << 4) | chNeg);
}

void MCP356x::configureContinuous(uint8_t ch)
{
    if (ch > 7) throw std::invalid_argument("MCP356x: channel must be 0–7");
    cont_mux_ = (uint8_t)((ch << 4) | 0x08);  // CHn vs AGND
}

// =============================================================================
// Thread management — mirrors IMU start/stop_sensor_thread()
// =============================================================================
void MCP356x::start_sensor_thread()
{
    if (running_) return;

    // Timer fires at the ADC data rate
    adc_timer_ = std::make_unique<Timer>(std::chrono::microseconds(periodUs()));

    running_ = true;

    try {
        sensor_thread_ = std::thread([this] { updateADC(); });
    } catch (const std::exception& e) {
        std::cerr << "MCP356x: failed to start thread: " << e.what() << "\n";
        running_ = false;
    }
}

void MCP356x::stop_sensor_thread()
{
    if (!running_) return;

    running_ = false;

    if (adc_timer_) adc_timer_->stop();

    if (sensor_thread_.joinable())
        sensor_thread_.join();
}

// =============================================================================
// updateADC() — thread loop, mirrors update10DOF()
// =============================================================================
void MCP356x::updateADC()
{
#ifdef __linux__
    sched_param param{};
    param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
#endif

    adc_timer_->start(true);

    while (running_) {
        if (!adc_timer_->check()) continue;

        // Trigger conversion and read
        writeReg(REG_MUX, cont_mux_);
        fastCmd(FC_START);

        if (!waitReady()) continue;  // timeout — skip this sample

        uint32_t raw24 = readADC();
        int32_t  code  = static_cast<int32_t>(raw24);
        if (raw24 & 0x800000u) code |= static_cast<int32_t>(0xFF000000u);

        // Write to DataBuffer
        ADCData* slot    = adc_buffer_.prepare_write();
        slot->raw_code   = code;
        slot->voltage    = (static_cast<double>(code) / 8388608.0) * vref_;
        slot->timestamp_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());
        adc_buffer_.commit();
        has_new_adc_.store(true, std::memory_order_release);

        // Fire user callback with the freshly written data
        if (callback_) {
            try {
                callback_(*adc_buffer_.get_latest());
            } catch (...) {
                // Swallow exceptions — callback must not crash the ADC thread
            }
        }
    }
}

// =============================================================================
// One-shot public reads
// =============================================================================
double MCP356x::readChannel(uint8_t ch)
{
    if (ch > 7) throw std::invalid_argument("MCP356x: channel must be 0–7");
    int32_t code = convert((uint8_t)((ch << 4) | 0x08));
    return (static_cast<double>(code) / 8388608.0) * vref_;
}

double MCP356x::readDifferential(uint8_t chPos, uint8_t chNeg)
{
    if (chPos > 7) throw std::invalid_argument("MCP356x: chPos must be 0–7");
    if (chNeg > 7) throw std::invalid_argument("MCP356x: chNeg must be 0–7");
    int32_t code = convert((uint8_t)((chPos << 4) | chNeg));
    return (static_cast<double>(code) / 8388608.0) * vref_;
}

double MCP356x::readTemperature()
{
    uint8_t saved  = config0_;
    uint8_t temp_c = (config0_ & 0x3F) | 0x80;
    writeReg(REG_CONFIG0, temp_c);
    int32_t code = convert(0xDE);
    writeReg(REG_CONFIG0, saved);
    double v = (static_cast<double>(code) / 8388608.0) * TEMP_VREF;
    return (v / TEMP_SENSITIVITY) - TEMP_OFFSET;
}

int32_t MCP356x::readRaw(uint8_t muxByte)
{
    return convert(muxByte);
}

// =============================================================================
// Calibration
// =============================================================================
void MCP356x::setOffsetCal(int32_t code)
{
    if (code >  8388607) code =  8388607;
    if (code < -8388608) code = -8388608;
    writeReg24(REG_OFFSETCAL, static_cast<uint32_t>(code) & 0xFFFFFF);
}

void MCP356x::setGainCal(uint32_t code)
{
    if (code > 0xFFFFFF) code = 0xFFFFFF;
    writeReg24(REG_GAINCAL, code);
}

void MCP356x::enableCalibration(bool offsetCal, bool gainCal)
{
    config3_ &= ~0x06;
    if (offsetCal) config3_ |= 0x02;
    if (gainCal)   config3_ |= 0x04;
    writeReg(REG_CONFIG3, config3_);
}

void MCP356x::calibrate()
{
    static constexpr int     CAL_SAMPLES = 32;
    static constexpr ADC_OSR CAL_OSR     = ADC_OSR::OSR_4096;

    uint8_t saved_config1 = config1_;
    config1_ = (config1_ & 0xC3) | (osrBits(CAL_OSR) << 2);
    writeReg(REG_CONFIG1, config1_);

    int32_t offset_code = averageRaw(0x88, CAL_SAMPLES);
    int32_t offsetcal   = -offset_code;
    if (offsetcal >  8388607) offsetcal =  8388607;
    if (offsetcal < -8388608) offsetcal = -8388608;

    int32_t  vref_code = averageRaw(0xB8, CAL_SAMPLES);
    uint32_t gaincal   = 0x800000;
    if (vref_code > 0) {
        double g = (8388607.0 / static_cast<double>(vref_code)) * 8388608.0;
        if (g > 16777215.0) g = 16777215.0;
        gaincal = static_cast<uint32_t>(g);
    }

    writeReg24(REG_OFFSETCAL, static_cast<uint32_t>(offsetcal) & 0xFFFFFF);
    writeReg24(REG_GAINCAL,   gaincal);

    config3_ |= 0x06;
    writeReg(REG_CONFIG3, config3_);

    config1_ = saved_config1;
    writeReg(REG_CONFIG1, config1_);
}

// =============================================================================
// SPI primitives
// =============================================================================
uint8_t MCP356x::cmdByte(uint8_t regOrFc, uint8_t type) const
{
    return (devAddr_ << 6) | ((regOrFc & 0x0F) << 2) | (type & 0x03);
}

void MCP356x::writeReg(uint8_t reg, uint8_t val)
{
    char tx[2] = { (char)cmdByte(reg, CMD_INC_WRITE), (char)val };
    char rx[2] = {};
    spiXfer(spiHandle_, tx, rx, 2);
}

void MCP356x::writeReg24(uint8_t reg, uint32_t val)
{
    char tx[4] = {
        (char)cmdByte(reg, CMD_INC_WRITE),
        (char)((val >> 16) & 0xFF),
        (char)((val >>  8) & 0xFF),
        (char)( val        & 0xFF),
    };
    char rx[4] = {};
    spiXfer(spiHandle_, tx, rx, 4);
}

uint8_t MCP356x::readReg8(uint8_t reg)
{
    char tx[2] = { (char)cmdByte(reg, CMD_STATIC_READ), 0x00 };
    char rx[2] = {};
    spiXfer(spiHandle_, tx, rx, 2);
    return static_cast<uint8_t>(rx[1]);
}

uint32_t MCP356x::readADC()
{
    char tx[4] = { (char)cmdByte(REG_ADCDATA, CMD_STATIC_READ), 0, 0, 0 };
    char rx[4] = {};
    spiXfer(spiHandle_, tx, rx, 4);
    return ((uint32_t)(uint8_t)rx[1] << 16)
         | ((uint32_t)(uint8_t)rx[2] <<  8)
         |  (uint32_t)(uint8_t)rx[3];
}

void MCP356x::fastCmd(uint8_t fc)
{
    char tx = (char)cmdByte(fc, CMD_FAST);
    char rx = 0;
    spiXfer(spiHandle_, &tx, &rx, 1);
}

// =============================================================================
// convert() and waitReady()
// =============================================================================
int32_t MCP356x::convert(uint8_t muxByte)
{
    writeReg(REG_MUX, muxByte);
    fastCmd(FC_START);

    if (!waitReady())
        throw std::runtime_error("MCP356x: conversion timeout");

    uint32_t raw  = readADC();
    int32_t  code = static_cast<int32_t>(raw);
    if (raw & 0x800000u) code |= static_cast<int32_t>(0xFF000000u);
    return code;
}

bool MCP356x::waitReady()
{
    // Sleep for exactly TADC_SETUP + TCONV before polling.
    // Both values are derived from Table 5-6 (TCONV in DMCLK periods)
    // and the timing spec TADC_SETUP = 256 DMCLK periods.
    // DMCLK = MCLK / 4 = 4.9152 MHz / 4 = 1.2288 MHz → period = 813.8 ns
    //
    // We sleep for the exact expected conversion time, then poll in short
    // 50µs intervals as a safety net for internal clock tolerance (±30%).
    // Timeout is 3x the expected conversion time to handle worst-case clock.

    static const uint32_t tconv_us_table[] = {
    //  OSR bits:  0      1      2      3      4      5      6      7
                   287,   365,   521,   833,   1459,  1875,  2709,  4375,
    //             8      9      10     11     12     13     14     15
                   7709,  14375, 17709, 21042, 34375, 41042, 67709, 81042,
    };

    uint8_t  idx      = static_cast<uint8_t>(osr_) & 0x0F;
    uint32_t wait_us  = tconv_us_table[idx];
    uint32_t timeout_us = (az_mux_ ? wait_us * 2 : wait_us) * 3;  // 3x for worst-case ±30% clock

    // Initial sleep — wait out the full expected conversion time.
    // AZ_MUX doubles TCONV since the chip does two conversions.
    uint32_t actual_wait = az_mux_ ? wait_us * 2 : wait_us;
    usleep(actual_wait);

    // Poll in short bursts until data ready or timeout
    uint32_t elapsed_us = wait_us;
    while (elapsed_us < timeout_us) {
        uint8_t irq = readReg8(REG_IRQ);
        if (!((irq >> 6) & 1)) return true;
        usleep(50);
        elapsed_us += 50;
    }
    return false;
}

int32_t MCP356x::averageRaw(uint8_t muxByte, int n)
{
    double sum = 0;
    for (int i = 0; i < n; ++i) {
        sum += convert(muxByte);
        usleep(1'000);
    }
    return static_cast<int32_t>(sum / n);
}