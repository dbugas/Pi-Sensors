#pragma once
// MCP356x.h — Driver for MCP3561R / MCP3562R / MCP3564R
// Mirrors the IMU class structure: Timer thread, DataBuffer, atomic flags.

#include <pigpio.h>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <memory>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>

#include "timer.h"
#include "DataBuffer.h"
#include <functional>

// =============================================================================
// Data structs
// =============================================================================
struct ADCData {
    double   voltage    = 0.0;   // volts, differential or single-ended
    int32_t  raw_code   = 0;     // signed 24-bit ADC code
    uint64_t timestamp_us = 0;
};

struct ADCChannelData {
    double   ch[8]      = {};    // volts, single-ended CH0–CH7
    uint64_t timestamp_us = 0;
};

// =============================================================================
// Enums
// =============================================================================
enum class ADC_OSR : uint8_t {
    OSR_32    = 0b0000,
    OSR_64    = 0b0001,
    OSR_128   = 0b0010,
    OSR_256   = 0b0011,
    OSR_512   = 0b0100,
    OSR_1024  = 0b0101,
    OSR_2048  = 0b0110,
    OSR_4096  = 0b0111,
    OSR_8192  = 0b1000,
    OSR_16384 = 0b1001,
    OSR_20480 = 0b1010,
    OSR_24576 = 0b1011,
    OSR_40960 = 0b1100,
    OSR_49152 = 0b1101,
    OSR_81920 = 0b1110,
    OSR_98304 = 0b1111,
};

enum class ADC_Gain : uint8_t {
    GAIN_0_33 = 0b000,   // 1/3x
    GAIN_1    = 0b001,   // 1x
    GAIN_2    = 0b010,   // 2x
    GAIN_4    = 0b011,   // 4x
    GAIN_8    = 0b100,   // 8x
    GAIN_16   = 0b101,   // 16x
    GAIN_32   = 0b110,   // 32x
    GAIN_64   = 0b111,   // 64x
};

// BOOST controls the modulator current (CONFIG2 bits [7:6])
// Higher boost = better noise at high data rates, more power consumption
// Lower boost  = lower power, suitable for slow/precision measurements
enum class ADC_Boost : uint8_t {
    BOOST_0_5 = 0b00,   // 0.5x — lowest power, use with OSR >= 4096
    BOOST_0_66= 0b01,   // 0.66x
    BOOST_1   = 0b10,   // 1x   — default, balanced
    BOOST_2   = 0b11,   // 2x   — best noise at high data rates (OSR <= 256)
};

// Performance mode presets — mirrors IMU PerformanceMode
// Settings chosen to match each use case out of the box:
//
//   Speed     : OSR=32,    Gain=1x,  Boost=2x,   AZ_MUX=off → 12800 sps, 16-bit ENOB
//               Best for: fast transient capture, vibration analysis, audio
//
//   Default   : OSR=1024,  Gain=1x,  Boost=1x,   AZ_MUX=off → 600 sps,   24-bit ENOB
//               Best for: general purpose voltage measurement, multi-channel
//
//   Precision : OSR=16384, Gain=16x, Boost=0.5x, AZ_MUX=on  → 35 sps,    24-bit ENOB
//               Best for: load cells, strain gauges, precision DC measurement
enum class ADC_Mode {
    Speed,
    Default,
    Precision,
};

// =============================================================================
// MCP356x class
// =============================================================================
class MCP356x
{
public:
    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------
    // spiChannel : AUX SPI channel (0=GPIO18, 1=GPIO17, 2=GPIO16)
    // devAddr    : hard-coded device address 0–3 (default 1)
    // vref       : reference voltage in volts (e.g. 2.5 for MCP1501-25)
    // osr        : oversampling ratio enum
    // gain       : programmable gain enum
    // spiBaud    : SPI clock rate in Hz (default 6 MHz)
    // Full constructor — explicit settings
    MCP356x(int        spiChannel,
            uint8_t    devAddr,
            double     vref,
            ADC_OSR    osr     = ADC_OSR::OSR_16384,
            ADC_Gain   gain    = ADC_Gain::GAIN_16,
            unsigned   spiBaud = 6'000'000);

    // Performance mode constructor — settings chosen automatically
    MCP356x(int        spiChannel,
            uint8_t    devAddr,
            double     vref,
            ADC_Mode   mode,
            unsigned   spiBaud = 6'000'000);

    ~MCP356x();

    MCP356x(const MCP356x&)            = delete;
    MCP356x& operator=(const MCP356x&) = delete;

    // -------------------------------------------------------------------------
    // Initialisation — must call before any read or thread start
    // -------------------------------------------------------------------------
    void begin();

    // -------------------------------------------------------------------------
    // One-shot reads (blocking, call from main thread when not using thread)
    // -------------------------------------------------------------------------
    double  readChannel(uint8_t ch);
    double  readDifferential(uint8_t chPos, uint8_t chNeg);
    double  readTemperature();
    int32_t readRaw(uint8_t muxByte);

    // -------------------------------------------------------------------------
    // Continuous mode thread — mirrors IMU start/stop_sensor_thread()
    // -------------------------------------------------------------------------
    // Starts a SCHED_FIFO 99 thread that reads the configured channel(s)
    // at the ADC data rate and writes results to the DataBuffer.
    // Call configureContinuous() before start to set which channel to read.
    void configureContinuous(uint8_t chPos, uint8_t chNeg);  // differential
    void configureContinuous(uint8_t ch);                     // single-ended

    void start_sensor_thread();
    void stop_sensor_thread();

    // -------------------------------------------------------------------------
    // DataBuffer accessors — mirrors IMU get_latest / get_latest_and_consume
    // -------------------------------------------------------------------------
    const ADCData* latest_adc() const noexcept {
        return adc_buffer_.get_latest();
    }

    ADCData get_latest_adc() const noexcept {
        return adc_buffer_.get_latest_copy();
    }

    bool get_latest_adc_and_consume(ADCData& out) noexcept {
        if (!has_new_adc_.exchange(false, std::memory_order_acquire))
            return false;
        const auto* latest = adc_buffer_.get_latest();
        if (latest) { out = *latest; return true; }
        return false;
    }

    // -------------------------------------------------------------------------
    // Callback — fired in the ADC thread immediately after new data is written
    // to the DataBuffer. Use for filtering, logging, or signalling.
    //
    // The callback receives a const reference to the latest ADCData.
    // IMPORTANT: the callback runs in the ADC thread (SCHED_FIFO 99) so it
    // must be non-blocking and complete well within one ADC period.
    // Do not call readChannel(), readDifferential(), or any blocking SPI
    // operation from inside the callback.
    using DataCallback = std::move_only_function<void(const ADCData&)>;
    void setCallback(DataCallback cb) {
        callback_ = std::move(cb);
    }
    void clearCallback() { callback_ = nullptr; }

    // -------------------------------------------------------------------------
    // Configuration (call before begin())
    // -------------------------------------------------------------------------
    void setOSR(ADC_OSR osr);
    void setGain(ADC_Gain gain);

    // BOOST — modulator current setting (CONFIG2[7:6])
    // Use BOOST_2 for OSR <= 256 (high speed), BOOST_0_5 for OSR >= 4096 (low power)
    // Default: BOOST_1
    void setBoost(ADC_Boost boost);

    // Auto-zero MUX (AZ_MUX) — CONFIG2 bit 2
    // When enabled, each conversion takes two samples with inverted inputs
    // and averages them, cancelling the ADC's own offset error automatically.
    // Halves the effective data rate. Recommended for precision DC measurements.
    // Not compatible with continuous conversion mode.
    // Default: disabled
    void setAutoZeroMux(bool enable);

    // -------------------------------------------------------------------------
    // Calibration
    // -------------------------------------------------------------------------
    void setOffsetCal(int32_t code);
    void setGainCal(uint32_t code);
    void enableCalibration(bool offsetCal, bool gainCal);

    // -------------------------------------------------------------------------
    // Utility
    // -------------------------------------------------------------------------
    // Returns the ADC data rate in Hz based on current OSR and internal clock
    double dataRateHz() const;

    // Returns the period in microseconds between ADC samples
    uint32_t periodUs() const;

private:
    // SPI config
    int        spiChannel_;
    uint8_t    devAddr_;
    double     vref_;
    ADC_OSR    osr_;
    ADC_Gain   gain_;
    unsigned   spiBaud_;
    int        spiHandle_ = -1;

    // Cached config register values
    uint8_t    config0_ = 0x62;  // ext vref, int clk, standby
    uint8_t    config1_ = 0x0C;  // OSR — updated from osr_ enum
    uint8_t    config2_ = 0x8B;  // BOOST=1x, GAIN — updated from gain_ enum
    uint8_t    config3_ = 0x80;  // one-shot→standby, 24-bit

    ADC_Boost  boost_    = ADC_Boost::BOOST_1;
    bool       az_mux_   = false;

    // Continuous mode MUX config
    uint8_t    cont_mux_  = 0x08;  // default CH0 vs AGND

    // Temperature sensor constants (datasheet Eq. 5-1)
    static constexpr double TEMP_VREF        = 2.4;
    static constexpr double TEMP_SENSITIVITY = 395e-6;
    static constexpr double TEMP_OFFSET      = 267.7;

    // Internal MCLK (typical)
    static constexpr double MCLK_HZ = 4.9152e6;

    static constexpr int TIMEOUT_MS = 50;

    // DataBuffer — 3 slots, single producer (ADC thread), multiple consumers
    DataBuffer<ADCData, 3> adc_buffer_;
    std::atomic<bool>      has_new_adc_{false};

    // User callback — called in ADC thread after each new sample
    DataCallback           callback_ = nullptr;

    // Thread
    std::atomic<bool> running_{false};
    std::thread       sensor_thread_;
    std::unique_ptr<Timer> adc_timer_;

    // ---- SPI primitives ----
    uint8_t  cmdByte(uint8_t regOrFc, uint8_t type) const;
    void     writeReg(uint8_t reg, uint8_t val);
    void     writeReg24(uint8_t reg, uint32_t val);
    uint8_t  readReg8(uint8_t reg);
    uint32_t readADC();
    void     fastCmd(uint8_t fc);

    // ---- Conversion ----
    int32_t  convert(uint8_t muxByte);
    bool     waitReady();
    int32_t  averageRaw(uint8_t muxByte, int n);
    void     calibrate();

    // ---- Thread loop ----
    void     updateADC();

    // ---- Helpers ----
    static uint8_t osrBits(ADC_OSR osr)     { return static_cast<uint8_t>(osr); }
    static uint8_t gainBits(ADC_Gain gain)   { return static_cast<uint8_t>(gain); }
    static uint8_t boostBits(ADC_Boost boost){ return static_cast<uint8_t>(boost); }
    static double  gainValue(ADC_Gain gain);
};