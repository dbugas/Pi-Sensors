// main.cpp — Load cell with callback-driven moving average filter
//
// The ADC thread fires a callback on every new sample at the exact data rate.
// The callback updates the moving average filter and stores the result.
// The main thread reads the filtered weight at its own pace.
//
// Build: g++ -std=c++17 -Wall -O2 -o main main.cpp MCP356x.cpp \
//            -lpigpio -lrt -pthread
// Run:   sudo ./main

#include "MCP356x.h"
#include <iostream>
#include <iomanip>
#include <atomic>
#include <unistd.h>

// ---- Load cell configuration ----
static constexpr uint8_t  CH_POS       = 0;
static constexpr uint8_t  CH_NEG       = 1;
static constexpr double   EXCITATION_V = 3.3;
static constexpr double   CAPACITY_KG  = 10.0;
static constexpr double   KNOWN_WEIGHT = 0.5;
static constexpr int      CAL_SAMPLES  = 64;

// ---- Moving average filter ----
// LEN is chosen to give ~1Hz output bandwidth regardless of ADC rate.
// At Precision mode (35 sps with AZ_MUX), LEN=35 averages over 1 second.
// Adjust to taste: smaller = more responsive, larger = quieter.
struct MovingAvg {
    static constexpr int LEN = 35;
    double buf[LEN] = {};
    int    idx      = 0;
    double sum      = 0.0;
    bool   filled   = false;

    double update(double val) {
        sum     -= buf[idx];
        buf[idx] = val;
        sum     += val;
        idx      = (idx + 1) % LEN;
        if (idx == 0) filled = true;
        int n = filled ? LEN : (idx == 0 ? LEN : idx);
        return sum / n;
    }
    void reset() { *this = MovingAvg{}; }
};

// ---- Calibration helper ----
static double averageReads(MCP356x& adc, int n)
{
    double sum = 0.0;
    for (int i = 0; i < n; ++i) {
        sum += adc.readDifferential(CH_POS, CH_NEG);
        usleep(10'000);
    }
    return sum / n;
}

int main()
{
    if (gpioInitialise() < 0) { std::cerr << "pigpio init failed\n"; return 1; }

    MCP356x adc(2, 0x01, 2.5, ADC_Mode::Precision);

    try { adc.begin(); }
    catch (const std::exception& e) {
        std::cerr << "ADC init failed: " << e.what() << "\n";
        gpioTerminate(); return 1;
    }

    std::cout << "MCP356x ready  ["
              << "rate=" << adc.dataRateHz() << " sps  "
              << "period=" << adc.periodUs() << " µs]\n\n";

    // ---- Tare ----
    std::cout << "Remove all load then press Enter...\n";
    std::cin.get();
    double tare_v = averageReads(adc, CAL_SAMPLES);
    std::cout << "Tare: " << std::fixed << std::setprecision(4)
              << tare_v * 1000.0 << " mV\n\n";

    // ---- Calibrate ----
    std::cout << "Place " << KNOWN_WEIGHT << " kg then press Enter...\n";
    std::cin.get();
    double cal_v     = averageReads(adc, CAL_SAMPLES);
    double net_cal_v = cal_v - tare_v;

    if (std::abs(net_cal_v) < 1e-9) {
        std::cerr << "Calibration failed — signal too small\n";
        gpioTerminate(); return 1;
    }

    double v_per_kg    = net_cal_v / KNOWN_WEIGHT;
    double sensitivity = net_cal_v * 1000.0 / EXCITATION_V;

    std::cout << "Calibration complete\n"
              << "  Sensitivity : " << sensitivity << " mV/V\n"
              << "  Scale factor: " << v_per_kg * 1000.0 << " mV/kg\n\n";

    // ---- Shared state written by callback, read by main ----
    MovingAvg            filter;
    std::atomic<double>  filtered_kg{0.0};
    std::atomic<double>  raw_kg{0.0};
    std::atomic<uint32_t> sample_count{0};

    // ---- Callback — runs in ADC thread at the data rate ----
    // Receives fresh ADCData, updates filter, stores results atomically.
    adc.setCallback([&](const ADCData& data) {
        double net_v  = data.voltage - tare_v;
        double raw     = net_v / v_per_kg;
        double filtered = filter.update(raw);

        // Atomic store — safe to read from main thread without a mutex
        raw_kg.store(raw,      std::memory_order_relaxed);
        filtered_kg.store(filtered, std::memory_order_relaxed);
        sample_count.fetch_add(1,   std::memory_order_relaxed);
    });

    // ---- Start ADC thread ----
    adc.configureContinuous(CH_POS, CH_NEG);
    adc.start_sensor_thread();

    // ---- Main thread: print at 2 Hz ----
    std::cout << std::setw(14) << "Raw (kg)"
              << std::setw(14) << "Filtered (kg)"
              << std::setw(10) << "Samples\n"
              << std::string(38, '-') << "\n";

    for (;;) {
        double   raw      = raw_kg.load(std::memory_order_relaxed);
        double   filtered = filtered_kg.load(std::memory_order_relaxed);
        uint32_t n        = sample_count.load(std::memory_order_relaxed);

        std::cout << std::setw(14) << std::fixed << std::setprecision(4) << raw
                  << std::setw(14) << filtered
                  << std::setw(10) << n << "\n";

        usleep(500'000);  // print at 2 Hz — filter does the work at 35 Hz
    }

    adc.stop_sensor_thread();
    gpioTerminate();
    return 0;
}
// g++ -std=c++17 -Wall -O2 -o main main.cpp MCP356x.cpp -lpigpio -lrt -pthread