/**
 * Embedded Sensor Monitor
 * ========================
 * Simulates a real-time embedded system that reads multi-channel
 * sensor data (temperature, voltage, current), applies moving-average
 * filtering, performs fault detection (threshold + rate-of-change),
 * and logs structured output — all without dynamic memory allocation
 * (embedded-style, fixed-size buffers).
 *
 * Build:
 *   g++ -std=c++17 -O2 -Wall -o sensor_monitor main.cpp
 *
 * Author: Muhammad Hassan
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <ctime>

// ── Configuration ─────────────────────────────────────────────────────────
static constexpr uint8_t  NUM_CHANNELS   = 4;
static constexpr uint16_t BUFFER_SIZE    = 256;   // circular buffer depth per channel
static constexpr uint8_t  MA_WINDOW      = 8;     // moving-average window
static constexpr uint32_t SAMPLE_RATE_HZ = 100;   // simulated sample rate
static constexpr uint16_t TOTAL_SAMPLES  = 500;   // simulation length


// ── Sensor Channel Descriptor ─────────────────────────────────────────────
struct SensorConfig {
    const char* name;
    float       nominal;       // expected value
    float       min_limit;     // lower fault threshold
    float       max_limit;     // upper fault threshold
    float       roc_limit;     // max rate-of-change per sample
    const char* unit;
};

static const SensorConfig SENSORS[NUM_CHANNELS] = {
    {"TEMP_MCU",   75.0f,  0.0f,  105.0f, 2.0f,  "°C" },
    {"VDD_CORE",    1.1f,  0.95f,   1.25f, 0.05f, "V"  },
    {"I_SUPPLY",  500.0f, 0.0f,  1200.0f, 50.0f, "mA" },
    {"V_BUS",      12.0f, 10.8f,   13.2f, 0.3f,  "V"  },
};


// ── Fault Types ────────────────────────────────────────────────────────────
enum class FaultCode : uint8_t {
    NONE         = 0,
    OVER_MAX     = 1,
    UNDER_MIN    = 2,
    HIGH_ROC     = 3,
};

static const char* fault_name(FaultCode f) {
    switch (f) {
        case FaultCode::OVER_MAX:  return "OVER_MAX";
        case FaultCode::UNDER_MIN: return "UNDER_MIN";
        case FaultCode::HIGH_ROC:  return "HIGH_ROC";
        default:                   return "OK";
    }
}


// ── Circular Buffer ────────────────────────────────────────────────────────
template<typename T, uint16_t N>
class RingBuffer {
public:
    RingBuffer() : head_(0), count_(0) { std::memset(buf_, 0, sizeof(buf_)); }

    void push(T val) {
        buf_[head_] = val;
        head_ = (head_ + 1) % N;
        if (count_ < N) ++count_;
    }

    T get(uint16_t ago) const {
        // ago=0 → most recent, wraps correctly within stored count
        if (ago >= count_) return buf_[(head_ + N - 1) % N];
        uint16_t idx = (static_cast<uint32_t>(head_) + N - 1 - ago) % N;
        return buf_[idx];
    }

    uint16_t size() const { return count_; }

private:
    T        buf_[N];
    uint16_t head_;
    uint16_t count_;
};


// ── Moving Average Filter ──────────────────────────────────────────────────
float moving_average(const RingBuffer<float, BUFFER_SIZE>& buf, uint8_t window) {
    uint16_t n = std::min(static_cast<uint16_t>(window), static_cast<uint16_t>(buf.size()));
    if (n == 0) return 0.0f;
    float sum = 0.0f;
    for (uint16_t i = 0; i < n; ++i)
        sum += buf.get(i);
    return sum / static_cast<float>(n);
}


// ── Signal Simulator ───────────────────────────────────────────────────────
/**
 * Simulate a realistic sensor reading:
 *   nominal + slow drift + periodic ripple + Gaussian-like noise
 */
float simulate_reading(const SensorConfig& cfg, uint32_t tick) {
    float t       = static_cast<float>(tick) / SAMPLE_RATE_HZ;
    float drift   = 0.02f * cfg.nominal * std::sin(2.0f * 3.14159f * 0.05f * t);
    float ripple  = 0.01f * cfg.nominal * std::sin(2.0f * 3.14159f * 1.0f  * t);

    // Box-Muller noise
    float u1 = (static_cast<float>(rand()) / RAND_MAX) + 1e-7f;
    float u2 =  static_cast<float>(rand()) / RAND_MAX;
    float noise = std::sqrt(-2.0f * std::log(u1)) * std::cos(2.0f * 3.14159f * u2);
    noise *= 0.005f * cfg.nominal;

    // Inject a spike fault on channel 0 around t=3s
    float spike = 0.0f;
    if (strcmp(cfg.name, "TEMP_MCU") == 0 && tick >= 295 && tick <= 305)
        spike = 35.0f;

    return cfg.nominal + drift + ripple + noise + spike;
}


// ── Fault Detector ─────────────────────────────────────────────────────────
FaultCode detect_fault(const SensorConfig& cfg, float filtered, float prev_filtered) {
    if (filtered > cfg.max_limit)                           return FaultCode::OVER_MAX;
    if (filtered < cfg.min_limit)                           return FaultCode::UNDER_MIN;
    if (std::fabs(filtered - prev_filtered) > cfg.roc_limit) return FaultCode::HIGH_ROC;
    return FaultCode::NONE;
}


// ── Log Entry ──────────────────────────────────────────────────────────────
struct LogEntry {
    uint32_t  tick;
    uint8_t   channel;
    float     raw;
    float     filtered;
    FaultCode fault;
};


// ── Main ───────────────────────────────────────────────────────────────────
int main() {
    srand(42);  // deterministic seed

    RingBuffer<float, BUFFER_SIZE> bufs[NUM_CHANNELS];
    float prev_filtered[NUM_CHANNELS] = {};

    // Open log file
    std::ofstream logfile("sensor_log.csv");
    logfile << "tick,channel,sensor,raw,filtered,fault\n";

    uint32_t fault_counts[NUM_CHANNELS] = {};
    uint32_t total_faults = 0;

    std::cout << "\n╔══════════════════════════════════════════════════════════╗\n";
    std::cout <<   "║          EMBEDDED SENSOR MONITOR — SIMULATION           ║\n";
    std::cout <<   "╚══════════════════════════════════════════════════════════╝\n\n";

    std::cout << std::left
              << std::setw(6)  << "Tick"
              << std::setw(14) << "Sensor"
              << std::setw(10) << "Raw"
              << std::setw(12) << "Filtered"
              << std::setw(12) << "Status"
              << "\n"
              << std::string(54, '-') << "\n";

    for (uint32_t tick = 0; tick < TOTAL_SAMPLES; ++tick) {
        for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
            float raw = simulate_reading(SENSORS[ch], tick);
            bufs[ch].push(raw);

            float filtered = moving_average(bufs[ch], MA_WINDOW);
            // Skip fault detection on very first sample (no prior reference)
            FaultCode fault = (tick == 0)
                ? FaultCode::NONE
                : detect_fault(SENSORS[ch], filtered, prev_filtered[ch]);

            // Log to CSV
            logfile << tick << ","
                    << static_cast<int>(ch) << ","
                    << SENSORS[ch].name << ","
                    << std::fixed << std::setprecision(3) << raw << ","
                    << filtered << ","
                    << fault_name(fault) << "\n";

            // Print to console only at 50-tick intervals or on fault
            if (tick % 50 == 0 || fault != FaultCode::NONE) {
                std::cout << std::left
                          << std::setw(6)  << tick
                          << std::setw(14) << SENSORS[ch].name
                          << std::setw(10) << std::fixed << std::setprecision(2) << raw
                          << std::setw(12) << filtered;
                if (fault != FaultCode::NONE)
                    std::cout << "[!] " << fault_name(fault);
                else
                    std::cout << "OK";
                std::cout << " " << SENSORS[ch].unit << "\n";
            }

            if (fault != FaultCode::NONE) {
                ++fault_counts[ch];
                ++total_faults;
            }

            prev_filtered[ch] = (tick == 0) ? filtered : prev_filtered[ch];
            prev_filtered[ch] = filtered;
        }
    }

    logfile.close();

    // ── Summary ──────────────────────────────────────────────────────────
    std::cout << "\n" << std::string(54, '═') << "\n";
    std::cout << "SIMULATION COMPLETE — " << TOTAL_SAMPLES << " samples @ "
              << SAMPLE_RATE_HZ << " Hz\n";
    std::cout << std::string(54, '─') << "\n";
    std::cout << std::left
              << std::setw(16) << "Channel"
              << std::setw(10) << "Faults"
              << std::setw(10) << "Rate(%)"
              << "\n"
              << std::string(36, '-') << "\n";
    for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
        float rate = 100.0f * fault_counts[ch] / TOTAL_SAMPLES;
        std::cout << std::left
                  << std::setw(16) << SENSORS[ch].name
                  << std::setw(10) << fault_counts[ch]
                  << std::fixed << std::setprecision(1) << rate << "%\n";
    }
    std::cout << std::string(36, '-') << "\n";
    std::cout << "Total faults detected: " << total_faults << "\n";
    std::cout << "Log saved to: sensor_log.csv\n\n";

    return 0;
}
