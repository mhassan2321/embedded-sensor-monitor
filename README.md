# Embedded Sensor Monitor

A C++17 simulation of a real-time embedded sensor monitoring system. Demonstrates embedded software patterns: fixed-size ring buffers, moving-average filtering, multi-channel fault detection, and structured CSV logging — all without dynamic memory allocation.

## Features

- **4-channel sensor simulation** — MCU temperature, core voltage, supply current, bus voltage
- **Ring buffer** (template class, fixed-size, no heap allocation)
- **Moving-average filter** (configurable window size)
- **Fault detection** — over/under threshold and rate-of-change alarms
- **Injected fault scenario** — temperature spike at t≈3s for validation
- **CSV log output** for post-processing in Python/Excel
- Embedded-style constraints (no STL containers, no dynamic memory)

## Build & Run

```bash
g++ -std=c++17 -O2 -Wall -o sensor_monitor main.cpp
./sensor_monitor
```

On Windows (MSVC):
```
cl /std:c++17 /O2 main.cpp /Fe:sensor_monitor.exe
sensor_monitor.exe
```

## Example Output

```
╔══════════════════════════════════════════════════════════╗
║          EMBEDDED SENSOR MONITOR — SIMULATION           ║
╚══════════════════════════════════════════════════════════╝

Tick  Sensor        Raw       Filtered    Status
──────────────────────────────────────────────────────
0     TEMP_MCU      75.12     75.12       OK °C
0     VDD_CORE      1.10      1.10        OK V
...
295   TEMP_MCU      109.44    91.23       [!] OVER_MAX °C
...
SIMULATION COMPLETE — 500 samples @ 100 Hz
Channel          Faults    Rate(%)
TEMP_MCU         11        2.2%
VDD_CORE         0         0.0%
```

## Configuration

Edit the constants at the top of `main.cpp`:

| Constant | Description |
|---|---|
| `NUM_CHANNELS` | Number of sensor channels |
| `BUFFER_SIZE` | Circular buffer depth |
| `MA_WINDOW` | Moving average window |
| `SAMPLE_RATE_HZ` | Simulated sample rate |
| `TOTAL_SAMPLES` | Simulation length |

Sensor limits are configured in the `SENSORS` array (`SensorConfig` structs).

## Skills Demonstrated

- Embedded C++ (C++17, no dynamic allocation)
- Real-time sensor processing & fault detection
- Template data structures (ring buffer)
- Signal filtering (moving average)
- Instrumentation & test/validation engineering

## Author

Muhammad Hassan — Electronics Engineer  
[LinkedIn](https://linkedin.com/in/muhammad-hassan-msc)
