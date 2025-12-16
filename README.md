# Parkinson's Disease Movement Detector

A real-time embedded system for detecting and monitoring Parkinson's Disease symptoms including tremor, dyskinesia, and Freezing of Gait (FOG) using advanced signal processing and machine learning techniques.

## 🎯 Overview

This project implements a wearable detection system that monitors movement patterns characteristic of Parkinson's Disease. The system uses an STM32 microcontroller with an IMU sensor to capture motion data, performs real-time FFT analysis, and transmits results via Bluetooth Low Energy (BLE).

## 🔧 Hardware Requirements

- **Board**: STM32 DISCO-L475VG-IOT01A
- **Sensor**: LSM6DSL (6-axis IMU - Accelerometer + Gyroscope)
- **Communication**: Built-in Bluetooth Low Energy
- **LED**: Onboard LED for visual indication
- **Interface**: USB for programming and serial monitoring

## ✨ Features

### Detection Capabilities
- 🔴 **Tremor Detection**: Identifies 3-5 Hz rhythmic oscillations
- 🟠 **Dyskinesia Detection**: Detects 5-7 Hz dance-like involuntary movements
- ❄️ **FOG Detection**: Recognizes Freezing of Gait through step tracking and gait analysis

### Technical Features
- **52 Hz Sampling Rate**: Interrupt-driven data acquisition
- **3-Second Windows**: Continuous windowed data collection
- **FFT Analysis**: 256-point Fast Fourier Transform for frequency analysis
- **BLE Transmission**: Real-time wireless data streaming
- **LED Indication**: Visual feedback patterns for different states
- **Modular Architecture**: Clean, maintainable code structure

## 🏗️ System Architecture

```
┌─────────────────┐
│   LSM6DSL       │  52 Hz interrupt-driven sampling
│   (IMU Sensor)  │  ±2g accel, ±250 dps gyro
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  Data Buffer    │  3-second windows (156 samples)
│  & Preprocessing│
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│  FFT Analysis   │  256-point FFT
│  (CMSIS-DSP)    │  Frequency domain analysis
└────────┬────────┘
         │
         ├─────────────────┬─────────────────┬──────────────────┐
         ▼                 ▼                 ▼                  ▼
    ┌────────┐       ┌──────────┐     ┌──────────┐      ┌──────────┐
    │ Tremor │       │Dyskinesia│     │   FOG    │      │   LED    │
    │Detection│       │Detection │     │Detection │      │ Control  │
    └────┬───┘       └─────┬────┘     └─────┬────┘      └─────┬────┘
         │                 │                 │                  │
         └─────────────────┴─────────────────┴──────────────────┘
                              │
                              ▼
                       ┌──────────────┐
                       │  BLE Service │  3 Characteristics
                       │ Transmission │  Tremor/Dysk/FOG
                       └──────────────┘
```

## 📁 Project Structure

```
demo-main/
├── src/
│   ├── main.cpp              # Main application entry point
│   ├── sensor.cpp            # LSM6DSL sensor interface
│   ├── signal_processing.cpp # FFT analysis & detection
│   ├── fog_detection.cpp     # Freezing of Gait detection
│   ├── ble_comm.cpp          # Bluetooth communication
│   ├── led_control.cpp       # LED pattern control
│   └── config.cpp            # Configuration implementation
├── include/
│   ├── sensor.h
│   ├── signal_processing.h
│   ├── fog_detection.h
│   ├── ble_comm.h
│   ├── led_control.h
│   └── config.h              # System configuration & constants
├── lib/
│   └── CMSIS-DSP-main/       # ARM CMSIS DSP library
├── platformio.ini            # PlatformIO configuration
└── README.md
```

## 🚀 Getting Started

### Prerequisites

1. **PlatformIO IDE** (VS Code extension) or PlatformIO CLI
2. **STM32 DISCO-L475VG-IOT01A** board
3. **USB cable** for programming and power

### Installation

1. **Clone the repository**
   ```bash
   git clone https://github.com/chetank03/pd-detect.git
   cd pd-detect
   ```

2. **Open in PlatformIO**
   - Open VS Code
   - Install PlatformIO extension
   - Open the project folder

3. **Build the project**
   ```bash
   platformio run
   ```

4. **Upload to board**
   ```bash
   platformio run --target upload
   ```

5. **Monitor serial output**
   ```bash
   platformio device monitor
   ```

## 📊 BLE Data Format

The system exposes three BLE characteristics:

| Characteristic | UUID | Range | Description |
|---|---|---|---|
| Tremor Intensity | Custom | 0-1000 | Tremor severity (0=none, 1000=max) |
| Dyskinesia Intensity | Custom | 0-1000 | Dyskinesia severity (0=none, 1000=max) |
| FOG Status | Custom | 0 or 1 | 0=Normal walking, 1=FOG detected |

### Connecting via BLE

1. Enable Bluetooth on your smartphone
2. Look for device: **PD_Detector**
3. Connect and subscribe to characteristics
4. Receive real-time movement data

## 🎨 LED Indication Patterns

The onboard LED provides visual feedback:

- 🔴 **Tremor**: Blink at ~2 Hz (duty cycle proportional to intensity)
- 🟠 **Dyskinesia**: Blink at ~4 Hz (duty cycle proportional to intensity)
- ❄️ **FOG**: Fast triple-blink alarm pattern
- 💚 **Normal**: Slow heartbeat (system alive indicator)

## 🧪 Testing Scenarios

1. **Tremor Detection**
   - Hold board and shake slowly at ~4 Hz
   - Observe LED blinking pattern and serial output

2. **Dyskinesia Detection**
   - Shake board rapidly at ~6 Hz
   - Check for dyskinesia indication

3. **FOG Detection**
   - Walk in place (simulate steps)
   - Stop suddenly
   - System should detect freeze after walking

## 🔬 Technical Details

### Signal Processing Pipeline

1. **Data Acquisition**
   - Interrupt-driven sampling at 52 Hz
   - 3-second sliding windows (156 samples)
   - Z-axis accelerometer primary, gyro complementary

2. **Preprocessing**
   - Mean removal (DC offset)
   - Optional filtering
   - Window buffering

3. **FFT Analysis**
   - 256-point FFT using ARM CMSIS-DSP
   - Frequency resolution: 0.2 Hz
   - Power spectral density calculation

4. **Detection Logic**
   - **Tremor**: Peak energy in 3-5 Hz band
   - **Dyskinesia**: Peak energy in 5-7 Hz band
   - **FOG**: State machine with cadence and variance analysis
   - Multi-window confirmation to reduce false positives

### FOG Detection State Machine

```
NOT_WALKING → WALKING → POTENTIAL_FREEZE → FREEZE_CONFIRMED
     ↑____________↓____________↓___________________↓
```

## ⚙️ Configuration

Key parameters can be adjusted in [`config.h`](include/config.h):

```cpp
#define TARGET_SAMPLE_RATE_HZ 52.0f
#define WINDOW_SIZE 156
#define FFT_SIZE 256
#define TREMOR_FREQ_MIN 3.0f
#define TREMOR_FREQ_MAX 5.0f
#define DYSK_FREQ_MIN 5.0f
#define DYSK_FREQ_MAX 7.0f
```

## 📈 Performance

- **Detection Latency**: ~3 seconds (window duration)
- **CPU Usage**: Optimized with ARM CMSIS-DSP
- **Memory**: Efficient buffering strategy
- **Power**: Low-power sensor configuration

**Note**: This is a research/educational project. For medical applications, proper clinical validation and regulatory approval would be required.
