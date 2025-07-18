# Dynamics PCB Firmware

## 🏎️ Overview
This firmware powers the Formula Student Dynamics PCB system, delivering real-time monitoring and data acquisition for critical vehicle dynamics parameters. Engineered for high-performance racing environments, the system provides dual-configuration support for both front and rear vehicle positions with specialized sensor suites.

## ⚙️ Configuration

### PCB Type Selection
The firmware supports two distinct configurations through compile-time defines:

```c
// Dynamics Front or Dynamics Rear?
#define DYNAMICS_FRONT
//#define DYNAMICS_REAR
```

> ⚠️ **Important:** Only one configuration should be active at compile time. Comment out the unused define.

| Configuration | Sensors | CAN IDs | Primary Function |
|---------------|---------|---------|------------------|
| **DYNAMICS_FRONT** | Steering Angle, Suspension (L/R), Wheel Speed (L/R) | 0x446, 0x456 | Front axle dynamics monitoring |
| **DYNAMICS_REAR** | Brake Pressure, Suspension (L/R), Wheel Speed (L/R) | 0x546, 0x556 | Rear axle dynamics monitoring |

## 🔧 Hardware Specifications

### Microcontroller Platform
- **MCU:** STM32F4xx series (72MHz ARM Cortex-M4)
- **Architecture:** 32-bit RISC processor with FPU
- **Memory:** Flash + SRAM configuration
- **Power:** 3.3V operation with 5V sensor compatibility

### Peripheral Configuration
| Peripheral | Function | Configuration |
|------------|----------|---------------|
| **ADC1** | 4-channel sensor input | 12-bit, 480-cycle sampling, DMA |
| **TIM13** | Left wheel speed capture | Input capture, falling edge |
| **TIM14** | Right wheel speed capture | Input capture, falling edge |
| **CAN1/CAN2** | Dual bus communication | 500 kbps, standard frame |
| **UART1** | Debug interface | 115200 baud, 8N1 |
| **DMA** | High-speed data transfer | ADC to memory |

## 📊 Sensor Specifications

### 🎯 Steering Angle Sensor (Front PCB Only)
```
Operating Voltage:    0.5V - 4.5V
Measurement Range:    ±180°
Resolution:          ~0.088°/step
Interface:           ADC Channel 1
Voltage Scaling:     2/3 divider (5V→3.3V)
Calibration Offset:  -31.3° (mechanical)
```

### 🔧 Brake Pressure Sensor (Rear PCB Only)
```
Operating Voltage:    0.5V - 4.5V
Pressure Range:      0 - 140 bar
Sensitivity:         28.57 mV/bar
Zero Offset:         0.5V @ 0 bar
Resolution:          ~0.034 bar/step
Interface:           ADC Channel 2
```

### 🏁 Suspension Position Sensors
```
Operating Voltage:    0V - 5V
Stroke Range:        75mm electrical
Channels:            Left (ADC8), Right (ADC9)
Linearity:           Linear potentiometric
Resolution:          ~0.018 mm/step
```

### ⚡ Wheel Speed Sensors
```
Sensor Type:         Hall effect / Magnetic pickup
Signal Type:         Digital pulse train
Trigger:             Falling edge detection
Cogwheel:           36 teeth per revolution
Wheel Diameter:     20.5" (520.7mm)
Timer Frequency:    62.937 kHz (1μs resolution)
Timeout:            500ms stationary detection
```

## 🔄 Signal Processing Pipeline

### ADC Signal Conditioning
1. **Hardware Filtering:** 480-cycle sampling for noise reduction
2. **Voltage Scaling:** Automatic 5V→3.3V conversion handling
3. **Moving Average:** 50-sample rolling filter per channel
4. **Boundary Checking:** Automatic range validation
5. **Calibration:** Sensor-specific offset and scaling

### Speed Calculation Algorithm
```c
// Simplified speed calculation flow
Period = CurrentCapture - LastCapture;
Frequency = 1 / (Period × Timer_Resolution);
RPM = (Frequency × 60) / TeethCount;
Speed_kmh = (WheelPerimeter_m × RPM) × 0.06;
```

### Noise Reduction Strategy
- **Multi-stage filtering:** Hardware + software approach
- **Outlier rejection:** Invalid period detection
- **Temporal smoothing:** Moving average filters
- **Overflow handling:** 16-bit timer wraparound compensation

## 📡 Communication Protocol

### CAN Bus Architecture
```
Topology:           Dual redundant buses
Baudrate:           500 kbps
Frame Format:       Standard 11-bit identifier
Transmission Rate:  20 Hz (50ms intervals)
Data Encoding:      Little-endian, scaled integers
Error Handling:     Automatic retransmission disabled
```

### Message Formats

#### 🔧 Front PCB Messages

**Sensor Data (ID: 0x446)**
```
Byte 0-1: Steering Angle [deg × 10] (signed 16-bit)
Byte 2-3: Right Suspension [mm × 10] (unsigned 16-bit)
Byte 4-5: Left Suspension [mm × 10] (unsigned 16-bit)
```

**Speed Data (ID: 0x456)**
```
Byte 0-1: Left Wheel Speed [km/h × 10] (unsigned 16-bit)
Byte 2-3: Right Wheel Speed [km/h × 10] (unsigned 16-bit)
```

#### 🔧 Rear PCB Messages

**Sensor Data (ID: 0x546)**
```
Byte 0-1: Brake Pressure [bar × 10] (unsigned 16-bit)
Byte 2-3: Right Suspension [mm × 10] (unsigned 16-bit)
Byte 4-5: Left Suspension [mm × 10] (unsigned 16-bit)
```

**Speed Data (ID: 0x556)**
```
Byte 0-1: Left Wheel Speed [km/h × 10] (unsigned 16-bit)
Byte 2-3: Right Wheel Speed [km/h × 10] (unsigned 16-bit)
```

### Debug Interface (UART)
- **Purpose:** Real-time sensor monitoring and diagnostics
- **Format:** Human-readable ASCII output
- **Rate:** Continuous in main loop
- **Connection:** Standard USB-UART bridge

## 🚀 Performance Metrics

| Parameter | Specification | Notes |
|-----------|---------------|--------|
| **System Update Rate** | Continuous | Real-time operation |
| **CAN Transmission** | 20 Hz | 50ms intervals |
| **ADC Resolution** | 12-bit (4095 levels) | Hardware limited |
| **Speed Accuracy** | ±0.1 km/h | After filtering |
| **Angle Accuracy** | ±0.1° | After calibration |
| **Pressure Accuracy** | ±0.1 bar | Linear sensor |
| **Response Time** | <50ms | Sensor to CAN output |

## 🛠️ Development Setup

### Prerequisites
- **IDE:** STM32CubeIDE (recommended) or compatible ARM toolchain
- **Debugger:** ST-LINK/V2 or compatible
- **Hardware:** Target STM32F4xx development board or custom PCB

### Build Instructions

1. **Configure PCB Type:**
   ```c
   // In main.c, lines 26-27
   #define DYNAMICS_FRONT    // Enable for front PCB
   //#define DYNAMICS_REAR   // Enable for rear PCB (comment out FRONT)
   ```

2. **Compile Project:**
   - Open project in STM32CubeIDE
   - Select appropriate build configuration
   - Build project (Ctrl+B)

3. **Flash Firmware:**
   - Connect ST-LINK debugger
   - Flash to target device
   - Verify successful programming

4. **Verify Operation:**
   - Connect UART terminal (115200 baud)
   - Monitor sensor outputs
   - Check CAN bus activity

### Development Tools
```bash
# Required tools
- ARM GCC Toolchain
- STM32CubeMX (hardware configuration)
- STM32CubeProgrammer (flashing)
- CAN analyzer (bus monitoring)
- Oscilloscope (signal verification)
```

## 📋 System Features

### ✅ Core Functionality
- [x] Multi-sensor data acquisition
- [x] Real-time CAN bus communication
- [x] Dual configuration support
- [x] Advanced signal filtering
- [x] Error detection and handling
- [x] Debug interface

### 🛡️ Reliability Features
- [x] Watchdog timer protection
- [x] Sensor timeout detection
- [x] CAN bus redundancy
- [x] Overflow compensation
- [x] Boundary value checking
- [x] Heartbeat LED monitoring

### 📈 Advanced Capabilities
- [x] Moving average filtering
- [x] Mechanical calibration offsets
- [x] Temperature-stable operation
- [x] High-frequency data acquisition
- [x] Deterministic real-time behavior

## 🔍 Troubleshooting

### Common Issues

| Issue | Symptoms | Solution |
|-------|----------|----------|
| **No CAN Output** | Silent bus, no messages | Check CAN termination, verify bus wiring |
| **Incorrect Speeds** | Wrong values | Verify wheel diameter, tooth count settings |
| **Noisy Signals** | Erratic readings | Check sensor mounting, increase filter size |
| **Missing Sensors** | Zero/invalid values | Verify power supply, sensor connections |

### Debug Commands
```c
// Enable verbose debugging (add to main.c)
#define DEBUG_VERBOSE
printf("Debug: ADC[0]=%u, Filtered=%u\n", ADC_VALUE[0], adc_filtered[0]);
```

## 📚 Dependencies

### Software Requirements
- **STM32 HAL Library** (STM32F4xx package)
- **CMSIS** (ARM Cortex-M4 support)
- **Standard C Library** (math.h, stdio.h)
- **STM32CubeMX Generated Code** (peripheral initialization)

### Hardware Requirements
- **STM32F4xx Microcontroller** (minimum 256KB Flash, 64KB RAM)
- **External Crystal** (HSE for accurate timing)
- **CAN Transceivers** (dual bus support)
- **Power Supply** (3.3V regulated, 5V for sensors)

## 📄 Version Information
```
Firmware Version:    v1.0.0
Target Platform:     STM32F4xx
Compiler:           ARM GCC
HAL Version:        STM32F4xx HAL Library
Language Standard:   C99
License:            Proprietary (Formula Student Team)
```

## ⚠️ Important Notes

> **Safety Warning:** This firmware is designed for Formula Student racing applications. Ensure proper testing and validation before competition use.

> **Calibration Required:** All sensors require proper calibration and validation before deployment.

> **CAN Bus:** Verify proper termination (120Ω) on both ends of each CAN bus.

> **Power Supply:** Ensure clean, stable power supply to prevent ADC noise and system instability.

> **Mechanical Installation:** Proper sensor mounting and alignment is critical for accurate measurements.

---

*For technical support and contributions, contact the Formula Student Electronics Team.*