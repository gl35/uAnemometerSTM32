# Ultrasonic Anemometer — STM32 Implementation

An updated STM32-based firmware for an ultrasonic anemometer using analog microphones, building on the research published at **IEEE ISIE 2025**.

📄 **[Read the published paper on IEEE Xplore](https://ieeexplore.ieee.org/document/11124668)**

> This is the latest version of the anemometer firmware. For the earlier RPi Pico-based implementation, see [usAnemometer](https://github.com/gl35/usAnemometer).

---

## Overview

This repository contains the STM32 firmware for a low-cost ultrasonic anemometer that measures wind speed using time-of-flight (ToF) of ultrasonic pulses. The receiver uses an **analog microphone** instead of a matched piezoelectric transducer, significantly reducing hardware cost.

This version migrates the real-time firmware entirely to STM32, improving timing precision and reducing system complexity compared to the dual-MCU architecture of the original implementation.

---

## Key Technical Features

- **FreeRTOS state machine** — deterministic task scheduling for accurate 36µs pulse width capture
- **22 kHz DAC sine wave generation** — stable analog drive signal for the ultrasonic transducer
- **High-resolution ToF measurement** — timer-based pulse capture for wind speed calculation
- **ADC temperature compensation** — raw voltage from temperature sensor digitized and used to correct speed of sound
- **PLL/JKFF signal conditioning** — converts sinusoidal microphone output to synchronized square pulses for phase detection

---

## System Architecture

```
[DAC] --> [22kHz Sine Wave] --> [Ultrasonic Transducer TX]
                                          |
                                 [Airflow path]
                                          |
[Analog Microphone RX] --> [PLL/JKFF Circuit] --> [STM32 Timer Capture]
                                                          |
                                               [FreeRTOS State Machine]
                                                          |
                                              [Wind Speed Calculation]
                                                          |
                                               [UART / Serial Output]
```

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller | STM32 |
| Transmitter | Ultrasonic transducer (22 kHz) |
| Receiver | Analog microphone |
| Signal Conditioning | JKFF + PLL for phase detection |
| Temperature Sensor | Analog output, ADC digitized |

---

## Software

| Component | Language | Description |
|---|---|---|
| Real-time firmware | C | FreeRTOS, ToF capture, DAC, ADC |
| Build system | Makefile | STM32 cross-compilation |

---

## Publication

**G. Li**, "Ultrasonic Anemometer Using Microphones," *2025 IEEE International Symposium on Industrial Electronics (ISIE)*, 2025.

🔗 https://ieeexplore.ieee.org/document/11124668

---

## Author

**Gene Li, EIT** — Mechatronic/Electrica/Electronics Engineer  
MASc, Simon Fraser University  
[GitHub](https://github.com/gl35)
