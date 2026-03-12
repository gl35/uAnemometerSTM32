# Ultrasonic Anemometer Using Microphones

A low-cost, hybrid analog-digital ultrasonic anemometer for indoor HVAC wind speed monitoring, developed as part of published research at the **IEEE International Symposium on Industrial Electronics (ISIE) 2025**.

📄 **[Read the published paper on IEEE Xplore](https://ieeexplore.ieee.org/document/11124668)**

---

## Overview

Traditional ultrasonic anemometers rely on expensive matched piezoelectric transducers. This project replaces one transducer pair with low-cost **analog microphones**, significantly reducing hardware cost while maintaining measurement accuracy.

Wind speed is determined by measuring the **time-of-flight (ToF)** difference of ultrasonic pulses traveling in opposite directions along the airflow axis. A 22 kHz sine wave is generated and transmitted by an ultrasonic transducer; the received signal is captured by an analog microphone and digitized for processing.

---

## Key Technical Features

- **22 kHz DAC sine wave generation** — stable analog drive signal for the ultrasonic transducer
- **36 µs pulse width timing** — high-resolution ToF measurement using a freeRTOS-based state machine on STM32
- **Phase-based measurement** — JKFF and PLL circuits convert sinusoidal signals into synchronized square pulses for precise phase detection
- **ADC temperature compensation** — raw voltage from a temperature sensor is digitized and used to correct the speed of sound for accurate wind speed calculation
- **Dual MCU architecture** — RPi Pico handles data acquisition and communication; STM32 handles real-time timing-critical tasks
- **FreeRTOS state machine** — ensures deterministic task scheduling for accurate pulse capture

---

## System Architecture

```
[Ultrasonic Transducer TX] --> [22kHz DAC Signal Gen]
                                        |
                               [Airflow measurement path]
                                        |
[Analog Microphone RX] --> [PLL/JKFF Phase Circuit] --> [STM32 - ToF Capture]
                                                                  |
                                                        [FreeRTOS State Machine]
                                                                  |
                                                         [RPi Pico - Data Processing]
                                                                  |
                                                        [Python - Logging & Visualization]
```

---

## Hardware

| Component | Details |
|---|---|
| Microcontroller 1 | STM32 (real-time ToF capture, freeRTOS) |
| Microcontroller 2 | Raspberry Pi Pico (data processing, comms) |
| Transmitter | Ultrasonic transducer (22 kHz) |
| Receiver | Analog microphone |
| Signal Conditioning | JKFF + PLL circuit for phase detection |
| Temperature Sensor | Analog output, ADC digitized for calibration |

---

## Software

| Component | Language | Description |
|---|---|---|
| Real-time firmware | C | FreeRTOS state machine, ToF measurement, ADC, DAC |
| Data processing | Python | Wind speed calculation, logging, visualization |
| Build system | CMake | Cross-platform embedded build |

---

## Results

The system successfully measured indoor wind speeds with accuracy validated against reference measurements. Results and full methodology are documented in the published paper.

---

## Publication

**G. Li**, "Ultrasonic Anemometer Using Microphones," *2025 IEEE International Symposium on Industrial Electronics (ISIE)*, 2025.

🔗 https://ieeexplore.ieee.org/document/11124668

---

## Repository Structure

```
usAnemometer/
├── program/        # Embedded firmware (C) and data processing (Python)
├── PCB/            # PCB design files
└── README.md
```

---

## Author

**Gene Li, EIT** — Mechatronic/Electrical/Electronics Engineer  
MASc, Simon Fraser University  
[GitHub](https://github.com/gl35)
