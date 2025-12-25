| Supported Targets | ESP32 | ESP32-C3 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- |

# MPPT Boost Converter using ESP32

This project implements a **Maximum Power Point Tracking (MPPT) Boost Converter**
using **ESP32** and **INA219 current/voltage sensor**.
The system dynamically adjusts the PWM duty cycle to extract maximum power from a
DC source (e.g. solar panel).

---

## Overview

The project uses:
- **INA219** for real-time voltage and current measurement
- **PWM (LEDC driver)** to control a DC-DC boost converter
- **Perturb & Observe (P&O)** MPPT algorithm
- **Soft-start** and **protection mechanisms** to ensure system safety

The firmware is developed using **ESP-IDF v5.5**.

---

## Hardware Required

- ESP32 development board
- INA219 current & voltage sensor (I2C)
- DC-DC Boost Converter circuit
  - Inductor
  - MOSFET
  - Diode
  - Output capacitor
- DC power source / Solar panel
- USB cable for programming and power

---

## Software & Tools

- ESP-IDF v5.5
- FreeRTOS
- LEDC (PWM driver)
- I2C driver
- Visual Studio Code (recommended)

---

## System Architecture

- **INA219** measures:
  - Input voltage (Vpin)
  - Input current (Ipin)
- ESP32 calculates:
  - Input power `P = V × I`
- MPPT algorithm updates:
  - PWM duty cycle (0.05 → 0.6)
- PWM drives:
  - MOSFET of the boost converter

---

## MPPT Algorithm

The project uses the **Perturb and Observe (P&O)** algorithm:

1. Measure voltage and current
2. Calculate power
3. Compare with previous power
4. Adjust PWM duty cycle to move toward maximum power point

A small step size is used to reduce oscillation.

---

## Key Features

- ✅ MPPT using Perturb & Observe
- ✅ Soft-start mechanism
- ✅ Input voltage and current filtering (LPF)
- ✅ Over-current protection
- ✅ Under-voltage protection
- ✅ PWM frequency: 20 kHz
- ✅ Duty cycle limit: max 60%

---

## Project Configuration

Before building, select the correct target:

```bash
idf.py set-target esp32
