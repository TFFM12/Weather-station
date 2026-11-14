# IoT Solar-Powered Weather Station (LABSI)

[![Framework](https://img.shields.io/badge/Framework-ESP--IDF-blue.svg)](https://docs.espressif.com/projects/esp-idf/)
[![Microcontroller](https://img.shields.io/badge/MCU-ESP32--S3%20(N16R8)-green.svg)](https://www.espressif.com/)
[![Protocol](https://img.shields.io/badge/Protocol-MQTT-orange.svg)](https://mqtt.org/)
[![Status](https://img.shields.io/badge/Status-Work%20in%20Progress-yellow.svg)]()

Autonomous, professional-grade IoT weather monitoring station developed as a final project for the **Integrated Systems Laboratory (LABSI)** course at **ISEP**. The system is built using native **ESP-IDF** in **C++**, featuring FreeRTOS multitasking, secure hardware-level sensor interfacing, solar power management, and real-time cloud telemetry via MQTT.

---

## 🚀 System Architecture

The firmware is designed with strict architectural separation across the dual cores of the ESP32-S3 using **FreeRTOS**:

* **Core 1 (Deterministic Sampling):** Handles high-priority sensor polling, hardware pulse counting (PCNT) for wind speed and precipitation, and $I^2C$ digital communications.
* **Core 0 (Communication & Management):** Manages the Wi-Fi stack, asynchronous MQTT telemetry publishing, and background system health tasks (including a dedicated 1Hz system status heartbeat).

```
 ┌────────────────────────────────────────────────────────┐
 │                 SOLAR POWER SUBSYSTEM                  │
 │  24V Voc Solar Panel ──> [ MPPT Manager ] <──> Li-Ion  │
 └───────────────────────────┬────────────────────────────┘
                             │ Regulated Rails
                             ▼
 ┌────────────────────────────────────────────────────────┐
 │                   ESP32-S3 MCU                         │
 │                                                        │
 │  [ Core 1: Sensor Sampling & HW Interrupts / PCNT ]    │
 │  [ Core 0: Wi-Fi Stack & MQTT Asynchronous Publisher ] │
 └───────────────────────────┬────────────────────────────┘
                             │
                             ▼ [ MQTT over Wi-Fi ]
                   [ Central Broker / Dashboard ]
```

---

## 🛠️ Hardware Components

* **Microcontroller:** ESP32-S3-N16R8 (16MB Flash, 8MB PSRAM).
* **Power Management:** Waveshare Solar Power Manager (D) with MPPT circuit and 3.7V Li-ion storage.
* **Wind Direction:** Magnetic absolute encoder ($360^\circ$) via $I^2C$.
* **Wind Speed:** 3-cup anemometer with digital pulse counter (Reed Switch + Hardware PCNT).
* **Precipitation:** Tipping-bucket rain gauge ($0.2\text{ mm/tip}$) via hardware GPIO interrupts.
* **Temperature & Humidity:** Sensirion SHT31-D digital sensor ($I^2C$) inside a solar radiation shield.

---

## ⚙️ Software Stack & Key Features

* **Bare-Metal / Native ESP-IDF:** No Arduino abstractions; custom low-level configuration of peripherals.
* **FreeRTOS Multitasking:** Deterministic task scheduling with priority inversion protection and queue-based inter-task communication.
* **Hardware Pulse Counter (PCNT):** Accurate frequency acquisition for anemometer pulses without CPU overhead.
* **Resilient Telemetry:** Asynchronous MQTT publishing with automated reconnection logic and quality-of-service (QoS 1) handling.

---

## 📂 Repository Structure

```text
.
├── 3D/                   # Modelos STL e 3MF para impressão 3D (pluviómetro, anemómetro, etc.)
├── Docs/                 # Documentação e datasheets dos sensores e módulos (AS5600, SHT31, ESP32-S3, etc.)
├── Project/
│   └── weather_station/  # Código-fonte principal do ESP-IDF (ex-sample_project)
│       ├── CMakeLists.txt
│       ├── main/
│       │   ├── CMakeLists.txt
│       │   └── main.c
│       └── sdkconfig
├── Test/
│   ├── teste_ReedSwitch/ # Projetos de teste isolados (ex: sensor reed switch)
│   └── teste_SenRot/     # Projetos de teste para sensores rotativos / encoder
└── README.md
```

---

## 💻 Getting Started & Status

### Prerequisites

1. Install the **ESP-IDF v5.x** framework and its toolchain.
2. Ensure Python and Ninja/CMake build tools are configured in your environment.

## 📄 License

This project is open-source under the terms of the MIT License.
