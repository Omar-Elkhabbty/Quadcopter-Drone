# 🛸 ESP32-FlightCore-GPS

![Platform](https://img.shields.io/badge/Platform-ESP32-orange.svg)
![Language](https://img.shields.io/badge/Language-C%2B%2B-blue.svg)
![Framework](https://img.shields.io/badge/Framework-Arduino%2FPlatformIO-green.svg)
![License](https://img.shields.io/badge/License-All%20Rights%20Reserved-red.svg)

An open-source, custom-built Quadcopter Flight Controller firmware and hardware architecture built from scratch using **ESP32**, **MPU-6050 IMU**, **BMP280 Barometer**, **u-blox NEO-6M GPS**, and **FlySky iBUS Receiver**.

---

## 📌 Project Overview

`ESP32-FlightCore-GPS` is a lightweight, low-latency quadcopter flight control system. Designed specifically for custom hardware implementations (Custom PCB), it features a **250Hz closed-loop PID controller**, non-blocking NMEA GPS parser, altitude estimation via pressure sensing, digital RC telemetry, and an independent FPV video streaming system via ESP32-CAM.

---

## 🚀 Key Features

* **250Hz Closed-Loop PID Control:** Fixed 4ms cycle time handling Roll, Pitch, and Yaw rate/angle stabilization.
* **FlySky iBUS Integration:** Non-blocking 32-byte serial frame parser running on Hardware UART2 at 115200 Baud.
* **Integrated GPS Telemetry:** Lightweight custom NMEA parser extracting Latitude, Longitude, Altitude, and Satellite count.
* **Automatic Home Lock:** Locks takeoff coordinates upon arming once valid GPS satellite fix is established.
* **Altitude Estimation:** High-precision pressure sensing via BMP280 over shared I2C bus.
* **Safety Arming System:** Hardware arming switch on Aux channel with fail-safe cutoff below minimum throttle.
* **Battery Telemetry:** ADC voltage monitoring (GPIO 34) calibrated for 3S LiPo batteries.
* **Standalone Live Video Stream:** ESP32-CAM onboard for WiFi live stream and SD card recording.

---

## 📐 System Architecture

```text
                        +-----------------------+
                        |   LiPo Battery 11.1V  |
                        +-----------+-----------+
                                    |
                                    v
                        +-----------------------+
                        |  Power Distribution   |
                        |      Board (PDB)      |
                        +---+---------------+---+
                            |               |
                (Main 11.1V Power)    (5V BEC Regulator)
                            |               |
            +---------------+---------------+---------------+
            |               |               |               |
            v               v               v               v
        +-------+       +-------+       +-------+       +---------------+
        | ESC 1 |       | ESC 2 |       | ESC 3 |       | ESC 4         |
        +---+---+       +---+---+       +---+---+       +---+-----------+
            |               |               |               |
            v               v               v               v
        [Motor 1]       [Motor 2]       [Motor 3]       [Motor 4]
       (Front-R CCW)   (Rear-R CW)     (Rear-L CCW)    (Front-L CW)

                                    | (5V Power)
                                    v
                          +-------------------+
                          |  ESP32 MCU Core   |
                          +---------+---------+
                                    |
    +-------------------+-----------+--------+-------------------+------------------+
    | I2C Bus           | Hardware UART1     | Hardware UART2    | PWM Motor Signal |
    v                   v                    v                   v                  v
+-----------+     +-----------+        +--------------+    +---------------+  
| MPU-6050  |     |  NEO-6M   |        | FlySky iBUS  |    | ESC Signals   |  
| BMP280    |     |  GPS      |        | RX Receiver  |    | M1: GPIO13    |  
| SCL: GPIO22     | RX1: GPIO18        | RX2: GPIO16  |    | M2: GPIO12    |  
| SDA: GPIO21     | TX1: GPIO17        +--------------+    | M3: GPIO14    |  
+-----------+     +-----------+                            | M4: GPIO27    |  
                                                           +---------------+
