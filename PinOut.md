# 🔌 ESP32 Flight Controller - Hardware Pinout & Wiring Specification

This document details the complete hardware pinout matrix and electrical connections for the ESP32-based Quadcopter Flight Controller.

---

## 1. Main Controller Pinout Matrix

| Component / Module | Component Pin | ESP32 GPIO Pin | Protocol / Signal Type | Software Assignment & Notes |
| :--- | :--- | :--- | :--- | :--- |
| **PDB 5V BEC** | 5V Output | `Vin` / `5V` | Power Input | Main 5V power supply to ESP32 board |
| **Power Ground** | GND | `GND` | Power Ground | Common system ground reference |
| **MPU-6050 IMU** | SDA | `GPIO 21` | I2C Data Bus | Motion & Accelerometer data (Addr: `0x68`) |
| **MPU-6050 IMU** | SCL | `GPIO 22` | I2C Clock Bus | I2C clock line for IMU |
| **BMP280 Baro** | SDA | `GPIO 21` | I2C Data Bus | Barometric altitude data (Addr: `0x76`) |
| **BMP280 Baro** | SCL | `GPIO 22` | I2C Clock Bus | Shared clock line with MPU-6050 |
| **NEO-6M GPS** | TX | `GPIO 18` | Hardware UART1 (RX1) | Serial NMEA GPS data line (9600 Baud) |
| **NEO-6M GPS** | RX | `GPIO 17` | Hardware UART1 (TX1) | GPS configuration line |
| **FS-iA6B Receiver**| iBUS Servo Pin | `GPIO 16` | Hardware UART2 (RX2) | Digital FlySky iBUS input (115200 Baud) |
| **ESC 1 (Front Right)**| Signal Pin | `GPIO 13` | PWM Output (LEDC) | Motor 1 speed control (CCW Propeller) |
| **ESC 2 (Rear Right)** | Signal Pin | `GPIO 12` | PWM Output (LEDC) | Motor 2 speed control (CW Propeller) |
| **ESC 3 (Rear Left)**  | Signal Pin | `GPIO 14` | PWM Output (LEDC) | Motor 3 speed control (CCW Propeller) |
| **ESC 4 (Front Left)** | Signal Pin | `GPIO 27` | PWM Output (LEDC) | Motor 4 speed control (CW Propeller) |
| **Voltage Sensing**| Divider Out | `GPIO 34` | Analog ADC Input | Battery voltage monitoring (Input Only) |

---

## 2. Electrical Subsystems & Connection Details

### A. I2C Bus Cluster (Sensors)
* **Bus Pins:** `SDA -> GPIO 21`, `SCL -> GPIO 22` (400kHz Fast Mode)
* **Devices:** MPU-6050 (`0x68`) and BMP280 (`0x76`) connected in parallel.
* **PCB Requirement:** Add 4.7kΩ pull-up resistors on both SDA and SCL lines to 3.3V.

### B. Serial Communications (UART)
* **UART1 (GPS):** RX1 -> `GPIO 18`, TX1 -> `GPIO 17` at 9600 Baud.
* **UART2 (FlySky iBUS):** RX2 -> `GPIO 16` at 115200 Baud.

### C. Battery Voltage Monitor (ADC Circuit)
* **Input Voltage:** 3S LiPo Battery (Up to 12.6V fully charged).
* **Divider Resistors:** $R_1 = 33\text{k}\Omega$, $R_2 = 10\text{k}\Omega$.
* **ADC Output Pin:** Connected to `GPIO 34` (Max Output Voltage $\approx 2.93\text{V}$).

### D. ESC Power & Signal
* **PWM Frequency:** 250 Hz (4ms period).
* **Pulse Range:** 1024 to 2048 (12-bit LEDC resolution).