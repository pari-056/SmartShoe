# Smart Safety Shoe

## Overview
The Smart Safety Shoe is an ESP32-based safety prototype that integrates motion sensing, foot-pressure monitoring, GPS positioning, and a manual emergency alert.

## Hardware Components
- ESP32 Dev Module
- MPU6050 accelerometer and gyroscope
- FSR pressure sensor
- NEO-6M GPS module
- Emergency push button
- Buzzer
- 10 kΩ resistor

## Code Functionality
- Reads motion data from the MPU6050 through I²C.
- Reads foot-pressure variations through the ESP32 ADC.
- Receives GPS data through UART serial communication.
- Monitors the emergency push button.
- Controls the buzzer according to the programmed conditions.

## Pin Configuration

| Component | ESP32 Pin |
|---|---|
| MPU6050 SDA | GPIO 25 |
| MPU6050 SCL | GPIO 26 |
| FSR | GPIO 34 |
| Emergency button | GPIO 27 |
| Buzzer | GPIO 23 |
| GPS RX | GPIO 16 |
| GPS TX | GPIO 17 |

## Requirements
- ESP32-compatible Arduino IDE setup
- ESP32 board package
- MPU6050 library compatible with the code
- GPS parsing library, if used by the code

## Setup
1. Install the required ESP32 board package and libraries.
2. Connect the hardware according to the pin configuration.
3. Open the firmware in Arduino IDE.
4. Select the appropriate ESP32 board and COM port.
5. Upload the code and verify operation using the Serial Monitor.

## Note
GPS positioning requires suitable satellite reception. The buzzer provides a local alert; remote notifications require additional communication hardware or services.
