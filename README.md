# RTOS Drone Firmware

Firmware for a quadcopter flight controller implemented on **STM32F4** using **FreeRTOS**.

This project implements the core components of a drone control system including sensor drivers, communication interfaces, and real-time control loops.

---

## Features

- STM32F4-based flight controller firmware
- FreeRTOS task-based architecture
- PID-based attitude stabilization
- IMU sensor integration
- iBus receiver communication
- BLDC motor control via PWM
- Modular driver architecture

---

## System Architecture

The firmware is organized into several modules:

---

## Hardware Platform

- MCU: **STM32F410RBTx**
- Sensors: **IMU (accelerometer + gyroscope)**
- Motors: **Brushless DC motors with ESC**
- Receiver: **FlySky iBus**

---

## RTOS Tasks

Example tasks used in the system:

- **MPU Task** – Reads IMU data
- **Control Task** – Runs PID stabilization loop and updates motor PWM outputs
- **Receiver Decode Task** – Handles receiver input
- **Telemetry Task** – Communicates with tuning GUI
- **Filter Task** – 
- **Barometer Task** – Updates barometer output
- **Euler Task** – Updates Euler angles (roll, pitch, yaw) on Gui
---

## Project Goals

This project was developed to practice:

- Embedded firmware architecture
- Real-time control systems
- Sensor integration
- RTOS-based embedded design

---

## Author

Nguyen Minh Quan  
Embedded Systems / Robotics
