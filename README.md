# RTOS Quadcopter Flight Controller

Firmware for a **quadcopter flight controller** implemented on **STM32F4** using **FreeRTOS**.

The project implements a modular real-time drone control system including **sensor drivers, flight stabilization algorithms, receiver communication, and telemetry interface**.

---

## Demo

### Euler Angle Telemetry (Ground Station GUI)

Real-time attitude data (roll, pitch, yaw) streamed from the STM32 flight controller to the ground station GUI via UART telemetry.

[![Euler Angle Telemetry](https://img.youtube.com/vi/O2XJQq_KRWs/0.jpg)](https://youtu.be/O2XJQq_KRWs)

### Roll / Pitch Control Response
Demonstration of roll and pitch stabilization response during manual testing.

Hardware response:

https://youtube.com/shorts/CnKSQ-TkWs8?feature=share

Telemetry visualization:

https://youtu.be/wwBK6YbCQGs

### IMU Calibration (Tripod Test)

Sensor calibration test while the drone is mounted on a tripod.

https://youtu.be/2brUX-lan2c

### Initial Flight Test

Early flight test demonstrating basic motor control and stabilization behavior.

https://youtu.be/DUdOFClbhr0

Note: This footage was recorded during an early development phase before further control tuning and filtering improvements were implemented.

---

## Hardware Platform

- **MCU:** STM32F410RBTx
- **IMU:** BNO055 (Accelerometer + Gyroscope + Sensor Fusion)
- **Barometer:** BMP280
- **Motors:** Brushless DC motors with ESC
- **Receiver:** FlySky iBus (FS-iA6B)
- **Communication:** UART telemetry interface
- **Battery Monitoring:** ADC voltage measurement

---

## System Architecture

The firmware is built using a **FreeRTOS task-based architecture** where each subsystem runs independently.

### Data Flow
```
IMU (BNO055) + Barometer (BMP280)
↓
Sensor Filtering (Low Pass + EKF)
↓
Attitude Estimation (Euler angles)
↓
PID Flight Controller
↓
Motor Mixing
↓
ESC PWM Output
```

---

## FreeRTOS Tasks

The system is divided into several real-time tasks:

| Task | Function |
|-----|------|
| **MPU Task** | Reads IMU acceleration and gyroscope data (~1kHz) |
| **Euler Task** | Reads Euler angles from IMU sensor (BNO055) |
| **Filter Task** | Fuses accelerometer and barometer data using EKF |
| **Barometer Task** | Reads altitude data from barometer (BMP280) |
| **Control Task** | Executes PID stabilization loop |
| **Receiver Decode Task** | Decodes FlySky iBus receiver data |
| **Telemetry Task** | Sends acceleration + gyroscope + Euler data to ground station GUI via DMA|

Inter-task communication is implemented using **FreeRTOS mail queues**.

---

## Control System

The flight controller uses **nested PID loops**:

### Attitude Control
Angle PID (Outer Loop)
↓
Angular Rate PID (Inner Loop)

### Yaw Control
Heading Hold PID
or
Yaw Rate PID

### Altitude Estimation

Altitude is estimated by fusing:

- **Barometer (BMP280)**
- **Accelerometer Z-axis**

using an **Extended Kalman Filter (EKF)**.

---

## Features

- STM32F4 real-time flight controller firmware
- FreeRTOS multitasking architecture
- IMU + Barometer sensor fusion
- Extended Kalman Filter for altitude estimation
- PID-based flight stabilization
- FlySky iBus receiver decoding
- ESC calibration and motor arming logic
- UART DMA telemetry interface
- Battery voltage monitoring
- Modular driver architecture

---

## Firmware Structure

```
RTOS_Drone_Firmware
│
├── communication
│   └── ibus
│
├── control
│   └── pid
│
├── drivers
│   ├── imu
│   ├── bldc
│   └── barometer
│
├── firmware
│   └── main.c
│
└── README.md
```

## Project Goals

This project was developed to practice:

- Embedded firmware architecture
- Real-time systems with FreeRTOS
- Sensor integration
- Control systems for robotics
- Drone flight controller design

---

## Author

Nguyen Minh Quan  
Embedded Systems / Robotics
