# STM32 Tilt-Stabilized Platform

Real-time closed-loop tilt stabilization system built using an STM32F407 microcontroller, MPU6050 IMU, and motor actuation.

## Features

- STM32F407 embedded firmware
- MPU6050 communication over I2C
- Real-time pitch and roll estimation
- Feedback control
- PWM-based motor actuation
- UART debugging
- LCD monitoring

## Hardware

- STM32F407G-DISC1
- MPU6050 IMU
- Stepper motor
- Motor driver
- 16x2 LCD

## Software & Tools

- Embedded C
- STM32CubeIDE
- STM32 HAL
- I2C
- UART
- PWM
- Timers

## Project Overview

The system reads motion data from the MPU6050, estimates platform tilt, applies feedback control, and generates motor commands to stabilize the platform in real time.

Additional project details, source code, system diagrams, and test results will be added as the repository is organized.
