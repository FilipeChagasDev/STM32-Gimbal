# Open Source Gimbal with Servomotor

![Version](https://img.shields.io/badge/Version-v0.1.0-blue)
![License](https://img.shields.io/badge/License-MIT-blue)

## Hardware

* 1x BluePill board (STM32F103C8 microcontroller) 
* 4x 2N7000 MOSFET
* 6x 1K resistors
* 2x 100K resistors
* 2x Tower Pro MG90S servomotors
* 1x MPU6050 3-axis gyroscope accelerometer

## Firmware

* C language
* **STM32CubeMX** generated HAL
* Compiled with **arm-none-eabi-gcc**
* Makefile toolchain
* No RTOS
* MCU Flashing with **st-link** device and **st-flash** (Linux) software

## Behaviour
* Information about the MPU6050's position is obtained 200 times per second **(TIM4 IRQ)**.
* Servomotors position is updates 50 times per second **(TIM3 IRQ)**.
* UART messages are sent periodically **(TIM2 IRQ)**.
* In the event of a failure, the ERROR led wil blink and the MCU will reset.

## Physical structure 

TODO
