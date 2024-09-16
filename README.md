Introduction
The Bluetooth_RC_Car_STM32F103 project is a remote-controlled car model operated via Bluetooth, powered by an STM32F103 microcontroller. The car can be controlled using an Android smartphone, offering users a convenient and flexible way to manage the car's movement.

Features
Wireless Control via Bluetooth: Uses the HC-05 Bluetooth module for communication between the phone and the car.
STM32F103 Microcontroller: Manages the car’s operations, processes signals, and controls the motors.
Custom Android App: User-friendly interface for easy car control.
DC Motor Control: Uses the L298 H-Bridge driver to control the speed and direction of the motors.
Key Components
STM32F103 Development Board
HC-05 Bluetooth Module
L298 Motor Driver
DC Motors and Wheels
Chassis and Power Supply
Setup Instructions
Hardware:

Connect the HC-05 Bluetooth module to the STM32F103 via UART.
Connect the L298 motor driver to the STM32F103 and the DC motors.
Assemble all components onto the chassis and connect the power supply.
Software:

Flash the firmware to the STM32F103 using STM32CubeIDE or Keil uVision.
Install the Android control app (available in the /app folder) on your smartphone.
