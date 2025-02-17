Motor Control with PID and Encoder Feedback

Description:

This project implements motor control system using PID (Proportional-Integral-Derivative) control with encoder feedback for precise speed regulation. It is designed to work with an Arduino-based setup, controlling two DC motor using PWM signals and reading encoder data for closed-loop feedback.

Features

Motor Control Class: Handles motor initialization, direction control, and speed control via PWM.

Sensor (Encoder) Class: Reads quadrature encoder signals to measure position and speed.

PID Controller Class: Implements a PID control algorithm for accurate speed regulation.

Serial Input Handling: Allows speed commands via serial communication.

Low-Pass Filtering: Smooths out speed measurements to reduce noise.

Hardware Requirements

Arduino board

DC motor with an H-Bridge driver (L298N, BTS7960, etc.)

Quadrature encoder

Power supply suitable for the motor

Connection cables

Pin Configuration

Motor Control:

ENA (PWM) -> Pin 9

IN1 -> Pin 7

IN2 -> Pin 8

Encoder:

Channel A -> Pin 2 (Interrupt)

Channel B -> Pin 3

Installation

Clone this repository:

git clone https://github.com/yourusername/motor-control.git

Open the project in Arduino IDE.

Upload the sketch to your Arduino board.

Usage

Connect the motor, encoder, and driver as per the pin configuration.

Open the serial monitor (9600 baud rate) and send commands:

L3.0 → Sets left motor speed to 3 rad/s.

R2.5 → Sets right motor speed to 2.5 rad/s (if implemented).

Observe the motor responding to the speed commands with PID regulation.

Code Structure

Motor class: Controls motor direction and speed.

Sensor class: Reads encoder signals and calculates speed.

Controller class: Implements PID control for speed regulation.

readSerial(): Handles serial input to update desired speed.

loop(): Runs the PID control loop.
