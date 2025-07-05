
🔹 Code 1: Single Motor with micro-ROS and PID (Left Wheel)
This code runs a PID-controlled DC motor with encoder feedback on an ESP32 and communicates with ROS 2 using micro-ROS. It defines modular Motor, Encoder, and Controller classes to manage hardware and control logic. The encoder uses the ESP32Encoder library to measure angular position and speed, and a FreeRTOS task runs the PID loop every 10ms to maintain the desired speed. It publishes motor angle and velocity to ROS topics and listens for speed commands (l_cmd_vel) from ROS, making it ideal for real-time robotics applications like differential drive systems.

🔹 Code 2: Dual Motor Control with micro-ROS (Left and Right Wheels)
This version expands on the first by controlling both left and right motors. It uses custom encoder logic with attachInterrupt() to manually count pulses and calculate revolutions. Each motor has its own Motor, Encoder, and Controller object. micro-ROS is used to publish angle and speed of each wheel and receive independent velocity commands (l_cmd_vel and r_cmd_vel). Only the left PID loop runs in a FreeRTOS task, while the right loop is not explicitly updated in loop(). This setup supports full differential drive control and ROS integration for real-time robot motion.

🔹 Code 3: Standalone Single Motor Control via Serial (No ROS)
This is a simplified Arduino sketch that controls a single motor using a PID controller, but without ROS. Instead, it receives commands through the serial port (e.g., "L3.5" to set left motor speed to 3.5 rad/s). It uses custom encoder reading via interrupts and manually calculates angular speed and revolutions. The controller uses a low-pass filter on the speed for stability. It’s great for basic motor testing, tuning, or standalone control systems without ROS or networking.

