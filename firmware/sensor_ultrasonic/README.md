# Ultrasonic Sensor Test

## Overview
This project involves using an Arduino/Teensy to test an HC-SR04 ultrasonic sensor. The `ultrasonic_test.ino` sketch measures the distance to an object and prints the result to the Serial Monitor.

## Requirements
- Arduino Compatible Device (Teensy, Uno, Mega, or similar)
- HC-SR04 Ultrasonic Sensor
- Jumper Wires

## Wiring
Connect the ultrasonic sensor to the Teensy 4.0:
- VCC to 5V
- GND to GND
- TRIG to Pin 9
- ECHO to Pin 15
  
OR

- Hook sensor to ULTRASONIC1 on Common Robotics Platform Main Board. Pin 1 VCC, Pin 2 Trigger, Pin 3 Echo, Pin 4 Ground. Looking at the common robotics platform board from the top, find the label "ULTRASONIC1" - Pin 1 is toward the U on ULTRASONIC1.
  - Ensure R6 3.3k and R7 4.6k resistors are installed so we don't burn out the teensy.

## Installation
1. Connect the Teensy/Arduino to your computer.
2. Open the `ultrasonic_test.ino` sketch in the Arduino IDE.
3. Upload the sketch to your Arduino board.

## Usage
After uploading the sketch, open the Serial Monitor in the Arduino IDE. You should see distance measurements displayed in centimeters, updated approximately every half second.

## Troubleshooting
- Ensure all connections are secure.
- Verify that the correct COM port is selected in the Arduino IDE.
- Ensure that your Arduino board is compatible with the sketch.

For more information or assistance, refer to the documentation of the HC-SR04 ultrasonic sensor and the Teensy/Arduino board being used.
