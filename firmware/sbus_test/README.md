# SBUS Test Readme

## Overview
This README provides instructions on using the SBUS library to integrate SBUS control into the PARTS Robot. The SBUS library can be found at [bolderflight/sbus](https://github.com/bolderflight/sbus).

## Prerequisites
- A working installation of Teensy microcontroller.
- Basic understanding of SBUS communication protocol.
- A compatible SBUS transmitter and receiver.

## Installation and Setup

1. **Library Installation**: Download and install the sbus library from [bolderflight/sbus](https://github.com/bolderflight/sbus).

2. **Hardware Connection**: 
   - Connect the SBUS out on your receiver to an open Serial RX port on your Teensy. 
   - In this example, RX4 (Serial4) on the teansy, which is also connected to I2C2 on the common platform board, is used. Connect the yellow wire from your reciever here. 
   - Connect the red wire to 5V and the black wire to ground.  
   - Ensure that R8 on the common platform board is disconnected.

3. **Firmware Upload**:
   - Flash your Teensy with the firmware. You can find the firmware file `sbus_test.ino` in this repository. Click [here](./sbus_test.ino) to view or download it.

4. **Transmitter Setup**:
   - Configure your transmitter to output on Channels 1 and 2. Standard AETR configuration means the right stick will control the robot.
   - Ensure your transmitter is bound to your receiver.

5. **Receiver Configuration**:
   - Make sure the receiver is in SBUS mode and not CPPM.

6. **Power and Test**:
   - Power up the system and test the functionality.
   - Data is logged to USB serial for monitoring.

## Important Parameters

- **Serial Port Configuration**:
  To change the serial port, modify the following line in your code:

  ```cpp 
  bfs::SbusRx sbus_rx(&Serial4, true); // Setup SBUS using Serial4 (RX4), using a standard SBUS inverted signal
  ```

- **Channel Configuration**:
  To change the channels you are monitoring:

  ```cpp
  int linear = map(data.ch[1], 172, 1810, -MAX_PWM, MAX_PWM);  // Channel 2 for linear speed
  int angular = map(data.ch[0], 172, 1810, -MAX_PWM, MAX_PWM); // Channel 1 for angular speed
  ```

- **Maximum Speed Setting**:
  To change the maximum speed:

  ```cpp
  const int MAX_PWM = 200;        // Maximum PWM value for motor speed 0-255
  ```

## Tested Hardware

- Radiomaster Zorro jp4in1
- FRSky R-XSR Sbus Receiver

## Contributing

For any suggestions or contributions, please create an issue or pull request on the GitHub repository.
