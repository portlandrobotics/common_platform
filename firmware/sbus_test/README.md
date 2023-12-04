#SBUS Test Readme

-This uses a SBUS library https://github.com/bolderflight/sbus to provide SBUS control to our PARTS Robot.  

-How to use:

- Download and install the sbus library: https://github.com/bolderflight/sbus
- Hook up SBUS out on your receiver to an open Serial RX port on your Teensy.  In my example I used RX4 (Serial4) which is connected to I2C2 on our common platform board.
- Ensure that R8 on the common platform board is not connected.  
- Flash your Teensy with sbus_test.ino.
- Ensure your transmitter is setup to output on Channels 1 and 2.  If you are using standard AETR- then the right stick will control the robot.
- Ensure your transmitter is bound to your Reciever.
- Ensure your receiver is in SBUS mode and not CPPM
- Power it all up and try it out!
- Data is logged to usb serial 

-Important Parameters:

Which serial RX port are you using?  Find this bit of code:

```cpp 
bfs::SbusRx sbus_rx(&Serial4, true); //Setup SBUS using Serial4 (RX4), Using a standard SBUS inverted signal
``````

If you need to change which channels you are listening on- look for this:

```cpp
int linear = map(data.ch[1], 172, 1810, -MAX_PWM, MAX_PWM);  // Channel 2 for linear speed
int angular = map(data.ch[0], 172, 1810, -MAX_PWM, MAX_PWM); // Channel 1 for angular speed
```

Change Max speed? Change this:

```cpp // Constants for motor control
const int MAX_PWM      = 200;        // Maximum PWM value for motor speed 0-255
```



-Tested on:

Radiomaster Zorro jp4in1

FRSky R-XSR Sbus Receiver