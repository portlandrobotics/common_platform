//***********************************
// PARTS Common Robot Platform
// SBUS Test Program  				 
//***********************************
#include <Wire.h> // Include the Wire library for I2C communication
#include <Arduino.h>
#include <sbus.h>

bfs::SbusRx sbus_rx(&Serial4, true); // Assuming Serial4 is connected to the SBUS receiver and the receiver is standard inverted sbus
bfs::SbusData data;

// Define constants to enable or disable various debug prints and features.
#define MOTORS_ENABLED 1      // Enable or disable motor operation


// Constants for motor control
const int MAX_PWM      = 100;        // Maximum PWM value for motor speed

// Pin assignments for the robot's hardware components
const int PIN_XD_EN   =  2;  // Pin for XD enable
const int PIN_RD_PWM1 =  3;  // Right drive motor, PWM pin 1
const int PIN_LD_PWM1 =  4;  // Left drive motor, PWM pin 1
const int PIN_RD_PWM2 =  5;  // Right drive motor, PWM pin 2
const int PIN_LD_PWM2 =  6;  // Left drive motor, PWM pin 2
const int PIN_VBAT    = 14;  // Battery voltage pin
const int PIN_LD_OCM  = 20;  // Left drive overcurrent monitor pin
const int PIN_RD_OCM  = 21;  // Right drive overcurrent monitor pin


// Define aliases for easier understanding of the code
#define RENC 0  // Right Encoder
#define RMOT RENC // Right Motor, same as right encoder
#define LENC 1  // Left Encoder
#define LMOT LENC // Left Motor, same as left encoder

// Arrays to hold encoder counts and speeds
const char *wheelName[2] = {"Right", "Left"}; // Array to store wheel names for easy reference
const int motorDir[2] = {1, -1};     // Array to set motor direction, 1 for forward, -1 for reverse
int motorPWMPin1[2] = {PIN_RD_PWM1, PIN_LD_PWM1}; // Array of PWM pins for each motor
int motorPWMPin2[2] = {PIN_RD_PWM2, PIN_LD_PWM2}; // Secondary PWM pins for each motor

// Setup function, runs once at the beginning
void setup() {
    // Set motor driver pins to output mode
    pinMode(PIN_XD_EN, OUTPUT);
    pinMode(PIN_RD_PWM1, OUTPUT);
    pinMode(PIN_LD_PWM1, OUTPUT);
    pinMode(PIN_RD_PWM2, OUTPUT);
    pinMode(PIN_LD_PWM2, OUTPUT);

    // Initialize motors to off state
    digitalWrite(PIN_XD_EN, HIGH);
    analogWrite(PIN_RD_PWM1,0);
    analogWrite(PIN_RD_PWM2,0);
    analogWrite(PIN_LD_PWM1,0);
    analogWrite(PIN_LD_PWM2,0);

    // Initialize SBUS
    sbus_rx.Begin();
}

// Main loop function, runs repeatedly
void loop() {
    int i;
    static int setspeed[2] = {0, 0}; // Array to store speed setpoints for motors

    if (sbus_rx.Read()) {
        data = sbus_rx.data();
        
        // Convert SBUS channel values to motor speed
        int setspeed[2];  // Array to hold motor speeds
        int linear = map(data.ch[1], 172, 1810, -MAX_PWM, MAX_PWM);  // Channel 2 for linear speed
        int angular = map(data.ch[0], 172, 1810, -MAX_PWM, MAX_PWM); // Channel 1 for angular speed

        // Calculate motor speeds for turning
        setspeed[0] = constrain(linear - angular, -MAX_PWM, MAX_PWM);  // Motor 0 speed
        setspeed[1] = constrain(linear + angular, -MAX_PWM, MAX_PWM);  // Motor 1 speed

        // Apply motor speeds
        for (i = 0; i < 2; i++) {
            // Control motor direction and speed using PWM
            if (setspeed[i] * motorDir[i] >= 0) {
                analogWrite(motorPWMPin1[i], setspeed[i] * motorDir[i]);
                analogWrite(motorPWMPin2[i], 0);
            } else {
                analogWrite(motorPWMPin1[i], 0);
                analogWrite(motorPWMPin2[i], -setspeed[i] * motorDir[i]);
            }
        }
    }
}

