//***********************************
// PARTS Common Robot Platform
// SBUS Test Program  				 
//***********************************

// Include necessary libraries
#include <Wire.h> // I2C communication library
#include <Arduino.h> // Main Arduino library
#include <sbus.h> // Library for handling SBUS communication

// Initialize SBUS receiver on Serial4 with inversion enabled (due to SBUS standard)
bfs::SbusRx sbus_rx(&Serial4, true); 
bfs::SbusData data; // Variable to store SBUS data

// Define constants for enabling/disabling features
#define MOTORS_ENABLED 1 // Enable (1) or disable (0) motor operation

// Constants for motor control
const int MAX_PWM = 200; // Maximum PWM value for motor speed, range 0-255

// Define pin assignments for various robot components
const int PIN_XD_EN = 2; // Pin for XD enable
const int PIN_RD_PWM1 = 3; // Right drive motor, PWM pin 1
const int PIN_LD_PWM1 = 4; // Left drive motor, PWM pin 1
const int PIN_RD_PWM2 = 5; // Right drive motor, PWM pin 2
const int PIN_LD_PWM2 = 6; // Left drive motor, PWM pin 2
const int PIN_VBAT = 14; // Battery voltage monitoring pin
const int PIN_LD_OCM = 20; // Left drive overcurrent monitor pin
const int PIN_RD_OCM = 21; // Right drive overcurrent monitor pin

// Arrays for motor control
const int motorDir[2] = {1, -1}; // Motor direction (1 for forward, -1 for reverse)
int motorPWMPin1[2] = {PIN_RD_PWM1, PIN_LD_PWM1}; // PWM pins for each motor
int motorPWMPin2[2] = {PIN_RD_PWM2, PIN_LD_PWM2}; // Secondary PWM pins for each motor

// Setup function - initializes the robot's components
void setup() {
    // Initialize Serial communication for debugging
    Serial.begin(9600); // Start serial communication at 9600 bits per second
    Serial.println("Setup complete. Robot initializing...");

    // Configure motor driver pins as outputs
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

    // Start SBUS communication
    sbus_rx.Begin();
}

void loop() {
    static int setspeed[2] = {0, 0}; // Array to store speed setpoints for motors
    static unsigned long lastUpdateTime = 0; // Store the last update time
    unsigned long currentMillis = millis(); // Get the current time

    // Read SBUS data
    if (sbus_rx.Read()) {
        data = sbus_rx.data(); // Update SBUS data

        // Convert SBUS channel values to motor speeds
        int linear = map(data.ch[1], 172, 1810, -MAX_PWM, MAX_PWM); // Channel 2 for linear speed
        int angular = map(data.ch[0], 172, 1810, -MAX_PWM, MAX_PWM); // Channel 1 for angular speed

        // Calculate motor speeds for turning
        setspeed[0] = constrain(linear - angular, -MAX_PWM, MAX_PWM);  // Speed for Motor 0
        setspeed[1] = constrain(linear + angular, -MAX_PWM, MAX_PWM);  // Speed for Motor 1

        // Apply calculated motor speeds
        for (int i = 0; i < 2; i++) {
            // Control motor direction and speed using PWM
            if (setspeed[i] * motorDir[i] >= 0) {
                analogWrite(motorPWMPin1[i], setspeed[i] * motorDir[i]);
                analogWrite(motorPWMPin2[i], 0);
            } else {
                analogWrite(motorPWMPin1[i], 0);
                analogWrite(motorPWMPin2[i], -setspeed[i] * motorDir[i]);
            }
        }

        // Update and log every 0.5 seconds
        if (currentMillis - lastUpdateTime >= 500) {
            lastUpdateTime = currentMillis; // Update the last update time

            // Log SBUS data
            Serial.print("SBUS Channel 1: ");
            Serial.print(data.ch[0]);
            Serial.print(", Channel 2: ");
            Serial.println(data.ch[1]);

            // Log calculated motor speeds
            Serial.print("Motor 0 Speed: ");
            Serial.print(setspeed[0]);
            Serial.print(", Motor 1 Speed: ");
            Serial.println(setspeed[1]);
        }
    } else {
        if (currentMillis - lastUpdateTime >= 500) {
            lastUpdateTime = currentMillis; // Update the last update time
            // Log if no SBUS data read
            Serial.println("No SBUS data received");
        }
    }
}


