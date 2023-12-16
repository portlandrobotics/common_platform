//***********************************
// PARTS Common Robot Platform
// Ultrasonic Test Program  				 
//***********************************

// Include necessary libraries
#include <Arduino.h> // Main Arduino library

// Define the Trigger and Echo pin connections
const int triggerPin = 9;
const int echoPin = 15;

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);

  // Define pin modes
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // Clear the triggerPin condition
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  // Sets the triggerPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  long distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Delay 500 milliseconds before the next measurement
  delay(500);
}
