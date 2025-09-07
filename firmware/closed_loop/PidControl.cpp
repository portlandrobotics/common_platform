// #include "setup.h"
#include "PidControl.h"
#include <Arduino.h>

PidControl::PidControl() { reset(); }

void PidControl::reset() {
  lastError = 0;
  errorSum = 0;
  lastTime = micros() / 1000000.0; // Convert to seconds
  lastDeriv = 0;
}

float PidControl::update(float error) {
  float currentTime = micros() / 1000000.0;
  float dt = currentTime - lastTime;
  if (dt <= 1e-6)
    dt = 0.001; // Prevent division by zero or tiny dt

  if (fabs(error) < deadband) {
    error = 0;
    // Optional: Decay integral term when in deadband to prevent slow drift
    // errorSum *= 0.99;
  }

  float pTerm = P * error;

  errorSum += error * dt;
  errorSum = constrain(errorSum, -maxErrorSum, maxErrorSum);
  float iTerm = I * errorSum;

  float deriv =
      (dt > 1e-6) ? (error - lastError) / dt : 0.0f; // Avoid division by zero
  lastDeriv = filterCoeff * deriv + (1 - filterCoeff) * lastDeriv;
  float dTerm = D * lastDeriv;

  float output = pTerm + iTerm + dTerm;
  output = constrain(output, -maxOutput, maxOutput);

  lastError = error;
  lastTime = currentTime;

  return output;
}

void PidControl::setLimits(float maxOut, float maxIntegral) {
  maxOutput = maxOut;
  maxErrorSum = maxIntegral;
}

void PidControl::setFiltering(float deadbandValue, float filterValue) {
  deadband = deadbandValue;
  filterCoeff = constrain(filterValue, 0, 1);
}
