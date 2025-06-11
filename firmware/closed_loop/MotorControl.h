#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "PidControl.h"
#include <Arduino.h>

// Forward declaration of external pin constants
extern const int PIN_LD_OCM;

class MotorControl {
public: // Made public for ISR access, ensure volatile
  volatile uint32_t lastEncTime;
  volatile int8_t lastEnc; // Stores the last combined A/B state (0,1,2,3)
  volatile int32_t counter;

  float lastSpeed;
  float targetSpeed;
  float mpwm; // Internal PID output state

  int32_t prevCounterForSpeed; // Store counter from previous motionUpdate
  uint32_t prevTimeForSpeed;   // Store time from previous motionUpdate

  const int ciPinCurrent;
  const int ciPinEncA;
  const int ciPinEncB;
  const int ciPwmA;
  const int ciPwmB;
  const char *label;

  int msgcnt;

  // Constants used only internally
  static constexpr float CURRENT_SCALE = 0.0064f;
  static constexpr float MAX_CURRENT = 3.0f;
  static constexpr float MAX_ACCELERATION = 2.0f; // m/s^2 (Currently unused)

  void applyPWM(int pwmValue);

public: // Public interface and constants
  // --- Public Constants ---
  static constexpr float COUNTS_PER_REV = 1440.0f;
  static constexpr float WHEEL_DIAMETER = 0.070f; // meters
  static constexpr float METERS_PER_COUNT =
      (M_PI * WHEEL_DIAMETER) / COUNTS_PER_REV;
  static constexpr float MAX_SPEED = 2.0f; // m/s

  // --- Public Member Objects ---
  PidControl pid;

  // --- Constructor ---
  MotorControl(int pinCurrent, int pinEncA, int pinEncB, int pinPwmA,
               int pinPwmB, const char *label);

  // --- Public Methods ---
  int32_t getCounter() const;
  float getSpeed() const;
  float getTargetSpeed() const;
  void setTargetSpeed(float speed);
  void resetCounter();
  void readCurrent();
  void motionUpdate();
};

#endif // MOTOR_CONTROL_H
