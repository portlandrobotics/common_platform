#ifndef PID_CONTROL_H
#define PID_CONTROL_H

class PidControl {
private:
  float lastError = 0;
  float errorSum = 0;
  float lastTime = 0;
  float maxOutput = 1.0;    // Maximum output value
  float maxErrorSum = 1.0;  // Maximum integral windup
  float deadband = 0.01;    // Error deadband
  float filterCoeff = 0.1;  // Derivative low-pass filter coefficient (0-1)
  float lastDeriv = 0;      // Last derivative value for filtering
public:
  float P = 1, I = .1, D = .1;

  PidControl();
  void reset();
  float update(float error);
  void setLimits(float maxOut, float maxIntegral);
  void setFiltering(float deadbandValue, float filterValue);
};

#endif // PID_CONTROL_H