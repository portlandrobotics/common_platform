/**
 * @file closed_loop.ino
 * @brief Firmware for a closed-loop motor control system with optional ROS integration.
 *
 * This firmware is designed to control a robot with two motors, using PID controllers
 * for precise speed and position control. It supports two modes of operation:
 * - **Standalone Mode**: The robot operates independently, using serial commands for control.
 * - **ROS Mode**: The robot integrates with the Robot Operating System (ROS) for remote control
 *   and communication via the micro-ROS framework.
 *
 * ## Features:
 * - **Motor Control**: PID-based speed control for left and right motors.
 * - **Odometry**: Tracks the robot's position and orientation using encoder feedback.
 * - **Command Parsing**: Supports serial commands for motor control, PID tuning, and state reset.
 * - **ROS Integration**: Subscribes to `cmd_vel` messages for velocity commands and manages
 *   connection to a ROS agent.
 * - **Safety Features**: Includes overcurrent protection and motor disable functionality.
 * - **Status Reporting**: Periodically outputs robot status, including odometry and motor speeds.
 *
 * ## Code Structure:
 * - **Global Configuration**: Pin definitions, ROS mode selection, and debug settings.
 * - **Classes**:
 *   - `PidControl`: Implements a PID controller for motor speed control.
 *   - `MotorControl`: Manages motor speed, encoder feedback, and PWM output.
 *   - `Point` and `Motion`: Handle robot position and orientation tracking.
 * - **Global Objects**: Instances of `MotorControl`, `Motion`, and `RobotState` for managing
 *   the robot's state and behavior.
 * - **ROS-Specific Logic**: Handles ROS communication, including agent connection and message
 *   subscription.
 * - **Main Functions**:
 *   - `setup()`: Initializes hardware, ROS (if enabled), and robot state.
 *   - `loop()`: Main control loop, handling sensor updates, motor control, ROS communication,
 *     and serial command processing.
 *   - `drivecontrol()`: Calculates motor target speeds based on ROS or standalone commands.
 *   - `parseCommand()`: Processes serial commands for robot control and configuration.
 *
 * ## Usage:
 * - Set `#define ROS` to `1` for ROS mode or `0` for standalone mode.
 * - In standalone mode, use serial commands to control the robot (e.g., `M` to toggle motors,
 *   `F` to move forward, `L`/`R` to turn).
 * - In ROS mode, ensure a ROS agent is running and publishing `cmd_vel` messages.
 *
 * ## Notes:
 * - Ensure proper tuning of PID parameters for optimal performance.
 * - Adjust hardware-specific constants (e.g., encoder counts per revolution, wheel diameter)
 *   as needed for your robot.
 */

// Pin Definitions
const int PIN_XD_EN = 2;
const int PIN_RD_PWM1 = 3;
const int PIN_LD_PWM1 = 4;
const int PIN_RD_PWM2 = 5;
const int PIN_LD_PWM2 = 6;
const int PIN_LENCB = 7;
const int PIN_LENCA = 8;
const int PIN_US_TRIG = 9;
const int PIN_VBAT = 14;
const int PIN_US_ECHO = 15;
const int PIN_LD_OCM = 20;
const int PIN_RD_OCM = 21;
const int PIN_RENCB = 22;
const int PIN_RENCA = 23;
const int LED_PIN = 13;

// --- Configuration ---
#define ROS 1  // 0 for Teensy stand alone, 1 for ROS based firmware
#define PRINT_MOVES 0

// --- Conditional Includes and Serial Definition ---
#include <Wire.h>
// #include <math.h>

#if ROS
// ROS-specific includes
#include <micro_ros_arduino.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <stdio.h>
#include <geometry_msgs/msg/twist.h>

#define SERIAL_OUT SerialUSB1  // Use USB Serial for ROS

#else                      // !ROS
#define SERIAL_OUT Serial  // Use default Serial for non-ROS
#endif                     // ROS

// --- Common Definitions ---
#define RENC 0
#define RMOT RENC
#define LENC 1
#define LMOT LENC

// --- PID Control Class (Common) ---
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

  PidControl() {
    reset();
  }

  void reset() {
    lastError = 0;
    errorSum = 0;
    lastTime = micros() / 1000000.0;  // Convert to seconds
    lastDeriv = 0;
  }

  /**
 * Update the PID controller with the current error and calculate the control output.
 *
 * This function computes the output of a PID (Proportional, Integral, Derivative)
 * controller given a current error value. It calculates the proportional term,
 * updates the integral term while applying limits to prevent windup, and computes
 * the derivative term with filtering. The result is a control output that is
 * constrained within specified limits.
 *
 * @param error The current error value, which is the difference between the
 *              desired setpoint and the actual process variable.
 * @return The constrained control output from the PID controller.
 */
  float update(float error) {
    float currentTime = micros() / 1000000.0;
    float dt = currentTime - lastTime;
    if (dt <= 1e-6) dt = 0.001;  // Prevent division by zero or tiny dt

    if (fabs(error) < deadband) {
      error = 0;
      // Optional: Decay integral term when in deadband to prevent slow drift
      // errorSum *= 0.99;
    }

    float pTerm = P * error;

    errorSum += error * dt;
    errorSum = constrain(errorSum, -maxErrorSum, maxErrorSum);
    float iTerm = I * errorSum;

    float deriv = (dt > 1e-6) ? (error - lastError) / dt : 0.0f;  // Avoid division by zero
    lastDeriv = filterCoeff * deriv + (1 - filterCoeff) * lastDeriv;
    float dTerm = D * lastDeriv;

    float output = pTerm + iTerm + dTerm;
    output = constrain(output, -maxOutput, maxOutput);

    lastError = error;
    lastTime = currentTime;

    return output;
  }

  void setLimits(float maxOut, float maxIntegral) {
    maxOutput = maxOut;
    maxErrorSum = maxIntegral;
  }

  void setFiltering(float deadbandValue, float filterValue) {
    deadband = deadbandValue;
    filterCoeff = constrain(filterValue, 0, 1);
  }
};


// --- Motor Control Class (Common) ---
class MotorControl {
private:
  uint32_t lastEncTime;
  int8_t lastEnc;
  int32_t counter;
  // int32_t lastCounter; // Removed as unused
  bool sawEdge;

  float lastSpeed;
  float targetSpeed;
  float mpwm;  // Internal PID output state

  const int ciPinCurrent;
  const int ciPinEncA;
  const int ciPinEncB;
  const int ciPwmA;
  const int ciPwmB;

  int msgcnt;

  // Constants used only internally
  static constexpr float CURRENT_SCALE = 0.0064f;
  static constexpr float MAX_CURRENT = 2.0f;
  static constexpr float MAX_ACCELERATION = 2.0f;  // m/s^2 (Currently unused)

  void applyPWM(int pwmValue) {
    pwmValue = constrain(pwmValue, -255, 255);
    if (pwmValue >= 0) {
      analogWrite(ciPwmA, pwmValue);
      analogWrite(ciPwmB, 0);
    } else {
      analogWrite(ciPwmA, 0);
      analogWrite(ciPwmB, -pwmValue);
    }
  }

public:  // Public interface and constants
  // --- Public Constants ---
  // These constants define the physical properties and limits,
  // potentially needed by other parts of the system (like Motion or drivecontrol).
  static constexpr float COUNTS_PER_REV = 1440.0f;
  static constexpr float WHEEL_DIAMETER = 0.070f;  // meters
  static constexpr float METERS_PER_COUNT = (M_PI * WHEEL_DIAMETER) / COUNTS_PER_REV;
  static constexpr float MAX_SPEED = 1.0f;  // m/s

  // --- Public Member Objects ---
  PidControl pid;

  // --- Constructor ---
  MotorControl(int pinCurrent, int pinEncA, int pinEncB, int pinPwmA, int pinPwmB)
    : ciPinCurrent(pinCurrent), ciPinEncA(pinEncA), ciPinEncB(pinEncB),
      ciPwmA(pinPwmA), ciPwmB(pinPwmB) {
    resetCounter();  // Initialize state in constructor via reset
    msgcnt = 0;
  }

  // --- Public Methods ---
  int32_t getCounter() const {
    return counter;
  }
  float getSpeed() const {
    return lastSpeed;
  }
  float getTargetSpeed() const {
    return targetSpeed;
  }

  void setTargetSpeed(float speed) {
    // Constrain speed to the public MAX_SPEED limit
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

    // Acceleration limiting (optional, currently commented out)
    // float dt = 0.01f; // Assumed update rate for accel limit
    // float currentSpeed = getSpeed(); // Use measured speed for accel calc? Or targetSpeed? Using targetSpeed is simpler.
    // float accel = (speed - targetSpeed) / dt;
    // if (abs(accel) > MAX_ACCELERATION) { // Uses private constant
    //   speed = targetSpeed + constrain(accel, -MAX_ACCELERATION, MAX_ACCELERATION) * dt;
    // }

    if (targetSpeed != speed) {
      targetSpeed = speed;
      // Consider PID reset strategy carefully (e.g., only on sign change or zero crossing)
      // pid.reset(); // Resetting integrator on any change can be jerky

#if PRINT_MOVES
      SERIAL_OUT.print((ciPinCurrent == PIN_LD_OCM) ? "Left" : "Right");
      SERIAL_OUT.print(" New speed target: ");
      SERIAL_OUT.println(speed);
#endif
    }
  }

  void resetCounter() {
    noInterrupts();  // Ensure atomic update if using interrupts later
    counter = 0;
    // lastCounter = 0; // Removed as unused
    interrupts();
    lastSpeed = 0;
    targetSpeed = 0;
    mpwm = 0;
    lastEncTime = micros();
    lastEnc = 0;  // Initialize encoder state
    sawEdge = false;
    pid.reset();
  }

  void readCurrent() {
    int ocm = analogRead(ciPinCurrent);
    float current = ocm * CURRENT_SCALE;  // Uses private constant

    static float lastCurrentL = 0, lastCurrentR = 0;  // Track separately
    float &lastCurrent = (ciPinCurrent == PIN_LD_OCM) ? lastCurrentL : lastCurrentR;

    if (current > 0.1 || lastCurrent > 0.1) {  // Threshold to reduce noise print
#if PRINT_MOVES > 1                            // Make current printing higher verbosity
      SERIAL_OUT.print((ciPinCurrent == PIN_LD_OCM) ? "Left" : "Right");
      SERIAL_OUT.print(" motor current: ");
      SERIAL_OUT.print(current * 1000);
      SERIAL_OUT.println(" mA");
#endif
      if (current > MAX_CURRENT) {  // Uses private constant
        SERIAL_OUT.print("!!! OVER CURRENT on ");
        SERIAL_OUT.print((ciPinCurrent == PIN_LD_OCM) ? "Left" : "Right");
        SERIAL_OUT.print(": ");
        SERIAL_OUT.print(current);
        SERIAL_OUT.println(" A !!!");
        // Consider a more robust safety stop, maybe disable motors via enable pin?
        setTargetSpeed(0);
        mpwm = 0;     // Force PID output to zero
        applyPWM(0);  // Force PWM off immediately
      }
    }
    lastCurrent = current;
  }

  /**
 * Reads the quadrature encoder values to determine the current position and movement
 * direction of the motor. The function calculates the encoder state based on the digital
 * inputs and determines the direction of movement using an XOR method. If there is a change
 * in encoder state, it updates the counter and calculates the speed of movement in meters
 * per second. Speed calculation is skipped for very fast state transitions (to filter noise)
 * and on the first read after a timer rollover. The function also updates the last encoder
 * state and time regardless of movement.
 */
  void readSensor() {
    // Read quadrature encoder - This is common logic
    // State: A B -> val
    //        0 0 -> 0
    //        0 1 -> 1
    //        1 1 -> 3
    //        1 0 -> 2
    int8_t current_enc_val = (digitalRead(ciPinEncA) ? 2 : 0) | (digitalRead(ciPinEncB) ? 1 : 0);
    // XOR method: (last_A ^ current_B) - (current_A ^ last_B) gives direction change (-1, 0, +1)
    int8_t last_A = (lastEnc >> 1) & 1;
    int8_t last_B = lastEnc & 1;
    int8_t current_A = (current_enc_val >> 1) & 1;
    int8_t current_B = current_enc_val & 1;

    int8_t dir = (last_A ^ current_B) - (current_A ^ last_B);

    if (dir != 0) {  // Only update if there was a change
      auto now = micros();
      auto dt = now - lastEncTime;

      noInterrupts();  // Protect counter access
      counter += dir;
      interrupts();

      if (dt > 100) {  // Avoid calculating speed on very fast transitions (noise) or first read
        // Calculate speed in m/s using the public constant
        float speed = METERS_PER_COUNT * dir / (dt * 1e-6f);
        lastSpeed = speed;  // Update measured speed
        sawEdge = true;     // We saw movement
      } else if (dt == 0) {
        // First edge or timer rollover, don't calculate speed
      }

      lastEncTime = now;

#if PRINT_MOVES > 2  // Make encoder printing highest verbosity
      SERIAL_OUT.print("Encoder ");
      SERIAL_OUT.print((ciPinCurrent == PIN_LD_OCM) ? 'L' : 'R');
      SERIAL_OUT.print(": state=");
      SERIAL_OUT.print(current_enc_val);
      SERIAL_OUT.print(" dir=");
      SERIAL_OUT.print(dir);
      SERIAL_OUT.print(" dt=");
      SERIAL_OUT.print(dt);
      SERIAL_OUT.print(" speed=");
      SERIAL_OUT.println(lastSpeed);
#endif
    }
    lastEnc = current_enc_val;  // Update last state regardless of movement
  }

  /**
   * Update the motor control for a single motor, using the PID controller to
   * adjust the PWM output based on the target speed and the measured speed.
   *
   * This function should be called periodically to update the motor control.
   * It implements the common logic for updating the PID controller and
   * applying the resulting PWM output to the motor.
   *
   * If no edges have been seen since the last update (i.e., no movement), the
   * measured speed is assumed to be zero. A timeout of 50ms is used to detect
   * this condition.
   *
   * The PID controller is reset when the target speed is effectively zero.
   * This is to prevent the integrator from "winding up" when the motor is
   * stopped.
   *
   * Debug output is printed periodically if the motor is moving.
   */
  void motionUpdate() {
    // Common logic: Update PID based on target vs measured speed
    if (!sawEdge) {
      // If no edges seen since last update, assume speed is zero
      // Check time elapsed too? If dt is large, speed is definitely zero.
      auto now = micros();
      if (now - lastEncTime > 50000) {  // 50ms timeout for zero speed
        lastSpeed = 0;
      }
    }
    sawEdge = false;  // Reset edge flag for next interval

    float err = targetSpeed - lastSpeed;
    float pidOutput = pid.update(err);  // Get PID adjustment

    // Apply PID output to the internal PWM state 'mpwm'
    mpwm = pidOutput;  // Direct PID output (scaled -1 to 1)

    // Limit mpwm (already done by pid.update's constrain, but good practice)
    mpwm = constrain(mpwm, -1.0, 1.0);

    // Convert PID output (-1.0 to 1.0) to PWM value (-255 to 255)
    int setspeed = static_cast<int>(mpwm * 255.0f);

    // Stop motor if target speed is effectively zero
    if (abs(targetSpeed) < 1e-3) {
      mpwm = 0;
      setspeed = 0;
      pid.reset();  // Reset PID when stopped
    }

    applyPWM(setspeed);

    // Debug output
    if (msgcnt++ > 100 && abs(setspeed) > 0) {  // Only print if moving and periodically
#if PRINT_MOVES
      char message[100];  // Smaller buffer
      snprintf(message, sizeof(message), "%c: Tgt=%.2f Cur=%.2f Err=%.2f PID=%.2f PWM=%d",
               (ciPinCurrent == PIN_LD_OCM) ? 'L' : 'R',
               targetSpeed, lastSpeed, err, mpwm, setspeed);
      SERIAL_OUT.println(message);
#endif
      msgcnt = 0;
    }
  }
};


// --- Point Class (Common) ---
class Point {
  // private: // Make members public for easier access from Motion, or keep private and use getters/setters
public:
  float x, y;

public:
  Point() : x(0), y(0) {}
  Point(float ix, float iy) : x(ix), y(iy) {}
  Point(const Point &o) : x(o.x), y(o.y) {}

  float getX() const {
    return x;
  }
  float getY() const {
    return y;
  }
  void setX(float newX) {
    x = newX;
  }
  void setY(float newY) {
    y = newY;
  }

  Point &operator+=(const Point &o) {
    x += o.x;
    y += o.y;
    return *this;
  }
};

// --- Motion Class (Common, but used differently by ROS/non-ROS) ---
class Motion {
private:
  Point location;
  float theta;
  int32_t lastLeft, lastRight;

public:
  // Constants moved inside class
  static constexpr float TRACK_WIDTH = 0.142f;

  Motion() {
    reset();
  }  // Initialize in constructor

  void reset() {
    location = Point();
    theta = 0;
    lastLeft = 0;
    lastRight = 0;
  }

  /**
   * @brief Updates the motion state based on new encoder readings.
   *
   * This function calculates the distance traveled by the left and right wheels
   * using the difference in encoder counts from the last update. It then computes
   * the change in position and orientation of the robot based on these distances.
   *
   * @param newLeft The current encoder count for the left wheel.
   * @param newRight The current encoder count for the right wheel.
   *
   * The function updates the internal state of the Motion class, including the
   * position (x, y) and orientation (theta) of the robot. It also normalizes
   * the orientation to the range [-PI, PI).
   */
  void update(int32_t newLeft, int32_t newRight) {
    // Use the public constant from MotorControl
    float leftTravel = (newLeft - lastLeft) * MotorControl::METERS_PER_COUNT;
    float rightTravel = (newRight - lastRight) * MotorControl::METERS_PER_COUNT;

#if PRINT_MOVES > 1
    char pbuf[64];
    snprintf(pbuf, sizeof(pbuf), "Move dL=%.4f, dR=%.4f", leftTravel, rightTravel);
    SERIAL_OUT.println(pbuf);
#endif

    lastLeft = newLeft;
    lastRight = newRight;

    float deltaDist = (rightTravel + leftTravel) / 2.0f;
    float deltaTheta = (rightTravel - leftTravel) / TRACK_WIDTH;

    // Update position: Use average distance moved along the new heading
    // The new heading is approximately the old heading + deltaTheta / 2
    float avgTheta = theta + deltaTheta / 2.0f;
    // Assuming theta=0 is Y-axis (forward) based on original sin/cos usage.
    // If theta=0 is X-axis (ROS standard REP 103), swap sin/cos below.
    location.x += deltaDist * sin(avgTheta);
    location.y += deltaDist * cos(avgTheta);


    // Update heading
    theta += deltaTheta;
    // Normalize theta to [-PI, PI)
    theta = atan2(sin(theta), cos(theta));
  }

  float getX() const {
    return location.getX();
  }
  float getY() const {
    return location.getY();
  }
  float getTheta() const {
    return theta;
  }  // Radians

  void print() const {
    char buf[100];
    snprintf(buf, sizeof(buf), "Pose: x=%.3f m, y=%.3f m, theta=%.1f deg",
             location.getX(), location.getY(), theta * 180.0f / M_PI);
    SERIAL_OUT.println(buf);
  }
};

// --- Robot State (Common structure, different usage) ---
struct RobotState {
  // Target velocities (used by ROS, can be calculated in non-ROS)
  float targetLinearVelocity = 0.0f;   // m/s
  float targetAngularVelocity = 0.0f;  // rad/s

  // Target positions/heading (used by non-ROS)
  int32_t leftTargetDistance = 0;
  int32_t rightTargetDistance = 0;
  float targetHeading = 0.0f;  // Radians

  // Control flags/parameters
  bool cmdDrive = true;   // Motor enable command
  bool move = false;      // Flag indicating active motion command (either pos or vel)
  float maxSpeed = 0.2f;  // Overall max speed limit (m/s) - adjusted default
  float topSpeed = 0.2f;  // Current operational speed limit (non-ROS) - adjusted default

#if !ROS
  // Non-ROS specific state
  bool isMoving = false;  // Flag for distance-based moves (non-ROS)
  float corr = 0.0f;      // Heading correction value (non-ROS)
#endif

  void reset() {
    targetLinearVelocity = 0.0f;
    targetAngularVelocity = 0.0f;
    leftTargetDistance = 0;
    rightTargetDistance = 0;
    targetHeading = 0.0f;
    move = false;
    // cmdDrive = true; // Should reset keep motors enabled? Maybe not.
    // maxSpeed = 0.2f; // Keep configured max speed
    // topSpeed = 0.2f; // Keep configured top speed
#if !ROS
    isMoving = false;
    corr = 0.0f;
#endif
  }
};

// --- Global Objects ---
RobotState robotState;
MotorControl leftMotor(PIN_LD_OCM, PIN_LENCB, PIN_LENCA, PIN_LD_PWM2, PIN_LD_PWM1);
MotorControl rightMotor(PIN_RD_OCM, PIN_RENCA, PIN_RENCB, PIN_RD_PWM1, PIN_RD_PWM2);
Motion motion;




// --- ROS Specific Section ---
#if ROS

// ROS-specific Globals
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

// ROS-specific Macros
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
      return false; \
    } \
  }
#define EXECUTE_EVERY_N_MS(MS, X) \
  do { \
    static volatile int64_t init = -1; \
    if (init == -1) { \
      init = uxr_millis(); \
    } \
    if (uxr_millis() - init > MS) { \
      X; \
      init = uxr_millis(); \
    } \
  } while (0)

// ROS Agent States
enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

// ROS-specific function: Error loop
void error_loop(const char *msg, int line, int rc) {
  SERIAL_OUT.printf("ROS Error at %s:%d code %d\n", __FILE__, line, rc);
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ROS-specific function: Create ROS entities
bool create_entities() {
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg,
                                         &subscription_callback, ON_NEW_DATA));
  return true;
}

// ROS-specific function: Destroy ROS entities
void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

// Twist message callback
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg =
    (const geometry_msgs__msg__Twist *)msgin;
  // Directly update robot state from twist message
  robotState.targetLinearVelocity = msg->linear.x;
  robotState.targetAngularVelocity = msg->angular.z;
  robotState.move = (robotState.targetLinearVelocity != 0.0f || robotState.targetAngularVelocity != 0.0f);  // Update move flag

  digitalWrite(LED_PIN, (robotState.targetLinearVelocity == 0 && robotState.targetAngularVelocity == 0) ? LOW : HIGH);

#if PRINT_MOVES  // Keep debug print conditional
  SERIAL_OUT.print("Twist Received: LinX=");
  SERIAL_OUT.print(robotState.targetLinearVelocity);
  SERIAL_OUT.print(" AngZ=");
  SERIAL_OUT.println(robotState.targetAngularVelocity);
#endif
}

// Handle agent connection state machine
void handleRosAgentState() {
  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                        ? AGENT_AVAILABLE
                                        : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();  // Clean up if connection failed
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200,                                                 // Check connection more often?
                         state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))  // Faster ping
                                   ? AGENT_CONNECTED
                                   : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_US_TO_NS(10000));  // Spin more often (10ms)
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      // Reset speeds when disconnected?
      robotState.targetLinearVelocity = 0.0f;
      robotState.targetAngularVelocity = 0.0f;
      robotState.move = false;
      break;
    default:
      break;
  }

  // Optional: Visual indicator for ROS connection state (distinct from command LED)
  // Could use a different LED or blinking pattern if desired.
  // For now, just keep the LED tied to receiving non-zero commands via the callback.
}

#endif  // ROS Specific Section End

#if !ROS
// Differential PID only needed for non-ROS heading control during F commands
PidControl differential;
#endif

// --- Command Parsing ---
/**
 * @brief Parse and execute serial commands
 * Common commands: M, X
 * Non-ROS commands: L, R, F, V, P, I, D
 */
void parseCommand(const char *const cmd) {
  if (!cmd || !cmd[0]) {
    SERIAL_OUT.println("Error: Empty command");
    return;
  }

  float value = (cmd[1] != '\0') ? atof(cmd + 1) : 0.0f;
  bool commandProcessed = false;  // Flag to check if any case matched

  switch (cmd[0]) {
    case 'M':  // Toggle motor drive
      robotState.cmdDrive = !robotState.cmdDrive;
      digitalWrite(PIN_XD_EN, robotState.cmdDrive ? HIGH : LOW);  // Apply enable immediately
      SERIAL_OUT.print("Motor Drive ");
      SERIAL_OUT.println(robotState.cmdDrive ? "Enabled" : "Disabled");
      commandProcessed = true;
      break;

    case 'X':  // Reset state
      leftMotor.resetCounter();
      rightMotor.resetCounter();
      motion.reset();
      robotState.reset();
#if !ROS
      differential.reset();
#endif
      SERIAL_OUT.println("Reset state");
      commandProcessed = true;
      break;

#if !ROS
    // --- Non-ROS Specific Commands ---
    case 'L':  // Turn Left (degrees)
    case 'R':  // Turn Right (degrees)
      {
        float angleRad = value * M_PI / 180.0f;
        float wheelTravel = (angleRad * Motion::TRACK_WIDTH) / 2.0f;
        // Use public constant from MotorControl
        float counts = wheelTravel / MotorControl::METERS_PER_COUNT;

        noInterrupts();  // Protect target distance updates
        if (cmd[0] == 'L') {
          robotState.leftTargetDistance -= static_cast<int32_t>(counts);
          robotState.rightTargetDistance += static_cast<int32_t>(counts);
        } else {
          robotState.leftTargetDistance += static_cast<int32_t>(counts);
          robotState.rightTargetDistance -= static_cast<int32_t>(counts);
        }
        interrupts();
        robotState.isMoving = false;  // This is a turn, not forward motion
        robotState.move = true;       // A move command is active
        SERIAL_OUT.print("Turn ");
        SERIAL_OUT.print(value);
        SERIAL_OUT.println(" deg");
        commandProcessed = true;
      }
      break;

    case 'F':  // Move Forward/Backward (meters)
      {
        // Use public constant from MotorControl
        float counts = value / MotorControl::METERS_PER_COUNT;
        noInterrupts();  // Protect target distance updates
        robotState.leftTargetDistance += static_cast<int32_t>(counts);
        robotState.rightTargetDistance += static_cast<int32_t>(counts);
        interrupts();
        robotState.targetHeading = motion.getTheta();  // Target current heading
        robotState.isMoving = true;                    // This is forward motion
        robotState.move = true;                        // A move command is active
        differential.reset();                          // Reset heading PID for new move
        SERIAL_OUT.print("Move ");
        SERIAL_OUT.print(value);
        SERIAL_OUT.println(" m");
        commandProcessed = true;
      }
      break;

    case 'V':  // Set Max Speed (m/s)
      // Use public constant from MotorControl
      if (value > 0.0f && value <= MotorControl::MAX_SPEED) {
        robotState.maxSpeed = value;
        robotState.topSpeed = value;  // Also update topSpeed used in non-ROS control
        SERIAL_OUT.print("Max speed set to: ");
        SERIAL_OUT.println(value);
        commandProcessed = true;
      } else {
        SERIAL_OUT.print("Error: Speed must be > 0 and <= ");
        SERIAL_OUT.println(MotorControl::MAX_SPEED);  // Use public constant
      }
      break;

    // Differential PID Tuning Commands (Non-ROS)
    case 'P':
      differential.P = value;
      commandProcessed = true;
      SERIAL_OUT.print("Diff P=");
      SERIAL_OUT.println(value);
      break;
    case 'I':
      differential.I = value;
      commandProcessed = true;
      SERIAL_OUT.print("Diff I=");
      SERIAL_OUT.println(value);
      break;
    case 'D':
      differential.D = value;
      commandProcessed = true;
      SERIAL_OUT.print("Diff D=");
      SERIAL_OUT.println(value);
      break;

#endif  // !ROS Specific Commands

    default:
      SERIAL_OUT.print("Error: Unknown command '");
      SERIAL_OUT.print(cmd[0]);
      SERIAL_OUT.println("'");
      commandProcessed = true;  // We processed it by saying it's unknown
  }

  if (!commandProcessed) {
    // This case might happen if a command is defined for one mode but not the other
    SERIAL_OUT.print("Error: Command '");
    SERIAL_OUT.print(cmd[0]);
    SERIAL_OUT.println("' not available in this mode (ROS=");
    SERIAL_OUT.print(ROS);
    SERIAL_OUT.println(")");
  }
}

// --- Setup ---
void setup() {
  // Common Serial Initialization
  SERIAL_OUT.begin(115200);
  // A small delay to allow serial monitor to connect, especially for non-USB serial
  // delay(1000);
  SERIAL_OUT.println("--- Robot Firmware Starting ---");
  SERIAL_OUT.print("ROS Mode: ");
  SERIAL_OUT.println(ROS);

#if ROS
  // ROS Specific Setup
  SERIAL_OUT.println("Setting up micro-ROS...");
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // LED off initially
  delay(1000);                 // Short delay before trying agent
  state = WAITING_AGENT;
  SERIAL_OUT.println("Waiting for micro-ROS Agent...");
#else
  // Non-ROS Specific Setup
  pinMode(LED_PIN, OUTPUT);  // Still use LED for status maybe?
  digitalWrite(LED_PIN, LOW);
  SERIAL_OUT.println("Running in Standalone Mode.");
  // Configure differential PID (only used in non-ROS)
  differential.setLimits(0.5, 0.2);      // Limit correction effect
  differential.setFiltering(0.01, 0.1);  // Add some filtering
  differential.P = 0.8;                  // Might need tuning
  differential.I = 0.01;
  differential.D = 0.05;
#endif

  // Common Hardware Setup
  // Enable encoder pullups
  pinMode(PIN_LENCA, INPUT_PULLUP);
  pinMode(PIN_LENCB, INPUT_PULLUP);
  pinMode(PIN_RENCA, INPUT_PULLUP);
  pinMode(PIN_RENCB, INPUT_PULLUP);

  // Enable motor drive outputs
  pinMode(PIN_XD_EN, OUTPUT);
  pinMode(PIN_RD_PWM1, OUTPUT);
  pinMode(PIN_LD_PWM1, OUTPUT);
  pinMode(PIN_RD_PWM2, OUTPUT);
  pinMode(PIN_LD_PWM2, OUTPUT);

  // Ensure motors are off and enabled initially
  digitalWrite(PIN_XD_EN, HIGH);  // HIGH likely means enabled for common drivers
  analogWrite(PIN_RD_PWM1, 0);
  analogWrite(PIN_RD_PWM2, 0);
  analogWrite(PIN_LD_PWM1, 0);
  analogWrite(PIN_LD_PWM2, 0);

  // Configure Motor PIDs (Common)
  // These values likely need tuning!
  leftMotor.pid.setLimits(1.0, 0.5);      // Max output, max integral sum
  leftMotor.pid.setFiltering(0.01, 0.1);  // Deadband, D filter coeff
  leftMotor.pid.P = 1.5;                  // Increased P gain example
  leftMotor.pid.I = 0.1;                  // Small I gain example
  leftMotor.pid.D = 0.01;                 // Small D gain example

  rightMotor.pid.setLimits(1.0, 0.5);
  rightMotor.pid.setFiltering(0.01, 0.1);
  rightMotor.pid.P = 1.5;  // Should be similar to left
  rightMotor.pid.I = 0.1;
  rightMotor.pid.D = 0.01;

  // Reset state
  leftMotor.resetCounter();
  rightMotor.resetCounter();
  motion.reset();
  robotState.reset();

  // I2C Setup (Common - Assuming IMU or other I2C device)
  SERIAL_OUT.println("Initializing I2C and IMU...");
  Wire.begin();
  Wire.beginTransmission(0x68);  // MPU6050 default address
  Wire.write(0x6B);              // PWR_MGMT_1 register
  Wire.write(0x80);              // Reset device
  auto rv = Wire.endTransmission(true);
  SERIAL_OUT.print("IMU Reset returned: ");
  SERIAL_OUT.println(rv);
  delay(100);  // Wait for reset

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0x00);  // Wake up device
  rv = Wire.endTransmission(true);
  SERIAL_OUT.print("IMU Wakeup returned: ");
  SERIAL_OUT.println(rv);
  delay(100);

  // Add more IMU config if needed (e.g., setting ranges, DLPF)

  SERIAL_OUT.println("--- Setup Complete ---");
}

// --- Status Printing ---
void printStatus() {
  char buf[128];

  // Print Odometry
  motion.print();

  // Print Motor Speeds (Target vs Actual)
  snprintf(buf, sizeof(buf), "Speed L: Tgt=%.2f Act=%.2f | R: Tgt=%.2f Act=%.2f m/s",
           leftMotor.getTargetSpeed(), leftMotor.getSpeed(),
           rightMotor.getTargetSpeed(), rightMotor.getSpeed());
  SERIAL_OUT.println(buf);

#if !ROS
  // Print Non-ROS specific info
  snprintf(buf, sizeof(buf), "Encoders L: %ld/%ld | R: %ld/%ld",
           leftMotor.getCounter(), robotState.leftTargetDistance,
           rightMotor.getCounter(), robotState.rightTargetDistance);
  SERIAL_OUT.println(buf);

  snprintf(buf, sizeof(buf), "Control: MaxSpd=%.2f HeadTgt=%.1f HeadCorr=%.3f",
           robotState.maxSpeed, robotState.targetHeading * 180.0f / M_PI, robotState.corr);
  SERIAL_OUT.println(buf);
  snprintf(buf, sizeof(buf), "Diff PID: P=%.2f I=%.2f D=%.2f",
           differential.P, differential.I, differential.D);
  SERIAL_OUT.println(buf);
#else
  // Print ROS specific info (if any needed beyond speeds)
  snprintf(buf, sizeof(buf), "ROS State: %d | Target Lin=%.2f Ang=%.2f",
           state, robotState.targetLinearVelocity, robotState.targetAngularVelocity);
  SERIAL_OUT.println(buf);
#endif
}

/**
 * Controls the movement of the robot by calculating and setting the target speeds for its left and right motors.
 * The control logic depends on whether the robot is in ROS (Robot Operating System) mode or not.
 *
 * In ROS mode, it converts target linear and angular velocities to wheel velocities using the robot's track width,
 * and optionally applies a speed limit to prevent high speeds.
 *
 * In non-ROS mode, it implements a simple P-controller for position-based control towards target distances,
 * calculates the remaining distance for each wheel and adjusts the velocity accordingly, applies a threshold to stop
 * the robot near the target, and optionally corrects the heading during forward/backward motion using a differential PID controller.
 *
 * Ultimately, it sets the target speeds for the left and right motors, ensuring they are within the robot's maximum speed limits.
 */
void drivecontrol() {
  float leftVelocity = 0.0f;
  float rightVelocity = 0.0f;

  if (!robotState.cmdDrive) {
    // If drive explicitly disabled, ensure motors are set to zero speed
    leftMotor.setTargetSpeed(0.0f);
    rightMotor.setTargetSpeed(0.0f);
    return;  // Don't proceed with control logic
  }

#if ROS
  // ROS mode: Convert target Twist velocities (linear/angular) to wheel velocities
  // Use public constant from Motion
  leftVelocity = robotState.targetLinearVelocity - (robotState.targetAngularVelocity * Motion::TRACK_WIDTH / 2.0f);
  rightVelocity = robotState.targetLinearVelocity + (robotState.targetAngularVelocity * Motion::TRACK_WIDTH / 2.0f);

  // Apply overall speed limit if needed (optional, Twist could command high speeds)
  // float max_vel = max(abs(leftVelocity), abs(rightVelocity));
  // if (max_vel > robotState.maxSpeed) { // Uses RobotState maxSpeed
  //    float scale = robotState.maxSpeed / max_vel;
  //    leftVelocity *= scale;
  //    rightVelocity *= scale;
  // }

#else  // !ROS
  // Non-ROS mode: Position-based control towards target distances
  // This implements a simple P-controller for position with optional heading correction

  // Calculate remaining distance for each wheel
  int32_t leftError = robotState.leftTargetDistance - leftMotor.getCounter();
  int32_t rightError = robotState.rightTargetDistance - rightMotor.getCounter();

  // Simple P-control for position: velocity proportional to error (with ceiling)
  // Use a threshold to stop near the target
  const int32_t stopThreshold = 10;       // Encoder counts threshold to stop
  const int32_t slowDownThreshold = 200;  // Start slowing down this far away

  if (abs(leftError) > stopThreshold) {
    float speedFactor = constrain(static_cast<float>(abs(leftError)) / slowDownThreshold, 0.1f, 1.0f);  // Scale speed
    leftVelocity = robotState.topSpeed * speedFactor * ((leftError > 0) ? 1.0f : -1.0f);
  } else {
    leftVelocity = 0.0f;  // Stop if close enough
  }

  if (abs(rightError) > stopThreshold) {
    float speedFactor = constrain(static_cast<float>(abs(rightError)) / slowDownThreshold, 0.1f, 1.0f);  // Scale speed
    rightVelocity = robotState.topSpeed * speedFactor * ((rightError > 0) ? 1.0f : -1.0f);
  } else {
    rightVelocity = 0.0f;  // Stop if close enough
  }

  // Check if the overall move command is complete
  if (abs(leftError) <= stopThreshold && abs(rightError) <= stopThreshold) {
    robotState.move = false;  // Mark move as complete
    robotState.isMoving = false;
    leftVelocity = 0.0f;
    rightVelocity = 0.0f;
    // Reset targets? Or allow accumulation? Current code accumulates.
  }

  // Apply heading correction during forward/backward motion (isMoving flag)
  if (robotState.isMoving && robotState.move) {  // Only correct during F commands that are still active
    // Calculate heading error (target - current)
    float headingError = robotState.targetHeading - motion.getTheta();
    // Normalize heading error to [-PI, PI)
    headingError = atan2(sin(headingError), cos(headingError));

    // Update differential PID
    robotState.corr = differential.update(headingError);

    // Apply correction: slow down the faster wheel, speed up the slower one
    // Correction should be proportional to the desired speed to avoid overshooting at low speeds
    float baseSpeed = max(abs(leftVelocity), abs(rightVelocity));  // Use the larger of the two target speeds
    if (baseSpeed > 1e-3) {                                        // Avoid division by zero
      float correctionAmount = robotState.corr * baseSpeed;        // Scale correction by speed
      leftVelocity -= correctionAmount;
      rightVelocity += correctionAmount;
    }
    if (abs(robotState.corr) > 0.01) {  // Debug print for correction
      SERIAL_OUT.print("Heading Err: ");
      SERIAL_OUT.print(headingError * 180.0f / M_PI);
      SERIAL_OUT.print(" Corr: ");
      SERIAL_OUT.println(robotState.corr);
    }
  } else {
    // Not doing a forward move or move is complete, reset correction
    robotState.corr = 0;
    differential.reset();  // Reset PID if not actively correcting heading
  }

#endif  // ROS / !ROS

  // Apply final speed limits (using RobotState maxSpeed) and set motor targets
  leftVelocity = constrain(leftVelocity, -robotState.maxSpeed, robotState.maxSpeed);
  rightVelocity = constrain(rightVelocity, -robotState.maxSpeed, robotState.maxSpeed);

  leftMotor.setTargetSpeed(leftVelocity);
  rightMotor.setTargetSpeed(rightVelocity);
}

// --- Main Loop ---
void loop() {
  const auto nowMicros = micros();

  // --- Sensor Reading ---
  // Should ideally be done at a high, consistent rate, maybe interrupt driven?
  // For now, call them frequently in the loop.
  leftMotor.readSensor();
  rightMotor.readSensor();
  // Consider reading current less frequently if needed
  // leftMotor.readCurrent();
  // rightMotor.readCurrent();

  // --- Periodic Updates ---
  static auto nextMotionUpdate = nowMicros;
  const unsigned long motionUpdateInterval = 10000;  // 10ms = 100Hz
  if (nowMicros >= nextMotionUpdate) {
    // Update Odometry
    motion.update(leftMotor.getCounter(), rightMotor.getCounter());

    // Update Motor PIDs and apply PWM
    leftMotor.motionUpdate();
    rightMotor.motionUpdate();

    // Calculate next update time, handling potential rollover
    nextMotionUpdate = (nowMicros / motionUpdateInterval + 1) * motionUpdateInterval;
  }

  // --- Drive Control Logic ---
  // This determines the target speeds based on ROS commands or non-ROS targets
  drivecontrol();

  // --- ROS Handling ---
#if ROS
  handleRosAgentState();  // Handles connection and spins the executor
#endif

  // --- Serial Command Input ---
  static char cmdbuf[32];  // Increased buffer size
  static size_t cmdbufpos = 0;
  while (SERIAL_OUT.available()) {
    char c = SERIAL_OUT.read();
    if (c == '\n' || c == '\r') {
      if (cmdbufpos > 0) {         // Process only if buffer is not empty
        cmdbuf[cmdbufpos] = '\0';  // Null terminate
        SERIAL_OUT.print("Received Command: [");
        SERIAL_OUT.print(cmdbuf);
        SERIAL_OUT.println("]");
        parseCommand(cmdbuf);
        cmdbufpos = 0;  // Reset buffer
      }
    } else if (cmdbufpos < (sizeof(cmdbuf) - 1)) {
      cmdbuf[cmdbufpos++] = c;  // Add char to buffer
    } else {
      // Buffer overflow, discard command
      SERIAL_OUT.println("Error: Command too long");
      cmdbufpos = 0;  // Reset buffer
    }
  }

  // --- Periodic Status Output ---
  static auto nextStatusUpdate = nowMicros;
  const unsigned long statusUpdateInterval = 1000000;  // 1 second
  if (nowMicros >= nextStatusUpdate) {
    SERIAL_OUT.println("--- Status Update ---");
    printStatus();
    // Read and print battery voltage
    // Assuming a voltage divider: Vbat -> R1 -> ADC_PIN -> R2 -> GND
    // Voltage = ADC_reading * (AREF / ADC_resolution) * (R1 + R2) / R2
    // Example: Teensy 3.3V AREF, 10-bit ADC (1024), R1=10k, R2=10k -> Factor = 3.3/1024 * 2 = 0.006445
    // Example: Arduino 5V AREF, 10-bit ADC (1024), R1=10k, R2=2k -> Factor = 5.0/1024 * (12/2) = 0.0293
    // *** ADJUST THE FACTOR BELOW BASED ON YOUR HARDWARE ***
    // const float VOLTAGE_FACTOR = 0.0293; // Example for 5V Arduino, 10k/2k divider
    // float voltage = analogRead(PIN_VBAT) * VOLTAGE_FACTOR;
    int voltage = analogRead(PIN_VBAT);
    SERIAL_OUT.print("Voltage: ");
    SERIAL_OUT.print(voltage);
    SERIAL_OUT.println(" V");
    SERIAL_OUT.println("---------------------");

    nextStatusUpdate = (nowMicros / statusUpdateInterval + 1) * statusUpdateInterval;
  }

  // Small delay to prevent loop from running too fast if nothing else blocks
  // delayMicroseconds(100); // Optional: yield CPU slightly
}
