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
#define ROS 0  // 0 for Teensy stand alone, 1 for ROS based firmware
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

// For ROS, SerialUSB0 is used for communication with microROS agent
// and  SerialUSB1 is used for debugging output.
#define SERIAL_OUT SerialUSB1

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


/**
 * @class MotorControl
 * @brief Manages all aspects of a single motor's operation.
 *
 * This class encapsulates the functionality required to control one motor of the robot.
 * It handles:
 * - **Speed Control**: Implements a PID controller to achieve and maintain a target speed.
 * - **Encoder Feedback**: Reads encoder signals to determine the motor's current speed and
 *   accumulated rotation (counter). Encoder readings are processed by ISRs which update
 *   volatile members of this class.
 * - **PWM Output**: Generates PWM signals to drive the motor based on the PID controller's output.
 * - **Current Sensing**: Reads analog values from a current sensor to monitor motor current
 *   and implement overcurrent protection.
 * - **State Management**: Keeps track of target speed, measured speed, encoder counts,
 *   and PID state.
 * - **Physical Properties**: Defines constants related to the motor and wheel, such as
 *   counts per revolution and wheel diameter, which are used for converting between
 *   encoder counts and linear distance/speed.
 *
 * Instances of this class are created for both the left and right motors of the robot.
 */
class MotorControl {
public:  // Made public for ISR access, ensure volatile
  volatile uint32_t lastEncTime;
  volatile int8_t lastEnc;  // Stores the last combined A/B state (0,1,2,3)
  volatile int32_t counter;
  volatile bool sawEdge;

  float lastSpeed;
  float targetSpeed;
  float mpwm;  // Internal PID output state

  int32_t prevCounterForSpeed;  // Store counter from previous motionUpdate
  uint32_t prevTimeForSpeed;    // Store time from previous motionUpdate

  const int ciPinCurrent;
  const int ciPinEncA;
  const int ciPinEncB;
  const int ciPwmA;
  const int ciPwmB;

  int msgcnt;

  // Constants used only internally
  static constexpr float CURRENT_SCALE = 0.0064f;  // .5V per amp, 3.3V ref, 1024 counts, so 3.3V / 1024 counts / .5V/A = 6.4mA/count
  static constexpr float MAX_CURRENT = 3.0f;
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
  static constexpr float MAX_SPEED = 2.0f;  // m/s

  // --- Public Member Objects ---
  PidControl pid;

  // --- Constructor ---
  MotorControl(int pinCurrent, int pinEncA, int pinEncB, int pinPwmA, int pinPwmB)
    : ciPinCurrent(pinCurrent), ciPinEncA(pinEncA), ciPinEncB(pinEncB),
      ciPwmA(pinPwmA), ciPwmB(pinPwmB) {
    resetCounter();  // Initialize state in constructor via reset
    msgcnt = 0;
    prevCounterForSpeed = 0;      // Initialized by resetCounter if called after member init
    prevTimeForSpeed = micros();  // Initialized by resetCounter
  }

  // --- Public Methods ---
  int32_t getCounter() const {
    int32_t tempCounter;
    noInterrupts();
    tempCounter = counter;
    interrupts();
    return tempCounter;
  }
  float getSpeed() const {
    return lastSpeed;
  }

  float getTargetSpeed() const {
    return targetSpeed;
  }

  void setTargetSpeed(float speed) {
    // Constrain speed to the public MAX_SPEED limit
    // Constrain the motor maximum speed.  There is also a limit in RobotState which
    // should be the robot speed limit.
    speed = constrain(speed, -MotorControl::MAX_SPEED, MotorControl::MAX_SPEED);

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
    noInterrupts();
    counter = 0;
    lastEncTime = micros();
    // Initialize lastEnc based on current pin readings.
    // This is crucial if reset is called while interrupts might be active.
    lastEnc = (digitalRead(ciPinEncA) ? 2 : 0) | (digitalRead(ciPinEncB) ? 1 : 0);
    sawEdge = false;
    // Reset speed calculation variables
    prevCounterForSpeed = counter;  // After counter itself is reset
    interrupts();

    // These are not shared with ISRs, so can be set outside critical section
    lastSpeed = 0;
    targetSpeed = 0;
    mpwm = 0;
    pid.reset();
    // prevTimeForSpeed is for speed calculation interval, should be updated carefully
    // Setting it here aligns it with the reset.
    prevTimeForSpeed = lastEncTime;  // Align with lastEncTime from critical section
  }

  void readCurrent() {
    int ocm = analogRead(ciPinCurrent);
    float current = ocm * CURRENT_SCALE;  // convert to amps.

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
    uint32_t currentTime = micros();

    int32_t currentCounter;
    uint32_t currentLastEncTime;

    noInterrupts();                    // Protect reads of volatile variables shared with ISRs
    currentCounter = counter;          // Copy volatile counter
    currentLastEncTime = lastEncTime;  // Copy volatile time
    // The sawEdge flag (set by encoder ISRs) is reset to false at the
    // beginning of each motionUpdate cycle.
    sawEdge = false;
    interrupts();
    // Calculate speed based on counts over the motionUpdate interval
    float dtSpeedCalc = (currentTime - prevTimeForSpeed) * 1e-6f;  // in seconds
    int32_t deltaCount = currentCounter - prevCounterForSpeed;


    if (dtSpeedCalc > 1e-6f) {  // Avoid division by zero or tiny dt
      lastSpeed = (static_cast<float>(deltaCount) * METERS_PER_COUNT) / dtSpeedCalc;
    } else if (currentCounter == prevCounterForSpeed) {
      // If no time passed for robust calculation AND no counts,
      // rely on the timeout below to force zero if truly stopped.
    }
    // else: dtSpeedCalc is too small, but counts might have changed (very fast ticks).
    // lastSpeed would retain its value from the previous valid calculation.

    // Robust zero-speed detection:
    // If counter hasn't changed since last motionUpdate AND >50ms since *any* edge.
    if ((currentCounter == prevCounterForSpeed) && (currentTime - currentLastEncTime > 50000)) {
      lastSpeed = 0.0f;
    }

    // Update history for the next speed calculation cycle
    prevCounterForSpeed = currentCounter;
    prevTimeForSpeed = currentTime;

    float err = targetSpeed - lastSpeed;
    mpwm = pid.update(err);  // Get PID adjustment
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
      snprintf(message, sizeof(message), "%c: Tgt=%.2f Cur=%.2f Err=%.2f PIDOut=%.2f PWM=%d",
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
public:
  float x, y;

public:
  Point() : x(0), y(0) {}
  Point(float ix, float iy) : x(ix), y(iy) {}
  Point(const Point &o) : x(o.x), y(o.y) {}

  // Point &operator+=(const Point &o) {
};

/**
 * @class Motion
 * @brief Manages the robot's odometry, tracking its position and orientation.
 *
 * This class uses encoder readings from the left and right wheels to estimate
 * the robot's movement. It calculates the change in position (x, y coordinates)
 * and orientation (theta angle) based on a differential drive model.
 * The class provides methods to update the pose with new encoder data,
 * reset the pose to the origin, and retrieve the current pose components.
 * It also includes a constant for the robot's track width, which is crucial
 * for accurate odometry calculations.
 */
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
    location.x += deltaDist * cos(avgTheta);
    location.y += deltaDist * sin(avgTheta);

    // Update heading
    theta += deltaTheta;
    // Normalize theta to [-PI, PI)
    theta = atan2(sin(theta), cos(theta));
  }

  float getX() const {
    return location.x;
  }
  float getY() const {
    return location.y;
  }
  float getTheta() const {
    return theta;
  }  // Radians

  void print() const {
    char buf[100];
    snprintf(buf, sizeof(buf), "Pose: x=%.3f m, y=%.3f m, theta=%.1f deg",
             location.x, location.y, theta * 180.0f / M_PI);
    SERIAL_OUT.println(buf);
  }
};

// --- Robot State (Common structure, different usage) ---
struct RobotState {
  // Target velocities (used by ROS, can be calculated in non-ROS)
  float targetLinearVelocity = 0.0f;   // m/s
  float targetAngularVelocity = 0.0f;  // rad/s

  // Control flags/parameters
  bool cmdDrive = true;  // Motor enable command
  bool move = false;     // Flag indicating active motion command (either pos or vel)

#if !ROS
  // Non-ROS specific state
  bool isMoving = false;  // Flag for distance-based moves (non-ROS)
  float corr = 0.0f;      // Heading correction value (non-ROS)
  float maxSpeed = 0.5f;  // Overall robot speed (non-ROS)
  // Target positions/heading (used by non-ROS)
  int32_t leftTargetDistance = 0;
  int32_t rightTargetDistance = 0;
  float targetHeading = 0.0f;  // Radians

#endif

  void reset() {
    targetLinearVelocity = 0.0f;
    targetAngularVelocity = 0.0f;
    move = false;
    // cmdDrive = true; // Should reset keep motors enabled? Maybe not.
#if !ROS
    leftTargetDistance = 0;
    rightTargetDistance = 0;
    targetHeading = 0.0f;
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


// --- Interrupt Service Routines for Encoders ---
// These ISRs directly manipulate the public volatile members of the global motor objects.

void leftEncoderInterrupt() {
  // Read current state of Left Motor's Encoder A and B pins
  // Note: digitalRead inside ISR is generally acceptable on fast MCUs like Teensy
  int8_t current_enc_val = (digitalRead(PIN_LENCA) ? 2 : 0) | (digitalRead(PIN_LENCB) ? 1 : 0);

  // Quadrature logic
  int8_t last_A_bit = (leftMotor.lastEnc >> 1) & 1;
  int8_t last_B_bit = leftMotor.lastEnc & 1;
  int8_t current_A_bit = (current_enc_val >> 1) & 1;
  int8_t current_B_bit = current_enc_val & 1;
  int8_t dir = (last_A_bit ^ current_B_bit) - (current_A_bit ^ last_B_bit);

  if (dir != 0) {
    // Physical forward rotation of the left motor causes its counter to decrease
    // (due to A leading B with the current XOR logic), so we invert dir here
    // to make the counter increase for forward motion.
    leftMotor.counter -= dir;          // Changed from += to -= to flip the direction for the left motor
    leftMotor.lastEncTime = micros();  // Update time of the edge
    leftMotor.sawEdge = true;          // Indicate movement
  }
  leftMotor.lastEnc = current_enc_val;  // Update last known state
}

void rightEncoderInterrupt() {
  // Read current state of Right Motor's Encoder A and B pins
  int8_t current_enc_val = (digitalRead(PIN_RENCA) ? 2 : 0) | (digitalRead(PIN_RENCB) ? 1 : 0);

  // Quadrature logic
  int8_t last_A_bit = (rightMotor.lastEnc >> 1) & 1;
  int8_t last_B_bit = rightMotor.lastEnc & 1;
  int8_t current_A_bit = (current_enc_val >> 1) & 1;
  int8_t current_B_bit = current_enc_val & 1;
  int8_t dir = (last_A_bit ^ current_B_bit) - (current_A_bit ^ last_B_bit);

  if (dir != 0) {
    rightMotor.counter += dir;
    rightMotor.lastEncTime = micros();
    rightMotor.sawEdge = true;
  }
  rightMotor.lastEnc = current_enc_val;
}


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

#if PRINT_MOVES
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
        rclc_executor_spin_some(&executor, RCL_US_TO_NS(1000));
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
  pinMode(LED_PIN, OUTPUT);
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

  // Initialize lastEnc states BEFORE attaching interrupts
  // This ensures the first interrupt call has a valid previous state.
  leftMotor.lastEnc = (digitalRead(PIN_LENCA) ? 2 : 0) | (digitalRead(PIN_LENCB) ? 1 : 0);
  rightMotor.lastEnc = (digitalRead(PIN_RENCA) ? 2 : 0) | (digitalRead(PIN_RENCB) ? 1 : 0);
  uint32_t now = micros();
  leftMotor.lastEncTime = now;
  rightMotor.lastEncTime = now;
  leftMotor.sawEdge = false;
  rightMotor.sawEdge = false;

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
  leftMotor.pid.P = 8.0;
  leftMotor.pid.I = 0.8;
  leftMotor.pid.D = 0.00;

  rightMotor.pid.setLimits(1.0, 0.5);
  rightMotor.pid.setFiltering(0.01, 0.1);
  rightMotor.pid.P = 8.0;
  rightMotor.pid.I = 0.8;
  rightMotor.pid.D = 0.00;

  // Reset state
  leftMotor.resetCounter();
  rightMotor.resetCounter();
  motion.reset();
  robotState.reset();

  // Attach interrupts for encoders
  // Note: digitalPinToInterrupt() is necessary for mapping pin numbers to interrupt numbers.
  attachInterrupt(digitalPinToInterrupt(PIN_LENCA), leftEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_LENCB), leftEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RENCA), rightEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_RENCB), rightEncoderInterrupt, CHANGE);

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
    leftVelocity = robotState.maxSpeed * speedFactor * ((leftError > 0) ? 1.0f : -1.0f);
  } else {
    leftVelocity = 0.0f;  // Stop if close enough
  }

  if (abs(rightError) > stopThreshold) {
    float speedFactor = constrain(static_cast<float>(abs(rightError)) / slowDownThreshold, 0.1f, 1.0f);  // Scale speed
    rightVelocity = robotState.maxSpeed * speedFactor * ((rightError > 0) ? 1.0f : -1.0f);
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
#if PRINT_MOVES > 2
    if (abs(robotState.corr) > 0.01) {  // Debug print for correction
      SERIAL_OUT.print("Heading Err: ");
      SERIAL_OUT.print(headingError * 180.0f / M_PI);
      SERIAL_OUT.print(" Corr: ");
      SERIAL_OUT.println(robotState.corr);
    }
#endif
  } else {
    // Not doing a forward move or move is complete, reset correction
    robotState.corr = 0;
    differential.reset();  // Reset PID if not actively correcting heading
  }
  // Apply final speed limits (using RobotState maxSpeed)
  leftVelocity = constrain(leftVelocity, -robotState.maxSpeed, robotState.maxSpeed);
  rightVelocity = constrain(rightVelocity, -robotState.maxSpeed, robotState.maxSpeed);

#endif  // ROS / !ROS


  // set motor speeds
  leftMotor.setTargetSpeed(leftVelocity);
  rightMotor.setTargetSpeed(rightVelocity);
}

// --- Main Loop ---
void loop() {
  const auto nowMicros = micros();

  leftMotor.readCurrent();
  rightMotor.readCurrent();

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
    const float VOLTAGE_FACTOR = 0.00967;  // Teensy 3.3V AREF, R!=2M, R2=1M -> Factor = (3.3/1024) * 3 = 0.00967
    float voltage = analogRead(PIN_VBAT) * VOLTAGE_FACTOR;
    SERIAL_OUT.print("Voltage: ");
    SERIAL_OUT.print(voltage);
    SERIAL_OUT.println(" V");
    SERIAL_OUT.println("---------------------");

    nextStatusUpdate = (nowMicros / statusUpdateInterval + 1) * statusUpdateInterval;
  }
}
