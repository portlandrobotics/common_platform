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

#include "setup.h"  // Include global configuration header
#include "PidControl.h"
#include "MotorControl.h"
#include "RobotState.h"
#include "Motion.h" // Include the new Motion header

#include <Wire.h>
// #include <math.h>

#if ROS
#include "RosInterface.h"
#endif  // ROS

// --- Global Objects ---
RobotState robotState;
MotorControl leftMotor(PIN_LD_OCM, PIN_LENCB, PIN_LENCA, PIN_LD_PWM2, PIN_LD_PWM1, "Left");
MotorControl rightMotor(PIN_RD_OCM, PIN_RENCA, PIN_RENCB, PIN_RD_PWM1, PIN_RD_PWM2, "Right");
Motion motion;

void robotState_reset() {
  robotState.targetLinearVelocity = 0.0f;
  robotState.targetAngularVelocity = 0.0f;
  robotState.move = false;
  // cmdDrive = true; // Should reset keep motors enabled? Maybe not.
#if !ROS
  robotState.leftTargetDistance = 0;
  robotState.rightTargetDistance = 0;
  robotState.targetHeading = 0.0f;
  robotState.isMoving = false;
  robotState.corr = 0.0f;
#endif
}

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
  }
  rightMotor.lastEnc = current_enc_val;
}


// --- ROS Specific Section ---
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
      robotState_reset();
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
  setup_ROS();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // LED off initially
  delay(1000);                 // Short delay before trying agent
  // state = WAITING_AGENT;
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
  robotState_reset();

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
  // TODO: figure out what to do for this message
  // snprintf(buf, sizeof(buf), "ROS State: %d | Target Lin=%.2f Ang=%.2f",
  //          currentRosAgentStatus, robotState.targetLinearVelocity, robotState.targetAngularVelocity);
  // SERIAL_OUT.println(buf);
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
  handleRosAgentState(&robotState);  // Handles connection and spins the executor
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
