/**
 * @struct RobotState
 * @brief Holds the robot's motion state, control flags, and target parameters.
 *
 * This struct encapsulates all relevant state information for robot motion control,
 * including target velocities, control flags, and (in non-ROS builds) additional
 * fields for distance-based moves and heading correction.
 *
 * Fields:
 * - targetLinearVelocity: Target linear velocity in meters per second.
 * - targetAngularVelocity: Target angular velocity in radians per second.
 * - cmdDrive: Motor enable command flag.
 * - move: Indicates if an active motion command is present.
 * - isMoving: (non-ROS) True if a distance-based move is in progress.
 * - corr: (non-ROS) Heading correction value.
 * - maxSpeed: (non-ROS) Maximum robot speed.
 * - leftTargetDistance, rightTargetDistance: (non-ROS) Target distances for each wheel.
 * - targetHeading: (non-ROS) Target heading in radians.
 *
 * The reset() method clears motion commands and resets state fields.
 */
#include "setup.h"
#include <stdint.h>
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

};

