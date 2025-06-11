#ifndef MOTION_H
#define MOTION_H

#include "MotorControl.h" // For MotorControl::METERS_PER_COUNT
#include "setup.h"        // For SERIAL_OUT, PRINT_MOVES
#include <Arduino.h>      // For int32_t, float, cos, sin, atan2, M_PI

/**
 * @brief Represents a point in 2D space.
 *
 * The Point class stores x and y coordinates as floating point values.
 * It provides constructors for default, parameterized, and copy initialization.
 */
class Point {
public:
  float x, y;

public:
  Point();
  Point(float ix, float iy);
  Point(const Point &o);
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
class Motion {
private:
  Point location;
  float theta;
  int32_t lastLeft, lastRight;

public:
  // Constants moved inside class
  static constexpr float TRACK_WIDTH = 0.142f;

  Motion();
  void reset();
  void update(int32_t newLeft, int32_t newRight);
  float getX() const;
  float getY() const;
  float getTheta() const; // Radians
  void print() const;
};

#endif // MOTION_H
