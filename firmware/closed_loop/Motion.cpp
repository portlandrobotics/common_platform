#include "Motion.h"
#include "setup.h"      // For SERIAL_OUT, PRINT_MOVES
#include <stdio.h>      // For snprintf
#include <math.h>       // For M_PI, cos, sin, atan2

// --- Point Class Implementation ---
Point::Point() : x(0), y(0) {}
Point::Point(float ix, float iy) : x(ix), y(iy) {}
Point::Point(const Point &o) : x(o.x), y(o.y) {}

// --- Motion Class Implementation ---
Motion::Motion() {
  reset();
}

void Motion::reset() {
  location = Point(); // Re-initializes location to (0,0)
  theta = 0;
  lastLeft = 0;
  lastRight = 0;
}

void Motion::update(int32_t newLeft, int32_t newRight) {
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

float Motion::getX() const {
  return location.x;
}
float Motion::getY() const {
  return location.y;
}
float Motion::getTheta() const {
  return theta;
}
void Motion::print() const {
  char buf[100];
  snprintf(buf, sizeof(buf), "Pose: x=%.3f m, y=%.3f m, theta=%.1f deg",
           location.x, location.y, theta * 180.0f / M_PI);
  SERIAL_OUT.println(buf);
}