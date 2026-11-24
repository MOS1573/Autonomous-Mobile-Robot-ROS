// Planar odometry for a holonomic base. Pure C++ so it can be tested on a PC.
#pragma once
#include <math.h>

class Odometry {
 public:
  // Advance the pose by a body-frame velocity over dt seconds (midpoint rule).
  // Mecanum wheels move sideways too, so vy is rotated into the odom frame with vx.
  void update(float vx, float vy, float wz, float dt) {
    const float heading = theta_ + wz * dt / 2.0f;
    x_ += (vx * cosf(heading) - vy * sinf(heading)) * dt;
    y_ += (vx * sinf(heading) + vy * cosf(heading)) * dt;
    theta_ = atan2f(sinf(theta_ + wz * dt), cosf(theta_ + wz * dt));
  }

  float x() const { return x_; }
  float y() const { return y_; }
  float theta() const { return theta_; }

  // Quaternion (z, w) of the yaw angle; x and y are zero for a planar robot.
  float qz() const { return sinf(theta_ / 2.0f); }
  float qw() const { return cosf(theta_ / 2.0f); }

 private:
  float x_ = 0, y_ = 0, theta_ = 0;
};
