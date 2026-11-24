// Mecanum wheel kinematics, X configuration (rollers at +-45 deg).
// Pure C++ so it can be unit tested on a PC (see firmware/test).
#pragma once

struct WheelRPM {
  float front_left;
  float front_right;
  float rear_left;
  float rear_right;
};

struct Velocity {
  float linear_x;   // m/s, forward
  float linear_y;   // m/s, to the left
  float angular_z;  // rad/s, counter clockwise
};

class MecanumKinematics {
 public:
  MecanumKinematics(float wheel_diameter, float wheel_base_x, float wheel_base_y, float max_rpm)
      : radius_(wheel_diameter / 2.0f),
        lever_((wheel_base_x + wheel_base_y) / 2.0f),
        max_rpm_(max_rpm) {}

  // Body velocity -> wheel speeds. If any wheel would exceed max_rpm, all four
  // are scaled down together so the direction of travel is kept.
  WheelRPM inverse(const Velocity& v) const {
    const float k = 60.0f / (2.0f * kPi * radius_);  // (m/s) -> rpm
    WheelRPM w;
    w.front_left  = k * (v.linear_x - v.linear_y - lever_ * v.angular_z);
    w.front_right = k * (v.linear_x + v.linear_y + lever_ * v.angular_z);
    w.rear_left   = k * (v.linear_x + v.linear_y - lever_ * v.angular_z);
    w.rear_right  = k * (v.linear_x - v.linear_y + lever_ * v.angular_z);

    float peak = absf(w.front_left);
    if (absf(w.front_right) > peak) peak = absf(w.front_right);
    if (absf(w.rear_left) > peak)   peak = absf(w.rear_left);
    if (absf(w.rear_right) > peak)  peak = absf(w.rear_right);
    if (peak > max_rpm_) {
      const float s = max_rpm_ / peak;
      w.front_left *= s; w.front_right *= s; w.rear_left *= s; w.rear_right *= s;
    }
    return w;
  }

  // Measured wheel speeds -> body velocity (used for odometry).
  Velocity forward(const WheelRPM& w) const {
    const float k = (2.0f * kPi * radius_) / 60.0f;  // rpm -> (m/s)
    const float fl = w.front_left * k, fr = w.front_right * k;
    const float rl = w.rear_left * k,  rr = w.rear_right * k;
    Velocity v;
    v.linear_x  = ( fl + fr + rl + rr) / 4.0f;
    v.linear_y  = (-fl + fr + rl - rr) / 4.0f;
    v.angular_z = (-fl + fr - rl + rr) / (4.0f * lever_);
    return v;
  }

 private:
  static constexpr float kPi = 3.14159265358979f;
  static float absf(float x) { return x < 0 ? -x : x; }

  float radius_;
  float lever_;    // (lx + ly) / 2
  float max_rpm_;
};
