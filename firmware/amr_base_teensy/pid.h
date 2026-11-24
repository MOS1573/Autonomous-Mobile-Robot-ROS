// Velocity PID for one wheel. Output is a signed PWM duty in [-out_max, out_max].
#pragma once

class PID {
 public:
  PID(float kp, float ki, float kd, float out_max)
      : kp_(kp), ki_(ki), kd_(kd), out_max_(out_max) {}

  float compute(float setpoint, float measured) {
    const float error = setpoint - measured;
    integral_ += error;
    // Anti-windup: keep the integral term inside the output range.
    if (ki_ > 0) {
      const float limit = out_max_ / ki_;
      if (integral_ > limit) integral_ = limit;
      if (integral_ < -limit) integral_ = -limit;
    }
    const float derivative = error - prev_error_;
    prev_error_ = error;

    float out = kp_ * error + ki_ * integral_ + kd_ * derivative;
    if (out > out_max_) out = out_max_;
    if (out < -out_max_) out = -out_max_;
    // Do not hold torque when the wheel is meant to stand still.
    if (setpoint == 0 && measured == 0) { integral_ = 0; out = 0; }
    return out;
  }

  void reset() { integral_ = 0; prev_error_ = 0; }

 private:
  float kp_, ki_, kd_, out_max_;
  float integral_ = 0;
  float prev_error_ = 0;
};
