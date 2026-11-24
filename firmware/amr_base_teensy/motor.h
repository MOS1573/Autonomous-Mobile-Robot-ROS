// BTS7960 (IBT_2) H-bridge: one PWM pin per direction, enables tied high.
#pragma once
#include <Arduino.h>

class Motor {
 public:
  Motor(uint8_t in_a, uint8_t in_b, int8_t direction)
      : in_a_(in_a), in_b_(in_b), direction_(direction) {}

  void begin(uint32_t pwm_frequency, uint8_t pwm_bits) {
    pinMode(in_a_, OUTPUT);
    pinMode(in_b_, OUTPUT);
    analogWriteFrequency(in_a_, pwm_frequency);
    analogWriteFrequency(in_b_, pwm_frequency);
    analogWriteResolution(pwm_bits);
    stop();
  }

  // duty in [-PWM_MAX, PWM_MAX]; sign selects the direction.
  void spin(int duty) {
    duty *= direction_;
    if (duty > 0) {
      analogWrite(in_b_, 0);
      analogWrite(in_a_, duty);
    } else if (duty < 0) {
      analogWrite(in_a_, 0);
      analogWrite(in_b_, -duty);
    } else {
      stop();
    }
  }

  void stop() {
    analogWrite(in_a_, 0);
    analogWrite(in_b_, 0);
  }

 private:
  uint8_t in_a_, in_b_;
  int8_t direction_;
};
