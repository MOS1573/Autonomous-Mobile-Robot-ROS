// MPU6050 over I2C without external libraries.
// Gyro and accelerometer offsets are measured at start-up while the robot
// stands still, the same calibration step the handbook describes (Fig. 3.34).
#pragma once
#include <Arduino.h>
#include <Wire.h>

struct ImuSample {
  float ax, ay, az;  // m/s^2
  float gx, gy, gz;  // rad/s
};

class MPU6050 {
 public:
  bool begin() {
    Wire.begin();
    Wire.setClock(400000);
    write(0x6B, 0x00);  // PWR_MGMT_1: wake up
    write(0x1B, 0x00);  // GYRO_CONFIG: +-250 deg/s
    write(0x1C, 0x00);  // ACCEL_CONFIG: +-2 g
    write(0x1A, 0x03);  // CONFIG: 44 Hz low-pass filter
    Wire.beginTransmission(kAddress);
    Wire.write(0x75);   // WHO_AM_I
    Wire.endTransmission(false);
    Wire.requestFrom(kAddress, (uint8_t)1);
    return Wire.available() && Wire.read() == 0x68;
  }

  // Average N samples at rest. Gravity stays on z.
  void calibrate(int samples) {
    ImuSample sum = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < samples; i++) {
      ImuSample s = readRaw();
      sum.ax += s.ax; sum.ay += s.ay; sum.az += s.az;
      sum.gx += s.gx; sum.gy += s.gy; sum.gz += s.gz;
      delay(2);
    }
    offset_.ax = sum.ax / samples;
    offset_.ay = sum.ay / samples;
    offset_.az = sum.az / samples - kGravity;
    offset_.gx = sum.gx / samples;
    offset_.gy = sum.gy / samples;
    offset_.gz = sum.gz / samples;
  }

  ImuSample read() {
    ImuSample s = readRaw();
    s.ax -= offset_.ax; s.ay -= offset_.ay; s.az -= offset_.az;
    s.gx -= offset_.gx; s.gy -= offset_.gy; s.gz -= offset_.gz;
    return s;
  }

 private:
  static constexpr uint8_t kAddress = 0x68;
  static constexpr float kGravity = 9.80665f;
  static constexpr float kAccelScale = kGravity / 16384.0f;               // +-2 g
  static constexpr float kGyroScale = (3.14159265f / 180.0f) / 131.0f;    // +-250 deg/s

  ImuSample offset_ = {0, 0, 0, 0, 0, 0};

  void write(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(kAddress);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  }

  ImuSample readRaw() {
    Wire.beginTransmission(kAddress);
    Wire.write(0x3B);  // ACCEL_XOUT_H, then temp, then gyro
    Wire.endTransmission(false);
    Wire.requestFrom(kAddress, (uint8_t)14);
    int16_t v[7];
    for (int i = 0; i < 7; i++) v[i] = (Wire.read() << 8) | Wire.read();
    ImuSample s;
    s.ax = v[0] * kAccelScale; s.ay = v[1] * kAccelScale; s.az = v[2] * kAccelScale;
    s.gx = v[4] * kGyroScale;  s.gy = v[5] * kGyroScale;  s.gz = v[6] * kGyroScale;
    return s;
  }
};
