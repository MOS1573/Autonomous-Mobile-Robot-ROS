// Robot configuration for the AMR base controller (Teensy 4.0).
// Values marked (handbook) come from the project handbook; the rest are
// starting points to tune on the real robot.
#pragma once

// ---------------------------------------------------------------------------
// Geometry
// ---------------------------------------------------------------------------
#define WHEEL_DIAMETER      0.196f   // m, custom mecanum wheel, Rw = 98 mm (handbook)
#define WHEEL_BASE_X        0.300f   // m, front to rear axle distance
#define WHEEL_BASE_Y        0.360f   // m, left to right wheel distance (track)

// ---------------------------------------------------------------------------
// Drive train
// ---------------------------------------------------------------------------
#define MAX_RPM             55       // geared DC motor, 55 rpm at 24 V (handbook)
#define ENCODER_PPR         600      // YT06-OP quadrature encoder (handbook)
#define COUNTS_PER_REV      (ENCODER_PPR * 4)   // x4 decoding of A/B channels
#define PWM_BITS            10
#define PWM_MAX             ((1 << PWM_BITS) - 1)
#define PWM_FREQUENCY       20000    // Hz, above audible range for BTS7960

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------
#define CONTROL_RATE_HZ     20
#define CMD_TIMEOUT_MS      400      // stop if no /cmd_vel for this long

#define PID_KP              0.6f
#define PID_KI              0.3f
#define PID_KD              0.5f

// ---------------------------------------------------------------------------
// Pins. Motor 1 matches the wiring diagram in the handbook (Fig. 2.21);
// motors 2 to 4 follow the same pattern on free PWM / interrupt pins.
// BTS7960: R_EN and L_EN are tied to 5 V, IN_A -> RPWM, IN_B -> LPWM.
// ---------------------------------------------------------------------------
//                       front
//            MOTOR1 (FL)      MOTOR2 (FR)
//            MOTOR3 (RL)      MOTOR4 (RR)
//                       rear
#define MOTOR1_IN_A   1
#define MOTOR1_IN_B   0
#define MOTOR1_ENC_A  15
#define MOTOR1_ENC_B  14

#define MOTOR2_IN_A   5
#define MOTOR2_IN_B   6
#define MOTOR2_ENC_A  11
#define MOTOR2_ENC_B  12

#define MOTOR3_IN_A   22
#define MOTOR3_IN_B   23
#define MOTOR3_ENC_A  17
#define MOTOR3_ENC_B  16

#define MOTOR4_IN_A   2
#define MOTOR4_IN_B   3
#define MOTOR4_ENC_A  9
#define MOTOR4_ENC_B  10

// Set to -1 on a wheel whose motor or encoder turns the wrong way.
#define MOTOR1_DIR   1
#define MOTOR2_DIR  -1
#define MOTOR3_DIR   1
#define MOTOR4_DIR  -1

// MPU6050 on I2C (SDA 18, SCL 19 on Teensy 4.0)
#define IMU_CALIBRATION_SAMPLES 500
