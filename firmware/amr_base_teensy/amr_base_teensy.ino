// AMR base controller: Teensy 4.0 + micro-ROS (ROS 2 Humble).
//
//   subscribes  /cmd_vel           geometry_msgs/Twist     target body velocity
//   publishes   /odom/unfiltered   nav_msgs/Odometry       wheel odometry from the encoders
//   publishes   /imu/data          sensor_msgs/Imu         calibrated accel + gyro
//
// robot_localization on the PC fuses both into /odometry/filtered and the
// odom -> base_footprint transform (see ros2/src/amr_bringup).
//
// Libraries: micro_ros_arduino (humble), Encoder (PJRC, ships with Teensyduino).
// Connect with:  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

#include <Encoder.h>
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>

#include "config.h"
#include "imu.h"
#include "kinematics.h"
#include "motor.h"
#include "odometry.h"
#include "pid.h"

// ---------------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------------
Motor motor_fl(MOTOR1_IN_A, MOTOR1_IN_B, MOTOR1_DIR);
Motor motor_fr(MOTOR2_IN_A, MOTOR2_IN_B, MOTOR2_DIR);
Motor motor_rl(MOTOR3_IN_A, MOTOR3_IN_B, MOTOR3_DIR);
Motor motor_rr(MOTOR4_IN_A, MOTOR4_IN_B, MOTOR4_DIR);

Encoder enc_fl(MOTOR1_ENC_A, MOTOR1_ENC_B);
Encoder enc_fr(MOTOR2_ENC_A, MOTOR2_ENC_B);
Encoder enc_rl(MOTOR3_ENC_A, MOTOR3_ENC_B);
Encoder enc_rr(MOTOR4_ENC_A, MOTOR4_ENC_B);

PID pid_fl(PID_KP, PID_KI, PID_KD, PWM_MAX);
PID pid_fr(PID_KP, PID_KI, PID_KD, PWM_MAX);
PID pid_rl(PID_KP, PID_KI, PID_KD, PWM_MAX);
PID pid_rr(PID_KP, PID_KI, PID_KD, PWM_MAX);

MecanumKinematics kinematics(WHEEL_DIAMETER, WHEEL_BASE_X, WHEEL_BASE_Y, MAX_RPM);
Odometry odometry;
MPU6050 imu;
bool imu_ok = false;

Velocity target = {0, 0, 0};
uint32_t last_cmd_ms = 0;
uint32_t last_control_ms = 0;

// ---------------------------------------------------------------------------
// micro-ROS
// ---------------------------------------------------------------------------
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t control_timer;
rcl_subscription_t cmd_sub;
rcl_publisher_t odom_pub;
rcl_publisher_t imu_pub;

geometry_msgs__msg__Twist cmd_msg;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;

enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState state = WAITING_AGENT;

#define RCCHECK(fn) { if ((fn) != RCL_RET_OK) return false; }

void stamp(builtin_interfaces__msg__Time& t) {
  const int64_t ns = rmw_uros_epoch_nanos();
  t.sec = ns / 1000000000LL;
  t.nanosec = ns % 1000000000LL;
}

void stopAll() {
  motor_fl.stop(); motor_fr.stop(); motor_rl.stop(); motor_rr.stop();
  pid_fl.reset(); pid_fr.reset(); pid_rl.reset(); pid_rr.reset();
  target = {0, 0, 0};
}

// Wheel speed in rpm from the encoder count change since the last call.
float measureRpm(Encoder& enc, int32_t& last_count, float dt_s, int8_t direction) {
  const int32_t count = enc.read();
  const int32_t delta = count - last_count;
  last_count = count;
  return direction * (delta / (float)COUNTS_PER_REV) * (60.0f / dt_s);
}

void onCmdVel(const void* msgin) {
  const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*)msgin;
  target.linear_x = msg->linear.x;
  target.linear_y = msg->linear.y;
  target.angular_z = msg->angular.z;
  last_cmd_ms = millis();
}

void publishOdometry(const Velocity& v) {
  stamp(odom_msg.header.stamp);
  odom_msg.pose.pose.position.x = odometry.x();
  odom_msg.pose.pose.position.y = odometry.y();
  odom_msg.pose.pose.orientation.z = odometry.qz();
  odom_msg.pose.pose.orientation.w = odometry.qw();
  odom_msg.twist.twist.linear.x = v.linear_x;
  odom_msg.twist.twist.linear.y = v.linear_y;
  odom_msg.twist.twist.angular.z = v.angular_z;
  rcl_publish(&odom_pub, &odom_msg, NULL);
}

void publishImu() {
  const ImuSample s = imu.read();
  stamp(imu_msg.header.stamp);
  imu_msg.linear_acceleration.x = s.ax;
  imu_msg.linear_acceleration.y = s.ay;
  imu_msg.linear_acceleration.z = s.az;
  imu_msg.angular_velocity.x = s.gx;
  imu_msg.angular_velocity.y = s.gy;
  imu_msg.angular_velocity.z = s.gz;
  rcl_publish(&imu_pub, &imu_msg, NULL);
}

// Runs at CONTROL_RATE_HZ: PID per wheel, odometry, IMU.
void controlCallback(rcl_timer_t* timer, int64_t /*last_call_time*/) {
  if (timer == NULL) return;
  static int32_t last_fl = 0, last_fr = 0, last_rl = 0, last_rr = 0;

  const uint32_t now = millis();
  const float dt = (now - last_control_ms) / 1000.0f;
  last_control_ms = now;
  if (dt <= 0) return;

  // Safety: stop when the PC stops sending commands.
  if (now - last_cmd_ms > CMD_TIMEOUT_MS) target = {0, 0, 0};

  const WheelRPM goal = kinematics.inverse(target);
  WheelRPM measured;
  measured.front_left  = measureRpm(enc_fl, last_fl, dt, MOTOR1_DIR);
  measured.front_right = measureRpm(enc_fr, last_fr, dt, MOTOR2_DIR);
  measured.rear_left   = measureRpm(enc_rl, last_rl, dt, MOTOR3_DIR);
  measured.rear_right  = measureRpm(enc_rr, last_rr, dt, MOTOR4_DIR);

  motor_fl.spin(pid_fl.compute(goal.front_left,  measured.front_left));
  motor_fr.spin(pid_fr.compute(goal.front_right, measured.front_right));
  motor_rl.spin(pid_rl.compute(goal.rear_left,   measured.rear_left));
  motor_rr.spin(pid_rr.compute(goal.rear_right,  measured.rear_right));

  const Velocity v = kinematics.forward(measured);
  odometry.update(v.linear_x, v.linear_y, v.angular_z, dt);
  publishOdometry(v);
  if (imu_ok) publishImu();
}

void initMessages() {
  odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
  odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_footprint");
  // Diagonal covariances (x, y, z, roll, pitch, yaw). Rollers slip sideways,
  // so lateral motion is trusted less than forward motion.
  odom_msg.pose.covariance[0] = 0.001;
  odom_msg.pose.covariance[7] = 0.002;
  odom_msg.pose.covariance[35] = 0.01;
  odom_msg.twist.covariance[0] = 0.001;
  odom_msg.twist.covariance[7] = 0.004;
  odom_msg.twist.covariance[35] = 0.02;

  imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");
  imu_msg.orientation_covariance[0] = -1;  // orientation not provided
  for (int i = 0; i < 9; i += 4) {
    imu_msg.angular_velocity_covariance[i] = 0.0004;
    imu_msg.linear_acceleration_covariance[i] = 0.04;
  }
}

bool createEntities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "amr_base", "", &support));
  RCCHECK(rclc_subscription_init_default(&cmd_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_publisher_init_best_effort(&odom_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom/unfiltered"));
  RCCHECK(rclc_publisher_init_best_effort(&imu_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data"));
  RCCHECK(rclc_timer_init_default(&control_timer, &support,
      RCL_MS_TO_NS(1000 / CONTROL_RATE_HZ), controlCallback));
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, &onCmdVel, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  rmw_uros_sync_session(10);  // align timestamps with the PC clock
  last_control_ms = millis();
  return true;
}

void destroyEntities() {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  rcl_publisher_fini(&odom_pub, &node);
  rcl_publisher_fini(&imu_pub, &node);
  rcl_subscription_fini(&cmd_sub, &node);
  rcl_timer_fini(&control_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  motor_fl.begin(PWM_FREQUENCY, PWM_BITS);
  motor_fr.begin(PWM_FREQUENCY, PWM_BITS);
  motor_rl.begin(PWM_FREQUENCY, PWM_BITS);
  motor_rr.begin(PWM_FREQUENCY, PWM_BITS);

  imu_ok = imu.begin();
  if (imu_ok) imu.calibrate(IMU_CALIBRATION_SAMPLES);  // keep the robot still at power-up

  set_microros_transports();
  initMessages();
}

// Reconnect automatically when the agent on the PC restarts.
void loop() {
  static uint32_t last_ping_ms = 0;
  const uint32_t now = millis();

  switch (state) {
    case WAITING_AGENT:
      if (now - last_ping_ms > 500) {
        last_ping_ms = now;
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) state = AGENT_AVAILABLE;
      }
      break;
    case AGENT_AVAILABLE:
      state = createEntities() ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) destroyEntities();
      break;
    case AGENT_CONNECTED:
      if (now - last_ping_ms > 200) {
        last_ping_ms = now;
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
          state = AGENT_DISCONNECTED;
          break;
        }
      }
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      break;
    case AGENT_DISCONNECTED:
      stopAll();
      destroyEntities();
      state = WAITING_AGENT;
      break;
  }
}
