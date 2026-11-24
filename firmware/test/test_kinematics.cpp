// Host-side tests for the kinematics and odometry used on the Teensy.
//   g++ -std=c++11 -I../amr_base_teensy test_kinematics.cpp -o /tmp/test_base && /tmp/test_base
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "kinematics.h"
#include "odometry.h"

static int failures = 0;

static void expectNear(const char* what, float got, float want, float tol = 1e-3f) {
  if (std::fabs(got - want) > tol) {
    std::printf("FAIL %s: got %.5f, want %.5f\n", what, got, want);
    failures++;
  }
}

int main() {
  MecanumKinematics k(0.196f, 0.300f, 0.360f, 1000.0f);  // high max_rpm: no scaling

  // Round trip: inverse then forward gives the same body velocity.
  const Velocity cases[] = {
      {0.2f, 0.0f, 0.0f},   // forward
      {0.0f, 0.2f, 0.0f},   // strafe left
      {0.0f, 0.0f, 0.5f},   // spin in place
      {0.15f, -0.1f, 0.3f}, // combined
  };
  for (const Velocity& v : cases) {
    const Velocity back = k.forward(k.inverse(v));
    expectNear("round trip vx", back.linear_x, v.linear_x);
    expectNear("round trip vy", back.linear_y, v.linear_y);
    expectNear("round trip wz", back.angular_z, v.angular_z);
  }

  // Forward motion: all wheels equal and positive.
  WheelRPM w = k.inverse({0.2f, 0.0f, 0.0f});
  expectNear("fwd fl=fr", w.front_left, w.front_right);
  expectNear("fwd fl=rl", w.front_left, w.rear_left);
  expectNear("fwd rpm", w.front_left, 0.2f * 60.0f / (3.14159265f * 0.196f), 1e-2f);

  // Strafe left: diagonal pairs turn in opposite directions (handbook 4.3.1).
  w = k.inverse({0.0f, 0.2f, 0.0f});
  expectNear("strafe fl=-fr", w.front_left, -w.front_right);
  expectNear("strafe fl=rr", w.front_left, w.rear_right);
  expectNear("strafe fr=rl", w.front_right, w.rear_left);

  // Saturation keeps the ratio between wheels.
  MecanumKinematics slow(0.196f, 0.300f, 0.360f, 55.0f);
  w = slow.inverse({1.0f, 0.5f, 0.0f});
  expectNear("saturated peak", std::fmax(std::fabs(w.front_right), std::fabs(w.front_left)), 55.0f, 1e-2f);
  expectNear("saturated ratio", w.front_left / w.front_right, (1.0f - 0.5f) / (1.0f + 0.5f));

  // Odometry: 1 m forward, 1 m strafe, full turn, and a closed circle.
  {
    Odometry o;
    for (int i = 0; i < 100; i++) o.update(0.2f, 0.0f, 0.0f, 0.05f);
    expectNear("odom forward x", o.x(), 1.0f);
    expectNear("odom forward y", o.y(), 0.0f);
  }
  {
    Odometry o;
    for (int i = 0; i < 100; i++) o.update(0.0f, 0.2f, 0.0f, 0.05f);
    expectNear("odom strafe y", o.y(), 1.0f);
  }
  {
    Odometry o;
    for (int i = 0; i < 80; i++) o.update(0.0f, 0.0f, 3.14159265f / 2, 0.05f);
    expectNear("odom full turn", o.theta(), 0.0f);
  }
  {
    Odometry o;  // 0.2 m/s while turning 0.5 rad/s: one lap is 4*pi s
    const int steps = 1257;
    for (int i = 0; i < steps; i++) o.update(0.2f, 0.0f, 0.5f, 4.0f * 3.14159265f / steps);
    expectNear("odom circle x", o.x(), 0.0f, 5e-3f);
    expectNear("odom circle y", o.y(), 0.0f, 5e-3f);
  }

  if (failures) {
    std::printf("%d check(s) failed\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("all kinematics and odometry checks passed\n");
  return EXIT_SUCCESS;
}
