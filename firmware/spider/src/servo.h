// Servo mapping (pure C++): joint angle (deg, CAD convention) -> servo angle -> PCA9685 ticks.
// Servo sign/trim are CALIBRATION values (CAD sign convention is unverified on hardware).
#pragma once
#include <stdint.h>
#include "kinematics.h"

namespace spider {

static const int kServos = 8;  // channel = leg * 2 + (0 coxa | 1 femur), leg order FL, FR, RR, RL

struct ServoCal {
  int8_t dir;       // +1 / -1: servo_angle = zero + dir * joint + trim
  float trim_deg;   // added after the direction (centre trim)
  float min_deg;    // allowed servo angle window (never exceeded)
  float max_deg;
};

void defaultCal(ServoCal cal[kServos]);          // from geometry_gen.h (absolute limits, assumed signs)
float jointToServo(int servo, float joint_deg, const ServoCal& c);   // clamps to [min,max] and to 0..180
float servoToTicks(float servo_deg, float us_min = 500.0f, float us_max = 2500.0f, float hz = 50.0f);

}  // namespace spider
