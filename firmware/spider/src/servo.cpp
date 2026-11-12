#include "servo.h"

namespace spider {

void defaultCal(ServoCal cal[kServos]) {
  for (int i = 0; i < kServos; i++) {
    bool coxa = (i % 2) == 0;
    cal[i].dir = (int8_t)(coxa ? geom::kCoxaSign : geom::kFemurSign);
    cal[i].trim_deg = 0.0f;
    cal[i].min_deg = coxa ? geom::kCoxaServoAbsMin : geom::kFemurServoAbsMin;
    cal[i].max_deg = coxa ? geom::kCoxaServoAbsMax : geom::kFemurServoAbsMax;
  }
}

float jointToServo(int servo, float joint, const ServoCal& c) {
  bool coxa = (servo % 2) == 0;
  float zero = coxa ? geom::kCoxaServoZero : geom::kFemurServoZero;
  float lo = coxa ? geom::kCoxaDegMin : geom::kFemurDegMin;
  float hi = coxa ? geom::kCoxaDegMax : geom::kFemurDegMax;
  joint = clampf(joint, lo, hi);
  float s = zero + (float)c.dir * joint + c.trim_deg;
  float mn = c.min_deg < 0.0f ? 0.0f : c.min_deg, mx = c.max_deg > 180.0f ? 180.0f : c.max_deg;
  return clampf(s, mn, mx);
}

float servoToTicks(float deg, float us_min, float us_max, float hz) {
  float us = us_min + (us_max - us_min) * clampf(deg, 0.0f, 180.0f) / 180.0f;
  return us * 4096.0f * hz / 1000000.0f;
}

}  // namespace spider
