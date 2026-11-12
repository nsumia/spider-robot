#include "gait.h"
#include <math.h>

namespace spider {

static const float kPi = 3.14159265358979f;
static const float kDeg2Rad = 0.017453292519943295f;
static const float kStandQ = 0.0f;     // femur angle of the neutral stance (ground contact at ride height)
static const float kSitQ = 55.0f;      // femur angle when sitting (body lowered)

GaitParams defaultParams(GaitType t) {
  GaitParams p;
  if (t == GAIT_CREEP) { p.yaw_amp_deg = 13.0f; p.lift_deg = 28.0f; p.shift_mm = 42.0f; p.duty = 0.75f; p.period_s = 2.5f; }
  else { p.yaw_amp_deg = 16.0f; p.lift_deg = 28.0f; p.shift_mm = 0.0f; p.duty = 0.5f; p.period_s = 1.0f; }
  return p;
}

static float fracf(float x) { return x - floorf(x); }

// swing start offsets (fraction of the cycle). Creep order RR, FR, FL, RL (each swing neighbours the previous one, so the shift windows blend instead of cancelling) (leg index FL0 FR1 RR2 RL3).
static const float kCreepOff[4] = {0.5f, 0.25f, 0.0f, 0.75f};
static const float kTrotOff[4] = {0.0f, 0.5f, 0.0f, 0.5f};   // diagonal pairs FL+RR, FR+RL
static const float kFwdSign[4] = {+1.0f, -1.0f, -1.0f, +1.0f};   // left legs yaw + when the foot moves backwards

static float bump(float ph, float start, float swing_len, float rw) {
  float d = fracf(ph - (start - rw));
  if (d < rw) return 0.5f - 0.5f * cosf(kPi * d / rw);
  if (d < rw + swing_len) return 1.0f;
  if (d < 2.0f * rw + swing_len) return 0.5f + 0.5f * cosf(kPi * (d - rw - swing_len) / rw);
  return 0.0f;
}

void gaitSample(GaitType t, const GaitParams& p, float phase, float fwd, float turn, JointAngles out[4], bool swing[4]) {
  const float* off = (t == GAIT_CREEP) ? kCreepOff : kTrotOff;
  float swing_len = 1.0f - p.duty;
  // neutral foot radius / directions for the creep shift
  float r0, z0;
  footRZ(kStandQ, &r0, &z0);
  float sx = 0, sy = 0;   // desired foot shift (body frame) = minus the body shift
  if (p.shift_mm > 0.0f) {
    for (int k = 0; k < 4; k++) {
      float b = bump(phase, off[k], swing_len, 0.1f);
      float h = geom::kHeadingDeg[k] * kDeg2Rad;
      // body moves away from the swinging foot (towards the opposite corner) => the feet shift the other way (+dir)
      sx += b * cosf(h) * p.shift_mm;
      sy += b * sinf(h) * p.shift_mm;
    }
  }
  for (int l = 0; l < 4; l++) {
    float u = fracf(phase - off[l]);
    float pp, q = kStandQ;
    bool sw = u < swing_len;
    if (sw) {
      float s = u / swing_len;
      pp = 1.0f - 2.0f * (0.5f - 0.5f * cosf(kPi * s));
      q = kStandQ + p.lift_deg * sinf(kPi * s);
    } else {
      pp = -1.0f + 2.0f * (u - swing_len) / p.duty;
    }
    float a = clampf(fwd * kFwdSign[l] - turn, -1.0f, 1.0f);
    float psi = a * p.yaw_amp_deg * pp;
    if (p.shift_mm > 0.0f) {
      float h = (geom::kHeadingDeg[l]) * kDeg2Rad;
      float tx = -sinf(h), ty = cosf(h);            // yaw tangent at psi = 0
      psi += (sx * tx + sy * ty) / r0 / kDeg2Rad;  // (sx,sy) = desired foot shift in the body frame
    }
    out[l].psi = psi;
    out[l].q = q;
    out[l] = clampJoints(out[l]);
    swing[l] = sw;
  }
}

Robot::Robot() : mode_(MODE_STAND), gait_(GAIT_CREEP), params_(defaultParams(GAIT_CREEP)),
                 phase_(0), cmd_fwd_(0), cmd_turn_(0), fwd_(0), turn_(0) {
  for (int i = 0; i < 4; i++) { cur_[i].psi = 0; cur_[i].q = kStandQ; }
}

void Robot::walk(float fwd, float turn) {
  if (mode_ == MODE_FALLEN) return;
  cmd_fwd_ = clampf(fwd, -1.0f, 1.0f);
  cmd_turn_ = clampf(turn, -1.0f, 1.0f);
  mode_ = MODE_WALK;
}

void Robot::update(float dt, const ImuSample* imu) {
  if (imu) {
    tilt_.update(dt, *imu);
    if (fall_.update(tilt_.tilt()) && mode_ != MODE_FALLEN) {
      mode_ = MODE_FALLEN;               // stop gait, sit
      cmd_fwd_ = cmd_turn_ = 0;
    }
  }
  JointAngles tgt[4];
  float slew;
  if (mode_ == MODE_WALK) {
    // ramp command towards the requested value (2 /s)
    float k = 2.0f * dt;
    fwd_ += clampf(cmd_fwd_ - fwd_, -k, k);
    turn_ += clampf(cmd_turn_ - turn_, -k, k);
    phase_ = phase_ + dt / params_.period_s;
    phase_ -= floorf(phase_);
    bool sw[4];
    gaitSample(gait_, params_, phase_, fwd_, turn_, tgt, sw);
    slew = 300.0f;
  } else {
    fwd_ = turn_ = 0;
    phase_ = 0;
    float q = (mode_ == MODE_STAND) ? kStandQ : kSitQ;
    for (int i = 0; i < 4; i++) { tgt[i].psi = 0; tgt[i].q = q; }
    slew = 100.0f;
  }
  float m = slew * dt;
  for (int i = 0; i < 4; i++) {
    cur_[i].psi += clampf(tgt[i].psi - cur_[i].psi, -m, m);
    cur_[i].q += clampf(tgt[i].q - cur_[i].q, -m, m);
    cur_[i] = clampJoints(cur_[i]);
  }
}

void Robot::servoAngles(const ServoCal cal[kServos], float out[kServos]) const {
  for (int l = 0; l < 4; l++) {
    out[2 * l] = jointToServo(2 * l, cur_[l].psi, cal[2 * l]);
    out[2 * l + 1] = jointToServo(2 * l + 1, cur_[l].q, cal[2 * l + 1]);
  }
}

}  // namespace spider
