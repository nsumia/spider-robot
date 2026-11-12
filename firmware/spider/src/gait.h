// Gait generators (pure C++, stateless sampling) + Robot controller.
// Gaits work in joint space: stance = femur at stand angle, coxa sweeps the foot back; swing = femur lifts the foot
// (four-bar couples r and z), coxa swings forward. Creep adds a body-weight shift (coxa-only, tangential) so the CoM
// stays inside the support triangle. CoM is assumed at the body centre (battery centred; leg mass ignored).
#pragma once
#include "kinematics.h"
#include "servo.h"
#include "imu_filter.h"

namespace spider {

enum GaitType { GAIT_CREEP = 0, GAIT_TROT = 1 };

struct GaitParams {
  float yaw_amp_deg;    // coxa half-stroke at full command
  float lift_deg;       // femur lift added in swing
  float shift_mm;       // creep: body-weight shift towards the support triangle
  float duty;           // stance fraction
  float period_s;
};
GaitParams defaultParams(GaitType t);

// Sample the gait at phase in [0,1). fwd, turn in [-1,1] (turn > 0 = turn left / CCW). Output joint angles per leg and
// whether the leg is in swing.
void gaitSample(GaitType t, const GaitParams& p, float phase, float fwd, float turn, JointAngles out[4], bool swing[4]);

enum Mode { MODE_STAND = 0, MODE_SIT, MODE_WALK, MODE_FALLEN };

class Robot {
 public:
  Robot();
  void stand() { cmd_fwd_ = cmd_turn_ = 0; if (mode_ != MODE_FALLEN) mode_ = MODE_STAND; }
  void sit() { cmd_fwd_ = cmd_turn_ = 0; mode_ = MODE_SIT; }
  void walk(float fwd, float turn);             // fwd, turn in [-1,1]
  void setGait(GaitType t) { gait_ = t; params_ = defaultParams(t); }
  GaitType gait() const { return gait_; }
  Mode mode() const { return mode_; }
  // Advance dt seconds; imu may be null (no fall detection).
  void update(float dt, const ImuSample* imu);
  const JointAngles* joints() const { return cur_; }
  void servoAngles(const ServoCal cal[kServos], float out[kServos]) const;   // 8 clamped servo angles (deg)
  const TiltFilter& tilt() const { return tilt_; }
  bool fallen() const { return mode_ == MODE_FALLEN; }
  void clearFall() { fall_.reset(); if (mode_ == MODE_FALLEN) mode_ = MODE_SIT; }
 private:
  JointAngles cur_[4];
  Mode mode_;
  GaitType gait_;
  GaitParams params_;
  float phase_, cmd_fwd_, cmd_turn_, fwd_, turn_;
  TiltFilter tilt_;
  FallDetector fall_;
};

}  // namespace spider
