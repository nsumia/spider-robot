// Complementary filter + fall detection (pure C++). Frame: x forward, y left, z up (GY-521 flat, +x forward).
#pragma once

namespace spider {

struct ImuSample { float ax, ay, az; float gx, gy, gz; };  // accel in g (any unit), gyro in deg/s

class TiltFilter {
 public:
  explicit TiltFilter(float alpha = 0.98f) : alpha_(alpha), roll_(0), pitch_(0), init_(false) {}
  void update(float dt, const ImuSample& s);
  float roll() const { return roll_; }     // deg
  float pitch() const { return pitch_; }   // deg
  float tilt() const;                      // deg, angle between body z and vertical
 private:
  float alpha_, roll_, pitch_;
  bool init_;
};

class FallDetector {
 public:
  explicit FallDetector(float limit_deg = 30.0f, int confirm = 3) : limit_(limit_deg), confirm_(confirm), n_(0), fallen_(false) {}
  // returns true while fallen (latched until reset())
  bool update(float tilt_deg);
  bool fallen() const { return fallen_; }
  void reset() { n_ = 0; fallen_ = false; }
  float limit() const { return limit_; }
 private:
  float limit_; int confirm_; int n_; bool fallen_;
};

}  // namespace spider
