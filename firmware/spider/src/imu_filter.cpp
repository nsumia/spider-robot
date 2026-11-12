#include "imu_filter.h"
#include <math.h>

namespace spider {

static const float kRad2Deg = 57.29577951308232f;

void TiltFilter::update(float dt, const ImuSample& s) {
  float ar = atan2f(s.ay, s.az) * kRad2Deg;
  float ap = atan2f(-s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * kRad2Deg;
  if (!init_) { roll_ = ar; pitch_ = ap; init_ = true; return; }
  roll_ = alpha_ * (roll_ + s.gx * dt) + (1.0f - alpha_) * ar;
  pitch_ = alpha_ * (pitch_ + s.gy * dt) + (1.0f - alpha_) * ap;
}

float TiltFilter::tilt() const {
  const float d = 0.017453292519943295f;
  float c = cosf(roll_ * d) * cosf(pitch_ * d);
  if (c > 1.0f) c = 1.0f;
  if (c < -1.0f) c = -1.0f;
  return acosf(c) * kRad2Deg;
}

bool FallDetector::update(float tilt_deg) {
  if (tilt_deg > limit_) { if (n_ < 1000) n_++; } else n_ = 0;
  if (n_ >= confirm_) fallen_ = true;
  return fallen_;
}

}  // namespace spider
