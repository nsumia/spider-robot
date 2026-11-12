#include "kinematics.h"
#include <math.h>

namespace spider {

static const float kDeg2Rad = 0.017453292519943295f;
static const float kRad2Deg = 57.29577951308232f;

float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

float wrap180(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d <= -180.0f) d += 360.0f;
  return d;
}

void footRZ(float q, float* r, float* z) {
  q = clampf(q, (float)geom::kTableQ0, (float)(geom::kTableQ0 + geom::kTableN - 1));
  float u = q - (float)geom::kTableQ0;
  int i = (int)u;
  if (i > geom::kTableN - 2) i = geom::kTableN - 2;
  float f = u - (float)i;
  *r = geom::kFootR[i] + f * (geom::kFootR[i + 1] - geom::kFootR[i]);
  *z = geom::kFootZ[i] + f * (geom::kFootZ[i + 1] - geom::kFootZ[i]);
}

Vec3 legFK(int leg, const JointAngles& a) {
  float r, z;
  footRZ(a.q, &r, &z);
  float h = (geom::kHeadingDeg[leg] + a.psi) * kDeg2Rad;
  Vec3 p = {geom::kHipX[leg] + r * cosf(h), geom::kHipY[leg] + r * sinf(h), z};
  return p;
}

JointAngles clampJoints(const JointAngles& a) {
  JointAngles c = {clampf(a.psi, geom::kCoxaDegMin, geom::kCoxaDegMax),
                   clampf(a.q, geom::kFemurDegMin, geom::kFemurDegMax)};
  return c;
}

static float dist2(float r, float z, float q) {
  float fr, fz;
  footRZ(q, &fr, &fz);
  return (r - fr) * (r - fr) + (z - fz) * (z - fz);
}

IkResult legIK(int leg, const Vec3& t) {
  float dx = t.x - geom::kHipX[leg], dy = t.y - geom::kHipY[leg];
  float r = sqrtf(dx * dx + dy * dy);
  float psi = wrap180(atan2f(dy, dx) * kRad2Deg - geom::kHeadingDeg[leg]);
  // nearest point on the (r,z) foot curve: coarse scan on table nodes, then ternary refinement
  int best = 0;
  float bd = 1e30f;
  for (int i = 0; i < geom::kTableN; i++) {
    float d = dist2(r, t.z, (float)(geom::kTableQ0 + i));
    if (d < bd) { bd = d; best = i; }
  }
  float lo = (float)(geom::kTableQ0 + best) - 1.0f, hi = (float)(geom::kTableQ0 + best) + 1.0f;
  lo = clampf(lo, (float)geom::kFemurDegMin, (float)geom::kFemurDegMax);
  hi = clampf(hi, (float)geom::kFemurDegMin, (float)geom::kFemurDegMax);
  for (int k = 0; k < 24; k++) {
    float m1 = lo + (hi - lo) / 3.0f, m2 = hi - (hi - lo) / 3.0f;
    if (dist2(r, t.z, m1) < dist2(r, t.z, m2)) hi = m2; else lo = m1;
  }
  IkResult res;
  res.a.q = 0.5f * (lo + hi);
  res.a.psi = psi;
  JointAngles c = clampJoints(res.a);
  bool clamped = (c.psi != res.a.psi) || (c.q != res.a.q);
  res.a = c;
  Vec3 f = legFK(leg, res.a);
  float ex = f.x - t.x, ey = f.y - t.y, ez = f.z - t.z;
  res.err_mm = sqrtf(ex * ex + ey * ey + ez * ez);
  res.reachable = !clamped && res.err_mm <= 0.5f;
  return res;
}

}  // namespace spider
