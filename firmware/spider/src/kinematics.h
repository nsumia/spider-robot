// Pure C++ kinematics (no Arduino deps). Geometry from geometry_gen.h (generated from the CAD JSON).
// Leg-local frame: r radial from coxa axis, z up (0 = body underside). Femur angle q (deg, +knee up) fully determines
// the foot (r,z) through the four-bar table; the coxa yaw psi (deg, CCW from the hip heading) rotates it about z.
// The reachable workspace is therefore a 2-D surface per leg; IK projects any target onto it (nearest point).
#pragma once
#include "geometry_gen.h"

namespace spider {

struct Vec3 { float x, y, z; };

struct JointAngles { float psi; float q; };   // degrees

struct IkResult {
  JointAngles a;
  bool reachable;   // false if the target was off the workspace surface (>0.5 mm) or a limit was clamped
  float err_mm;     // distance between the target and FK(a)
};

float clampf(float v, float lo, float hi);
float wrap180(float deg);

// Linear interpolation of the CAD foot table. q is clamped to the table range.
void footRZ(float q_deg, float* r, float* z);

// FK: leg index (0..3, order of geom::) + joint angles -> foot contact point in the body frame.
Vec3 legFK(int leg, const JointAngles& a);

// IK: body-frame foot target -> joint angles. Clamps to coxa/femur limits and to the workspace surface.
IkResult legIK(int leg, const Vec3& target);

JointAngles clampJoints(const JointAngles& a);

}  // namespace spider
