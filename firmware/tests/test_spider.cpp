// Host unit tests for the pure C++ firmware library. Exit code != 0 on any failure.
#include <stdio.h>
#include <math.h>
#include "kinematics.h"
#include "servo.h"
#include "gait.h"
#include "imu_filter.h"
using namespace spider;

static int fails = 0, checks = 0;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL: %s:%d %s | ", __FILE__, __LINE__, #c); printf(__VA_ARGS__); printf("\n"); } } while (0)

static float dist3(Vec3 a, Vec3 b) { return sqrtf((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z)); }

static void test_ik_roundtrip() {
  float worst = 0; int n = 0;
  for (int leg = 0; leg < 4; leg++)
    for (float psi = -45; psi <= 45.01f; psi += 2.5f)
      for (float q = -30; q <= 60.01f; q += 0.7f) {
        JointAngles a = {psi, q};
        Vec3 p = legFK(leg, a);
        IkResult r = legIK(leg, p);
        float e = dist3(legFK(leg, r.a), p);
        if (e > worst) worst = e;
        n++;
        CHECK(r.reachable, "leg %d psi %.1f q %.1f not reachable", leg, psi, q);
      }
  printf("IK round trip: %d points, worst FK(IK(p))-p = %.4f mm\n", n, worst);
  CHECK(worst < 0.5f, "worst %.4f", worst);
  // neutral foot matches the CAD neutral contact point
  Vec3 n0 = legFK(0, JointAngles{0, 0});
  CHECK(fabsf(n0.z - (-75.021f)) < 0.01f, "neutral z %.3f", n0.z);
}

static void test_unreachable() {
  for (int leg = 0; leg < 4; leg++) {
    Vec3 t = {geom::kHipX[leg] * 8, geom::kHipY[leg] * 8, -75};
    IkResult r = legIK(leg, t);
    CHECK(!r.reachable, "far target flagged reachable");
    CHECK(r.a.q >= geom::kFemurDegMin && r.a.q <= geom::kFemurDegMax, "q %.2f out of range", r.a.q);
    CHECK(r.a.psi >= geom::kCoxaDegMin && r.a.psi <= geom::kCoxaDegMax, "psi %.2f out of range", r.a.psi);
    // target above the body, and exactly at the hip
    IkResult r2 = legIK(leg, Vec3{geom::kHipX[leg], geom::kHipY[leg], 200});
    CHECK(!r2.reachable && r2.a.q >= geom::kFemurDegMin && r2.a.q <= geom::kFemurDegMax, "hip target");
    // yaw beyond +-45 deg clamps
    float h = (geom::kHeadingDeg[leg] + 80.0f) * 0.017453292f;
    IkResult r3 = legIK(leg, Vec3{geom::kHipX[leg] + 76.7f * cosf(h), geom::kHipY[leg] + 76.7f * sinf(h), -75.0f});
    CHECK(!r3.reachable && fabsf(r3.a.psi - 45.0f) < 1e-3f, "yaw clamp psi %.2f", r3.a.psi);
  }
}

// signed distance from the origin to the polygon (positive = inside), vertices in cyclic order
static float polyMargin(const Vec3* v, int n) {
  float area = 0;
  for (int i = 0; i < n; i++) { int j = (i + 1) % n; area += v[i].x * v[j].y - v[j].x * v[i].y; }
  float sgn = area > 0 ? 1.0f : -1.0f;
  float m = 1e30f;
  for (int i = 0; i < n; i++) {
    int j = (i + 1) % n;
    float ex = v[j].x - v[i].x, ey = v[j].y - v[i].y;
    float len = sqrtf(ex * ex + ey * ey);
    float d = sgn * (ex * (0 - v[i].y) - ey * (0 - v[i].x)) / len;   // cross(e, p - v_i)/|e| with p = origin
    if (d < m) m = d;
  }
  return m;
}

static void test_creep() {
  GaitParams p = defaultParams(GAIT_CREEP);
  CHECK(fabsf(p.duty - 0.75f) < 1e-6f && p.period_s >= 2.0f && p.period_s <= 3.0f, "creep params");
  float ground = legFK(0, JointAngles{0, 0}).z;
  const float cmds[][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {0.5f, 0.5f}, {1, -1}};
  float minMargin = 1e30f; int samples = 0, minDown = 4;
  for (auto& c : cmds)
    for (int i = 0; i < 2000; i++) {
      float ph = i / 2000.0f;
      JointAngles j[4]; bool sw[4];
      gaitSample(GAIT_CREEP, p, ph, c[0], c[1], j, sw);
      Vec3 foot[4]; Vec3 st[4]; int down = 0;
      for (int l = 0; l < 4; l++) {
        foot[l] = legFK(l, j[l]);
        if (foot[l].z <= ground + 0.5f) st[down++] = foot[l];   // physically on the ground (FK, not the swing flag)
      }
      if (down < minDown) minDown = down;
      CHECK(down >= 3, "cmd (%.1f,%.1f) phase %.4f only %d feet down", c[0], c[1], ph, down);
      if (down >= 3) {
        float m = polyMargin(st, down);
        if (m < minMargin) minMargin = m;
        CHECK(m > 0.0f, "cmd (%.1f,%.1f) phase %.4f CoM outside support polygon (margin %.2f)", c[0], c[1], ph, m);
      }
      samples++;
    }
  printf("creep: %d samples, min feet down %d, min static margin %.2f mm\n", samples, minDown, minMargin);
  CHECK(minMargin > 10.0f, "static margin %.2f mm too small", minMargin);
}

static void test_trot() {
  GaitParams p = defaultParams(GAIT_TROT);
  CHECK(p.period_s >= 0.8f && p.period_s <= 1.2f, "trot period");
  float worst = 0; int nswing = 0;
  for (int i = 0; i < 2000; i++) {
    JointAngles j[4]; bool sw[4];
    float ph = i / 2000.0f;
    gaitSample(GAIT_TROT, p, ph, 1, 0, j, sw);
    // diagonal pairs FL(0)+RR(2) and FR(1)+RL(3): same swing state, same femur, mirrored coxa per their sign
    CHECK(sw[0] == sw[2] && sw[1] == sw[3], "diagonal swing mismatch at %.3f", ph);
    CHECK(sw[0] != sw[1], "pairs not in anti-phase at %.3f", ph);
    float dq = fmaxf(fabsf(j[0].q - j[2].q), fabsf(j[1].q - j[3].q));
    float dpsi = fmaxf(fabsf(j[0].psi + j[2].psi), fabsf(j[1].psi + j[3].psi));  // FL +, RR - sign for forward
    worst = fmaxf(worst, fmaxf(dq, dpsi));
    nswing += sw[0];
  }
  printf("trot: worst diagonal mismatch %.6f deg, swing fraction %.3f\n", worst, nswing / 2000.0f);
  CHECK(worst < 1e-3f, "diagonal mismatch %.6f", worst);
  CHECK(fabsf(nswing / 2000.0f - 0.5f) < 0.01f, "duty");
}

static void test_servo_limits() {
  ServoCal cal[kServos]; defaultCal(cal);
  float lo[kServos], hi[kServos];
  for (int i = 0; i < kServos; i++) { lo[i] = cal[i].min_deg; hi[i] = cal[i].max_deg; }
  int n = 0; float mn = 1e9, mx = -1e9;
  for (int variant = 0; variant < 6; variant++) {   // direction +-1 x trim {0, +25, -25} (trim pushes towards the limits)
    for (int i = 0; i < kServos; i++) { cal[i].dir = (variant % 2) ? -1 : 1; cal[i].trim_deg = (variant / 2 == 0) ? 0.0f : (variant / 2 == 1 ? 25.0f : -25.0f); }
    for (int g = 0; g < 2; g++)
      for (int c = 0; c < 6; c++) {
        const float cmds[6][2] = {{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,1}};
        Robot r; r.setGait(g ? GAIT_TROT : GAIT_CREEP);
        r.walk(cmds[c][0] * 3, cmds[c][1] * 3);   // over-range commands are clamped
        for (int s = 0; s < 4000; s++) {
          r.update(0.01f, nullptr);
          float sv[kServos]; r.servoAngles(cal, sv);
          for (int i = 0; i < kServos; i++) {
            CHECK(sv[i] >= lo[i] - 1e-3f && sv[i] <= hi[i] + 1e-3f, "servo %d = %.2f outside [%.0f,%.0f]", i, sv[i], lo[i], hi[i]);
            mn = fminf(mn, sv[i]); mx = fmaxf(mx, sv[i]); n++;
          }
          const JointAngles* j = r.joints();
          for (int l = 0; l < 4; l++)
            CHECK(j[l].psi >= -45.001f && j[l].psi <= 45.001f && j[l].q >= -30.001f && j[l].q <= 60.001f, "joint range");
        }
      }
  }
  // extreme joint requests are clamped too
  ServoCal c0[kServos]; defaultCal(c0);
  for (float x = -400; x <= 400; x += 7) for (int i = 0; i < kServos; i++) {
    float s = jointToServo(i, x, c0[i]);
    CHECK(s >= lo[i] && s <= hi[i], "extreme %f", x);
  }
  CHECK(fabsf(servoToTicks(90) - 307.2f) < 0.1f, "ticks %f", servoToTicks(90));
  printf("servo limits: %d samples, servo angle range seen %.1f..%.1f deg\n", n, mn, mx);
}

static void test_fall() {
  const float d = 0.017453292f;
  for (int k = 0; k < 2; k++) {
    float tiltDeg = k ? 31.0f : 29.0f;
    TiltFilter f; FallDetector fd; bool fell = false;
    ImuSample s = {-sinf(tiltDeg * d), 0, cosf(tiltDeg * d), 0, 0, 0};   // pitched by tiltDeg
    for (int i = 0; i < 200; i++) { f.update(0.01f, s); fell = fd.update(f.tilt()); }
    CHECK(fell == (k == 1), "tilt %.0f deg -> fallen=%d", tiltDeg, (int)fell);
    CHECK(fabsf(f.tilt() - tiltDeg) < 0.5f, "filter tilt %.2f vs %.0f", f.tilt(), tiltDeg);
  }
  // gyro integration: starts level, rotates at 100 deg/s for 0.5 s with a consistent accel -> ~50 deg tracked
  TiltFilter f; f.update(0.01f, ImuSample{0, 0, 1, 0, 0, 0});
  for (int i = 0; i < 50; i++) {
    float ang = (i + 1) * 0.01f * 100.0f * d;
    f.update(0.01f, ImuSample{0, sinf(ang), cosf(ang), 100, 0, 0});
  }
  CHECK(fabsf(f.roll() - 50.0f) < 3.0f, "roll tracking %.2f", f.roll());
  // a single noisy sample does not trigger
  FallDetector fd; CHECK(!fd.update(45) && !fd.update(10) && !fd.update(45) && !fd.update(45), "debounce");
  // robot: tilt triggers stop + sit
  Robot r; r.walk(1, 0);
  ImuSample level = {0, 0, 1, 0, 0, 0};
  for (int i = 0; i < 100; i++) r.update(0.01f, &level);
  CHECK(r.mode() == MODE_WALK, "walking while level");
  ImuSample tipped = {-sinf(40 * d), 0, cosf(40 * d), 0, 0, 0};
  for (int i = 0; i < 400; i++) r.update(0.01f, &tipped);   // filter must converge, then detect
  CHECK(r.fallen() && r.mode() == MODE_FALLEN, "fall not detected");
  for (int i = 0; i < 100; i++) r.update(0.01f, &tipped);
  for (int l = 0; l < 4; l++) CHECK(fabsf(r.joints()[l].q - 55.0f) < 0.5f && fabsf(r.joints()[l].psi) < 0.5f, "not sitting");
  r.walk(1, 0);
  CHECK(r.mode() == MODE_FALLEN, "walk accepted while fallen");
  printf("fall detect: 29 deg no, 31 deg yes, robot stops and sits\n");
}

static void test_boot() {
  Robot r; r.startSitting();
  CHECK(r.mode() == MODE_SIT && fabsf(r.joints()[0].q - 55.0f) < 1e-3f, "boot pose not sit");
  r.stand();
  float prev = r.joints()[1].q, maxStep = 0;
  for (int i = 0; i < 100; i++) { r.update(0.01f, nullptr); maxStep = fmaxf(maxStep, fabsf(r.joints()[1].q - prev)); prev = r.joints()[1].q; }
  CHECK(fabsf(r.joints()[1].q) < 0.01f, "did not reach stand");
  CHECK(maxStep <= 1.0f + 1e-3f, "stand ramp too fast: %.2f deg per 10 ms", maxStep);
  printf("boot: starts sitting, stand ramp max %.2f deg/10 ms\n", maxStep);
}

int main() {
  test_boot();
  test_ik_roundtrip(); test_unreachable(); test_creep(); test_trot(); test_servo_limits(); test_fall();
  printf("%d checks, %d failures\n", checks, fails);
  if (fails) { printf("HOST TESTS FAILED\n"); return 1; }
  printf("HOST TESTS PASSED\n");
  return 0;
}
