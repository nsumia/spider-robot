// Spider robot firmware: Arduino Nano Every + PCA9685 + MPU-6050. Geometry/gaits live in src/ (pure C++, host-tested).
// Serial 115200. Commands (one per line): see HELP below or ASSEMBLY.md.
#include <Wire.h>
#include "src/gait.h"
#include "pca9685.h"
#include "mpu6050.h"
#include "storage.h"

using namespace spider;

static PCA9685 pwm(0x40);
static Mpu6050 imu(0x68);
static Robot robot;
static ServoCal cal[kServos];
static bool imuOk = false, pwmOk = false, calMode = false;
static int calSel = 0;
static float calAngle[kServos];            // raw servo angles used in calibration mode
static uint32_t lastMs = 0, lastPrint = 0;
static char line[48];
static uint8_t n = 0;

static void help() {
  Serial.println(F("cmds: f b l r (walk fwd/back/turn L/R)  s (stand)  x (sit)  gait creep|trot  clear (reset fall)  stat"));
  Serial.println(F("cal: cal (enter)  sel <0-7>  a <deg>  + -  trim <deg>  dir <1|-1>  min <deg>  max <deg>  all90  save  load  reset  exit"));
  Serial.println(F("channel = leg*2 + (0 coxa, 1 femur); legs FL FR RR RL"));
}

static void writeServo(int i, float deg) { pwm.setTicks(i, (uint16_t)(servoToTicks(deg) + 0.5f)); }

static void printCal(int i) {
  Serial.print(F("servo ")); Serial.print(i);
  Serial.print(F(" dir ")); Serial.print(cal[i].dir);
  Serial.print(F(" trim ")); Serial.print(cal[i].trim_deg);
  Serial.print(F(" min ")); Serial.print(cal[i].min_deg);
  Serial.print(F(" max ")); Serial.print(cal[i].max_deg);
  Serial.print(F(" angle ")); Serial.println(calAngle[i]);
}

static void handle(char* s) {
  char* arg = strchr(s, ' ');
  if (arg) { *arg++ = 0; }
  float v = arg ? atof(arg) : 0.0f;
  if (!strcmp(s, "cal")) {
    calMode = true; for (int i = 0; i < kServos; i++) calAngle[i] = 90.0f;
    robot.sit(); Serial.println(F("calibration mode: servos hold raw angles (90). Keep the legs off the ground!")); return;
  }
  if (calMode) {
    if (!strcmp(s, "sel") && arg) { calSel = constrain((int)v, 0, kServos - 1); printCal(calSel); }
    else if (!strcmp(s, "a") && arg) { calAngle[calSel] = constrain(v, cal[calSel].min_deg, cal[calSel].max_deg); printCal(calSel); }
    else if (!strcmp(s, "+")) { calAngle[calSel] = constrain(calAngle[calSel] + 1, cal[calSel].min_deg, cal[calSel].max_deg); printCal(calSel); }
    else if (!strcmp(s, "-")) { calAngle[calSel] = constrain(calAngle[calSel] - 1, cal[calSel].min_deg, cal[calSel].max_deg); printCal(calSel); }
    else if (!strcmp(s, "trim") && arg) { cal[calSel].trim_deg = constrain(v, -30.0f, 30.0f); printCal(calSel); }
    else if (!strcmp(s, "dir") && arg) { cal[calSel].dir = (v < 0) ? -1 : 1; printCal(calSel); }
    else if (!strcmp(s, "min") && arg) { cal[calSel].min_deg = constrain(v, 0.0f, cal[calSel].max_deg - 1); printCal(calSel); }
    else if (!strcmp(s, "max") && arg) { cal[calSel].max_deg = constrain(v, cal[calSel].min_deg + 1, 180.0f); printCal(calSel); }
    else if (!strcmp(s, "all90")) { for (int i = 0; i < kServos; i++) calAngle[i] = 90.0f; Serial.println(F("all 90")); }
    else if (!strcmp(s, "save")) { saveCal(cal); Serial.println(F("saved")); }
    else if (!strcmp(s, "load")) { Serial.println(loadCal(cal) ? F("loaded") : F("no valid data")); }
    else if (!strcmp(s, "reset")) { defaultCal(cal); Serial.println(F("defaults (not saved)")); }
    else if (!strcmp(s, "exit")) { calMode = false; robot.stand(); Serial.println(F("calibration off")); }
    else help();
    return;
  }
  if (!strcmp(s, "f")) robot.walk(1, 0);
  else if (!strcmp(s, "b")) robot.walk(-1, 0);
  else if (!strcmp(s, "l")) robot.walk(0, 1);
  else if (!strcmp(s, "r")) robot.walk(0, -1);
  else if (!strcmp(s, "s")) robot.stand();
  else if (!strcmp(s, "x")) robot.sit();
  else if (!strcmp(s, "gait") && arg) { robot.setGait(!strcmp(arg, "trot") ? GAIT_TROT : GAIT_CREEP); Serial.println(robot.gait() == GAIT_TROT ? F("trot") : F("creep")); }
  else if (!strcmp(s, "clear")) { robot.clearFall(); Serial.println(F("fall cleared (sitting)")); }
  else if (!strcmp(s, "stat")) {
    Serial.print(F("mode ")); Serial.print((int)robot.mode());
    Serial.print(F(" tilt ")); Serial.print(robot.tilt().tilt());
    Serial.print(F(" imu ")); Serial.print(imuOk); Serial.print(F(" pca ")); Serial.println(pwmOk);
  } else help();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  defaultCal(cal);
  bool loaded = loadCal(cal);
  pwmOk = pwm.begin();
  imuOk = imu.begin();
  if (imuOk) imu.calibrateGyro();
  Serial.print(F("spider fw: pca9685 ")); Serial.print(pwmOk ? F("ok") : F("MISSING"));
  Serial.print(F(", mpu6050 ")); Serial.print(imuOk ? F("ok") : F("MISSING"));
  Serial.println(loaded ? F(", calibration loaded") : F(", default calibration (run 'cal')"));
  help();
  lastMs = millis();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (n) { line[n] = 0; handle(line); n = 0; } }
    else if (n < sizeof(line) - 1) line[n++] = c;
  }
  uint32_t now = millis();
  if (now - lastMs < 20) return;     // 50 Hz
  float dt = (now - lastMs) * 0.001f;
  lastMs = now;

  float out[kServos];
  if (calMode) {
    for (int i = 0; i < kServos; i++) out[i] = calAngle[i];
  } else {
    ImuSample s;
    bool have = imuOk && imu.read(&s);
    robot.update(dt, have ? &s : nullptr);
    robot.servoAngles(cal, out);
  }
  if (pwmOk) for (int i = 0; i < kServos; i++) writeServo(i, out[i] + (calMode ? cal[i].trim_deg : 0.0f));
  if (robot.fallen() && now - lastPrint > 2000) { lastPrint = now; Serial.println(F("FALL detected: stopped and sitting. 'clear' then 's' to resume.")); }
}
