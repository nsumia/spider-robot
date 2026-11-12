// Minimal MPU-6050 (GY-521) reader. +-2 g, +-500 deg/s. Mount flat: +x forward, +y left, +z up (see WIRING.md).
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "src/imu_filter.h"

class Mpu6050 {
 public:
  explicit Mpu6050(uint8_t addr = 0x68) : addr_(addr), gbx_(0), gby_(0), gbz_(0) {}
  bool begin() {
    if (!write8(0x6B, 0x00)) return false;   // wake
    write8(0x1B, 0x08);                      // gyro +-500 deg/s
    write8(0x1C, 0x00);                      // accel +-2 g
    write8(0x1A, 0x03);                      // DLPF ~44 Hz
    return true;
  }
  bool read(spider::ImuSample* s) {
    Wire.beginTransmission(addr_); Wire.write((uint8_t)0x3B);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)addr_, 14) != 14) return false;
    int16_t v[7];
    for (int i = 0; i < 7; i++) { uint8_t h = Wire.read(); uint8_t l = Wire.read(); v[i] = (int16_t)((h << 8) | l); }
    s->ax = v[0] / 16384.0f; s->ay = v[1] / 16384.0f; s->az = v[2] / 16384.0f;
    s->gx = v[4] / 65.5f - gbx_; s->gy = v[5] / 65.5f - gby_; s->gz = v[6] / 65.5f - gbz_;
    return true;
  }
  void calibrateGyro(int n = 100) {          // robot must stand still
    float sx = 0, sy = 0, sz = 0; int k = 0; spider::ImuSample s;
    gbx_ = gby_ = gbz_ = 0;
    for (int i = 0; i < n; i++) { if (read(&s)) { sx += s.gx; sy += s.gy; sz += s.gz; k++; } delay(10); }
    if (k) { gbx_ = sx / k; gby_ = sy / k; gbz_ = sz / k; }
  }
 private:
  bool write8(uint8_t reg, uint8_t v) { Wire.beginTransmission(addr_); Wire.write(reg); Wire.write(v); return Wire.endTransmission() == 0; }
  uint8_t addr_; float gbx_, gby_, gbz_;
};
