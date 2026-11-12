// Minimal PCA9685 driver over Wire (no external library). 50 Hz servo PWM, 12-bit ticks.
#pragma once
#include <Arduino.h>
#include <Wire.h>

class PCA9685 {
 public:
  explicit PCA9685(uint8_t addr = 0x40) : addr_(addr) {}
  bool begin() {
    if (!write8(0x00, 0x00)) return false;      // MODE1: normal
    delay(5);
    write8(0x00, 0x10);                         // sleep to set prescale
    write8(0xFE, 121);                          // PRESCALE: round(25 MHz / (4096 * 50 Hz)) - 1 = 121
    write8(0x00, 0x20);                         // wake, auto-increment
    delay(5);
    write8(0x00, 0xA0);                         // restart + auto-increment
    return true;
  }
  void setTicks(uint8_t ch, uint16_t ticks) {   // on = 0, off = ticks
    if (ticks > 4095) ticks = 4095;
    Wire.beginTransmission(addr_);
    Wire.write((uint8_t)(0x06 + 4 * ch));
    Wire.write((uint8_t)0); Wire.write((uint8_t)0);
    Wire.write((uint8_t)(ticks & 0xFF)); Wire.write((uint8_t)(ticks >> 8));
    Wire.endTransmission();
  }
  void off(uint8_t ch) {                        // full-off bit: no pulse, servo relaxes
    Wire.beginTransmission(addr_);
    Wire.write((uint8_t)(0x06 + 4 * ch));
    Wire.write((uint8_t)0); Wire.write((uint8_t)0); Wire.write((uint8_t)0); Wire.write((uint8_t)0x10);
    Wire.endTransmission();
  }
 private:
  bool write8(uint8_t reg, uint8_t v) {
    Wire.beginTransmission(addr_); Wire.write(reg); Wire.write(v);
    return Wire.endTransmission() == 0;
  }
  uint8_t addr_;
};
