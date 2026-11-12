// Per-servo calibration (direction, trim, limits) in EEPROM with magic + checksum.
#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include "src/servo.h"

struct CalBlob {
  uint32_t magic;
  spider::ServoCal cal[spider::kServos];
  uint16_t sum;
};
static const uint32_t kCalMagic = 0x53504431UL;  // "SPD1"

static uint16_t calSum(const CalBlob& b) {
  const uint8_t* p = (const uint8_t*)&b;
  uint16_t s = 0;
  for (size_t i = 0; i < sizeof(CalBlob) - sizeof(uint16_t); i++) s = (uint16_t)(s * 31 + p[i] + 1);
  return s;
}

static bool loadCal(spider::ServoCal cal[spider::kServos]) {
  CalBlob b; EEPROM.get(0, b);
  if (b.magic != kCalMagic || b.sum != calSum(b)) return false;
  for (int i = 0; i < spider::kServos; i++) {
    const spider::ServoCal& c = b.cal[i];
    // reject NaN/Inf (comparisons with NaN are false), bad direction, out-of-range trim or limits
    if (c.dir != 1 && c.dir != -1) return false;
    if (!(c.trim_deg >= -30.0f && c.trim_deg <= 30.0f)) return false;
    if (!(c.min_deg >= 0.0f && c.max_deg <= 180.0f && c.min_deg < c.max_deg)) return false;
  }
  for (int i = 0; i < spider::kServos; i++) cal[i] = b.cal[i];
  return true;
}

static void saveCal(const spider::ServoCal cal[spider::kServos]) {
  CalBlob b; memset(&b, 0, sizeof(b));
  b.magic = kCalMagic;
  for (int i = 0; i < spider::kServos; i++) b.cal[i] = cal[i];
  b.sum = calSum(b);
  EEPROM.put(0, b);
}
