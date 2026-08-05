#pragma once

#include <Arduino.h>

#include "hardware/pins.h"

// Left/right arrow LEDs (cam0 → left, cam1 → right).

class StatusLeds {
 public:
  void begin();
  void off();

  // Start a non-blocking blink on left (cam0) or right (cam1).
  void startBlink(int8_t camera, uint8_t blinks, uint32_t onMs, uint32_t offMs);

  // Advance blink timing. Returns true when the sequence has finished (or idle).
  bool update();

  bool blinking() const { return phase_ < phaseEnd_; }

 private:
  int activePin_ = -1;
  uint8_t phase_ = 0;
  uint8_t phaseEnd_ = 0;
  uint32_t onMs_ = 0;
  uint32_t offMs_ = 0;
  uint32_t nextMs_ = 0;
};
