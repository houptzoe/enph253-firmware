#pragma once

#include <Arduino.h>

#include "hardware/pins.h"

class StatusLeds {
 public:
  void begin();
  void off();
  void startBlink(int8_t camera, uint8_t blinks, uint32_t onMs, uint32_t offMs);
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
