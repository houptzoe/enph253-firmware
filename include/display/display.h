#pragma once

#include <Arduino.h>

// SSD1306 OLED — metal-detector + tape-follow status, plus short messages.

class ReflectanceDisplay {
 public:
  void begin();
  void showStatus(float leftHz, float rightHz, float baselineLeft,
                  float baselineRight, bool metalLeftHit, bool metalRightHit,
                  bool leftOnTape, bool rightOnTape);
  void showMetalHit(char side, float baselineHz, float deltaHz);
  void showMessage(const char* line1, const char* line2 = "");

 private:
  static constexpr uint32_t kMinUpdateMs = 50;  // limit I2C refresh rate

  uint32_t lastUpdateMs_ = 0;
};
