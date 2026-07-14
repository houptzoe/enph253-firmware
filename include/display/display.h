#pragma once

#include <Arduino.h>

// SSD1306 OLED — shows reflectance analog readings and on/off tape state.

class ReflectanceDisplay {
 public:
  void begin();
  void showReadings(int leftAvg, int rightAvg, bool leftOnTape, bool rightOnTape);

 private:
  static constexpr uint32_t kMinUpdateMs = 50;  // limit I2C refresh rate

  uint32_t lastUpdateMs_ = 0;
};
