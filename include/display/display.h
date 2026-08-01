#pragma once

#include <Arduino.h>

// SSD1306 OLED — reflectance readings, or teletubby detect status.

class ReflectanceDisplay {
 public:
  void begin();
  void showReadings(int leftAvg, int rightAvg, bool leftOnTape, bool rightOnTape);
  void showTeletubbyDetected(int8_t camera);

 private:
  static constexpr uint32_t kMinUpdateMs = 50;  // limit I2C refresh rate

  uint32_t lastUpdateMs_ = 0;
};
