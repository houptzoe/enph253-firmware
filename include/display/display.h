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

// SSD1306 OLED — metal-detector baseline / live frequency and detect status.

class MetalDisplay {
 public:
  void begin();
  void show(float baselineLeftHz, float baselineRightHz, float liveLeftHz,
            float liveRightHz, const char* status);

 private:
  static constexpr uint32_t kMinUpdateMs = 100;

  uint32_t lastUpdateMs_ = 0;
  bool forceNext_ = true;
};
