#pragma once

#include <Arduino.h>

// SSD1306 OLED — metal, reflectance, sonar, and IMU turn, plus messages.

class ReflectanceDisplay {
 public:
  void begin();
  void showStatus(float leftHz, float rightHz, float baselineLeft,
                  float baselineRight, bool metalLeftHit, bool metalRightHit,
                  int leftAnalog, int rightAnalog, float distanceCm,
                  bool distanceValid, float turnedDeg);
  void showMetalHit(char side, float baselineHz, float deltaHz);
  void showMessage(const char* line1, const char* line2 = "");
  // Live HC-SR04 reading while RotateScan is aiming.
  void showSonarScan(float distanceCm, bool valid, float lockMinCm,
                     float lockMaxCm);

 private:
  static constexpr uint32_t kMinUpdateMs = 50;  // limit I2C refresh rate

  uint32_t lastUpdateMs_ = 0;
};
