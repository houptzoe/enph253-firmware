#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "sensors/imu.h"

// SSD1306 helper for the IMU bench test: text pose + top-down heading glyph.

class ImuPoseDisplay {
 public:
  void begin(TwoWire& oledWire);
  void showPose(const ImuPose& pose);
  void showStatus(const char* line1, const char* line2 = nullptr);

 private:
  static constexpr int kScreenWidth = 128;
  static constexpr int kScreenHeight = 64;
  static constexpr uint32_t kMinUpdateMs = 50;

  uint32_t lastUpdateMs_ = 0;
  bool ready_ = false;
};
