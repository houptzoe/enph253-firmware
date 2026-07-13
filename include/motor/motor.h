#pragma once

#include <Arduino.h>

#include "hardware/pins.h"

// Two-wheel drivetrain using per-bridge two-wire H-bridge PWM (test/pwm scheme).
// Each side has two PWM pins: forward drives pin0, reverse drives pin1.

class MotorDriver {
 public:
  void begin();
  void applyDrive(float leftSpeed, float rightSpeed);  // signed: +forward, -reverse
  void stop();

  // Speed input range used by main (maps to voltage-limited PWM duty internally).
  static constexpr int kSpeedMax = 255;
  static constexpr int kPwmMax = kSpeedMax;

 private:
  static constexpr int kPwmFreqHz = 20000;
  static constexpr int kPwmResolutionBits = 10;
  static constexpr uint32_t kSwitchDeadtimeMs = 10;

  struct Bridge {
    int pin0;
    int pin1;
    int channel0;
    int channel1;
    int direction;  // -1 reverse, 0 stopped, +1 forward

    Bridge(int p0, int p1, int c0, int c1)
        : pin0(p0), pin1(p1), channel0(c0), channel1(c1), direction(0) {}
  };

  void initBridge(Bridge& bridge);
  void bridgeAllOff(const Bridge& bridge);
  void setBridgeSpeed(Bridge& bridge, float speed);
  int speedToPercent(float speed) const;
  uint32_t percentToDuty(int speedPercent) const;

  Bridge left_{kLeftMotorPwm0Pin, kLeftMotorPwm1Pin, 0, 1};
  Bridge right_{kRightMotorPwm0Pin, kRightMotorPwm1Pin, 2, 3};
  uint32_t maxDuty_ = 0;
};
