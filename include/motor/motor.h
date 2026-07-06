#pragma once

#include <Arduino.h>

// Two-wheel motor driver using ESP32 LEDC PWM and H-bridge direction GPIOs.

class MotorDriver {
 public:
  void begin();
  void applyDrive(float leftSpeed, float rightSpeed);  // signed: +forward, -reverse
  void stop();

  static constexpr int kPwmMax = (1 << kPwmResolutionBits) - 1;

 private:
  static constexpr int kPwmFreqHz = 20000;
  static constexpr int kPwmResolutionBits = 8;

  void setMotor(int pwmPin, int dirPin, float speed);
};
