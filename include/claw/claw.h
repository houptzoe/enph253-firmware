#pragma once

#include <Arduino.h>

// Claw servo on a single PWM pin (50 Hz, angle via pulse width).

class ClawServo {
 public:
  void begin(int pin);
  void setAngle(int degrees);
  void openClaw();
  void closeClaw();

  // Tune open/close on the bench.
  static constexpr int kOpenAngleDeg = 130;
  static constexpr int kCloseAngleDeg = 80;

 private:
  static constexpr int kPwmChannel = 4;  // avoid drive-motor channels 0–3
  static constexpr int kPwmFreqHz = 50;
  static constexpr int kPwmResolutionBits = 14;
  static constexpr int kMinPulseUs = 500;
  static constexpr int kMaxPulseUs = 2500;

  int pin_ = -1;
  uint32_t maxDuty_ = 0;
};
