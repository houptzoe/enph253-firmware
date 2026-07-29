#pragma once

#include <Arduino.h>

// Simple DIR/STEP stepper driver (one pulse per step).

class StepperMotor {
 public:
  void begin(int dirPin, int stepPin);
  // dirHigh: DIR pin level for the positive direction of this call.
  void step(int steps, bool dirHigh);

  // Half-period of each STEP pulse (HIGH then LOW). Tune for torque/speed.
  static constexpr uint32_t kPulseHalfPeriodUs = 800;

 private:
  int dirPin_ = -1;
  int stepPin_ = -1;
};
