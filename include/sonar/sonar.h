#pragma once

#include <Arduino.h>

#include "hardware/pins.h"

// HC-SR04 ultrasonic rangefinder — trigger pulse + echo timing.

struct UltrasonicConfig {
  int trigPin = kSonarTrigPin;
  int echoPin = kSonarEchoPin;
  uint32_t samplePeriodMs = 100;   // how often to ping
  uint32_t echoTimeoutUs = 25000;  // ~4 m max range
};

class UltrasonicSonar {
 public:
  void begin(const UltrasonicConfig& config = UltrasonicConfig{});

  // Non-blocking between pings. Returns true when a new sample is ready.
  // distanceCm is centimeters; valid=false means no echo / out of range.
  bool update(float& distanceCm, bool& valid);

  // Immediate blocking ping (ignores samplePeriodMs). Used during arm scan.
  bool ping(float& distanceCm, bool& valid);

  float lastDistanceCm() const { return lastDistanceCm_; }
  bool lastValid() const { return lastValid_; }

 private:
  UltrasonicConfig config_{};
  uint32_t lastPingMs_ = 0;
  float lastDistanceCm_ = 0.0f;
  bool lastValid_ = false;
};
