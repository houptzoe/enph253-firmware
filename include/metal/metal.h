#pragma once

#include <Arduino.h>

#include "hardware/pins.h"

// Metal detector — two LC-oscillator coils (left/right). Each coil drives a
// digital pin whose toggle rate shifts when metal is nearby. We count RISING
// edges per side over a fixed gate window (via ISR) and compare against a
// calibrated no-metal baseline, all without blocking the main loop.

enum class MetalSide { None, Left, Right };

// Tunable parameters passed to MetalDetector::begin().
struct MetalDetectorConfig {
  int leftPin = kMetalDetectorLeftPin;
  int rightPin = kMetalDetectorRightPin;
  uint32_t gateTimeMs = 100;    // measurement window per reading
  float thresholdHz = 3000.0f;  // |delta from baseline| that counts as a hit
  uint16_t calibrationSamples = 20;
};

// Latest frequency readings and hit classification, produced each gate close.
struct MetalDetectorState {
  float leftHz = 0.0f;
  float rightHz = 0.0f;
  float baselineLeft = 0.0f;
  float baselineRight = 0.0f;
  MetalSide side = MetalSide::None;
};

class MetalDetector {
 public:
  void begin(const MetalDetectorConfig& config);

  // Blocking — call once at startup with the robot away from any metal.
  // Takes config.calibrationSamples readings (~calibrationSamples *
  // gateTimeMs) to establish the no-metal baseline frequency per side.
  void calibrate();

  // Non-blocking. Returns true when a gate window has just closed and
  // `state` has a fresh reading (call every loop iteration).
  bool update(MetalDetectorState& state);

 private:
  static void IRAM_ATTR pulseIsrLeft(void* arg);
  static void IRAM_ATTR pulseIsrRight(void* arg);

  MetalDetectorConfig config_{};
  MetalDetectorState state_{};

  portMUX_TYPE pulseMux_ = portMUX_INITIALIZER_UNLOCKED;
  volatile uint32_t pulseCountLeft_ = 0;
  volatile uint32_t pulseCountRight_ = 0;

  uint32_t gateStartMs_ = 0;
};
