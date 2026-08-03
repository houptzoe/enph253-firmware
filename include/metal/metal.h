#pragma once

#include <Arduino.h>

#include "hardware/pins.h"

// Metal detector — two LC-oscillator coils (left/right). Hardware PCNT units
// count rising edges so OLED/I2C traffic cannot drop pulses the way GPIO ISRs
// do. Frequency is pulses * 1e6 / elapsed_us over a fixed gate window.

enum class MetalSide { None, Left, Right };

// Tunable parameters passed to MetalDetector::begin().
struct MetalDetectorConfig {
  int leftPin = kMetalDetectorLeftPin;
  int rightPin = kMetalDetectorRightPin;
  uint32_t gateTimeMs = 100;             // measurement window per reading
  float thresholdLeftHz = 500.0f;        // |delta from baseline| for left hit
  float thresholdRightHz = 500.0f;       // |delta from baseline| for right hit
  uint32_t baselineDurationMs = 3000;  // no-metal averaging window at boot
};

// Latest frequency readings and hit classification, produced each gate close.
struct MetalDetectorState {
  float leftHz = 0.0f;
  float rightHz = 0.0f;
  float baselineLeft = 0.0f;
  float baselineRight = 0.0f;
  float deltaLeftHz = 0.0f;   // |leftHz - baselineLeft|
  float deltaRightHz = 0.0f;  // |rightHz - baselineRight|
  bool leftHit = false;
  bool rightHit = false;
  MetalSide side = MetalSide::None;  // preferred side when either/both hit
};

class MetalDetector {
 public:
  void begin(const MetalDetectorConfig& config);

  // Blocking — call once at startup with the robot away from any metal and
  // standing still. Averages gate readings for config.baselineDurationMs
  // (default 3 s) to establish the no-metal baseline frequency per side.
  // Detection must not start until this returns.
  void calibrate();

  // Non-blocking. Returns true when a gate window has just closed and
  // `state` has a fresh reading (call every loop iteration).
  bool update(MetalDetectorState& state);

 private:
  bool setupPcnt(int pin, int unitIndex);
  uint32_t readAndClearPcnt(int unitIndex);

  MetalDetectorConfig config_{};
  MetalDetectorState state_{};

  bool pcntReady_ = false;
  uint32_t gateStartUs_ = 0;
};
