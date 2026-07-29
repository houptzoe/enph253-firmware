#pragma once

#include <Arduino.h>

// Digital pulse-frequency metal detector helpers (left / right pins).

enum class MetalSide : int8_t {
  kNone = 0,
  kLeft = 1,
  kRight = 2,
};

struct MetalBaseline {
  float leftHz = 0.0f;
  float rightHz = 0.0f;
};

// Count rising edges over windowMs and return frequency in Hz.
float measureFreqHz(int pin, uint32_t windowMs = 80);

// Sample both detectors into a baseline.
MetalBaseline sampleBaseline(int leftPin, int rightPin, uint32_t windowMs = 80);

// Returns the side whose |live - baseline| first exceeds thresholdHz.
// Prefer the larger delta if both exceed.
MetalSide detectMetalSide(float baselineLeftHz, float baselineRightHz,
                          float liveLeftHz, float liveRightHz,
                          float thresholdHz);
