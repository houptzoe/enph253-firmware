#include "metal/metal.h"

namespace {

volatile uint32_t gEdgeCount = 0;

void IRAM_ATTR onEdgeIsr() { gEdgeCount++; }

}  // namespace

float measureFreqHz(int pin, uint32_t windowMs) {
  pinMode(pin, INPUT);
  gEdgeCount = 0;
  attachInterrupt(digitalPinToInterrupt(pin), onEdgeIsr, RISING);
  delay(windowMs);
  detachInterrupt(digitalPinToInterrupt(pin));
  const uint32_t edges = gEdgeCount;
  if (windowMs == 0) {
    return 0.0f;
  }
  return (static_cast<float>(edges) * 1000.0f) / static_cast<float>(windowMs);
}

MetalBaseline sampleBaseline(int leftPin, int rightPin, uint32_t windowMs) {
  MetalBaseline baseline;
  baseline.leftHz = measureFreqHz(leftPin, windowMs);
  baseline.rightHz = measureFreqHz(rightPin, windowMs);
  return baseline;
}

MetalSide detectMetalSide(float baselineLeftHz, float baselineRightHz,
                          float liveLeftHz, float liveRightHz,
                          float thresholdHz) {
  const float deltaL = fabsf(liveLeftHz - baselineLeftHz);
  const float deltaR = fabsf(liveRightHz - baselineRightHz);
  const bool leftHit = deltaL >= thresholdHz;
  const bool rightHit = deltaR >= thresholdHz;

  if (!leftHit && !rightHit) {
    return MetalSide::kNone;
  }
  if (leftHit && rightHit) {
    return (deltaL >= deltaR) ? MetalSide::kLeft : MetalSide::kRight;
  }
  return leftHit ? MetalSide::kLeft : MetalSide::kRight;
}
