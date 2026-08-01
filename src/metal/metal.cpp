#include "metal/metal.h"

// ---------------------------------------------------------------------------
// ISRs — count RISING edges per side; kept tiny and IRAM-resident.
// ---------------------------------------------------------------------------

void IRAM_ATTR MetalDetector::pulseIsrLeft(void* arg) {
  auto* self = static_cast<MetalDetector*>(arg);
  portENTER_CRITICAL_ISR(&self->pulseMux_);
  self->pulseCountLeft_++;
  portEXIT_CRITICAL_ISR(&self->pulseMux_);
}

void IRAM_ATTR MetalDetector::pulseIsrRight(void* arg) {
  auto* self = static_cast<MetalDetector*>(arg);
  portENTER_CRITICAL_ISR(&self->pulseMux_);
  self->pulseCountRight_++;
  portEXIT_CRITICAL_ISR(&self->pulseMux_);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void MetalDetector::begin(const MetalDetectorConfig& config) {
  config_ = config;
  state_ = MetalDetectorState{};

  pinMode(config_.leftPin, INPUT);
  pinMode(config_.rightPin, INPUT);

  attachInterruptArg(digitalPinToInterrupt(config_.leftPin), pulseIsrLeft,
                      this, RISING);
  attachInterruptArg(digitalPinToInterrupt(config_.rightPin), pulseIsrRight,
                      this, RISING);

  pulseCountLeft_ = 0;
  pulseCountRight_ = 0;
  gateStartMs_ = millis();
}

// ---------------------------------------------------------------------------
// Blocking calibration — robot must be stationary and away from metal.
// ---------------------------------------------------------------------------

void MetalDetector::calibrate() {
  float sumL = 0.0f;
  float sumR = 0.0f;

  for (uint16_t i = 0; i < config_.calibrationSamples; i++) {
    portENTER_CRITICAL(&pulseMux_);
    pulseCountLeft_ = 0;
    pulseCountRight_ = 0;
    portEXIT_CRITICAL(&pulseMux_);

    const uint32_t start = millis();
    while (millis() - start < config_.gateTimeMs) {
    }

    uint32_t countL = 0;
    uint32_t countR = 0;
    portENTER_CRITICAL(&pulseMux_);
    countL = pulseCountLeft_;
    countR = pulseCountRight_;
    portEXIT_CRITICAL(&pulseMux_);

    const float scale = 1000.0f / static_cast<float>(config_.gateTimeMs);
    sumL += static_cast<float>(countL) * scale;
    sumR += static_cast<float>(countR) * scale;
  }

  state_.baselineLeft = sumL / static_cast<float>(config_.calibrationSamples);
  state_.baselineRight = sumR / static_cast<float>(config_.calibrationSamples);

  portENTER_CRITICAL(&pulseMux_);
  pulseCountLeft_ = 0;
  pulseCountRight_ = 0;
  portEXIT_CRITICAL(&pulseMux_);
  gateStartMs_ = millis();
}

// ---------------------------------------------------------------------------
// Non-blocking gate — one reading per gateTimeMs, driven off millis().
// ---------------------------------------------------------------------------

bool MetalDetector::update(MetalDetectorState& state) {
  const uint32_t now = millis();
  const uint32_t elapsedMs = now - gateStartMs_;
  if (elapsedMs < config_.gateTimeMs) {
    return false;
  }

  uint32_t countL = 0;
  uint32_t countR = 0;
  portENTER_CRITICAL(&pulseMux_);
  countL = pulseCountLeft_;
  countR = pulseCountRight_;
  pulseCountLeft_ = 0;
  pulseCountRight_ = 0;
  portEXIT_CRITICAL(&pulseMux_);

  gateStartMs_ = now;

  const float scale = 1000.0f / static_cast<float>(elapsedMs);
  state_.leftHz = static_cast<float>(countL) * scale;
  state_.rightHz = static_cast<float>(countR) * scale;

  const float deltaL = fabsf(state_.leftHz - state_.baselineLeft);
  const float deltaR = fabsf(state_.rightHz - state_.baselineRight);
  const bool leftHit = deltaL > config_.thresholdHz;
  const bool rightHit = deltaR > config_.thresholdHz;

  if (leftHit && rightHit) {
    state_.side = (deltaL >= deltaR) ? MetalSide::Left : MetalSide::Right;
  } else if (leftHit) {
    state_.side = MetalSide::Left;
  } else if (rightHit) {
    state_.side = MetalSide::Right;
  } else {
    state_.side = MetalSide::None;
  }

  state = state_;
  return true;
}