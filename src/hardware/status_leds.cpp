#include "hardware/status_leds.h"

void StatusLeds::begin() {
  pinMode(kLedLeftPin, OUTPUT);
  pinMode(kLedRightPin, OUTPUT);
  off();
}

void StatusLeds::off() {
  digitalWrite(kLedLeftPin, LOW);
  digitalWrite(kLedRightPin, LOW);
  activePin_ = -1;
  phase_ = 0;
  phaseEnd_ = 0;
}

void StatusLeds::startBlink(int8_t camera, uint8_t blinks, uint32_t onMs,
                            uint32_t offMs) {
  off();
  if (blinks == 0) {
    return;
  }
  activePin_ = (camera == 0) ? kLedLeftPin : kLedRightPin;
  onMs_ = onMs;
  offMs_ = offMs;
  // Each blink is ON then OFF → 2 phases per blink.
  phase_ = 0;
  phaseEnd_ = static_cast<uint8_t>(blinks * 2);
  nextMs_ = millis();  // apply first ON immediately on next update()
}

bool StatusLeds::update() {
  if (phase_ >= phaseEnd_ || activePin_ < 0) {
    return true;
  }
  if (static_cast<int32_t>(millis() - nextMs_) < 0) {
    return false;
  }

  const bool on = (phase_ % 2) == 0;
  digitalWrite(activePin_, on ? HIGH : LOW);
  nextMs_ = millis() + (on ? onMs_ : offMs_);
  ++phase_;

  if (phase_ >= phaseEnd_) {
    digitalWrite(activePin_, LOW);
    activePin_ = -1;
    return true;
  }
  return false;
}
