#include "sensors/vision.h"

#include "hardware/mission_config.h"

void VisionInference::begin() {
  pinMode(kPiStartPin, OUTPUT);
  digitalWrite(kPiStartPin, LOW);  // idle low; rising edge starts the Pi

  pinMode(kPiDetectPin, INPUT_PULLDOWN);
  enabled_ = false;
  pendingInject_ = false;
  detectHigh_ = false;
  detectFired_ = false;
  detectHighSinceMs_ = 0;
}

void VisionInference::enable(bool on) {
  if (on) {
    // Rising edge on START wakes mars-cv (idle was low).
    digitalWrite(kPiStartPin, LOW);
    delay(2);
    digitalWrite(kPiStartPin, HIGH);
    enabled_ = true;
    detectHigh_ = false;
    detectFired_ = false;
    detectHighSinceMs_ = 0;
    Serial.println("[VISION] START rising edge -> Pi search");
  } else {
    digitalWrite(kPiStartPin, LOW);
    enabled_ = false;
    pendingInject_ = false;
    detectHigh_ = false;
    detectFired_ = false;
  }
}

VisionDetectResult VisionInference::poll() {
  VisionDetectResult out;

  if (pendingInject_) {
    out.found = true;
    pendingInject_ = false;
    return out;
  }

  if (!enabled_) {
    return out;
  }

  const int level = digitalRead(kPiDetectPin);
  const bool detectHit = MissionConfig::kPiDetectActiveHigh ? level == HIGH
                                                            : level == LOW;

  if (detectHit) {
    if (!detectHigh_) {
      detectHigh_ = true;
      detectHighSinceMs_ = millis();
    } else if (!detectFired_ &&
               (millis() - detectHighSinceMs_) >=
                   MissionConfig::kDetectMinPulseMs) {
      out.found = true;
      detectFired_ = true;
    }
  } else {
    detectHigh_ = false;
    detectFired_ = false;
  }

  return out;
}

void VisionInference::inject() { pendingInject_ = true; }

void VisionInference::clearInject() { pendingInject_ = false; }
