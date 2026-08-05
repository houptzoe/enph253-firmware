#include "sonar/sonar.h"

void UltrasonicSonar::begin(const UltrasonicConfig& config) {
  config_ = config;
  pinMode(config_.trigPin, OUTPUT);
  pinMode(config_.echoPin, INPUT);
  digitalWrite(config_.trigPin, LOW);
  lastPingMs_ = 0;
  lastDistanceCm_ = 0.0f;
  lastValid_ = false;
}

bool UltrasonicSonar::ping(float& distanceCm, bool& valid) {
  lastPingMs_ = millis();

  digitalWrite(config_.trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(config_.trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(config_.trigPin, LOW);

  const unsigned long durationUs =
      pulseIn(config_.echoPin, HIGH, config_.echoTimeoutUs);

  if (durationUs == 0) {
    lastValid_ = false;
    lastDistanceCm_ = 0.0f;
  } else {
    // Round-trip time → one-way distance (speed of sound ≈ 343 m/s).
    lastDistanceCm_ = static_cast<float>(durationUs) / 58.0f;
    lastValid_ = true;
  }

  distanceCm = lastDistanceCm_;
  valid = lastValid_;
  return true;
}

bool UltrasonicSonar::update(float& distanceCm, bool& valid) {
  const uint32_t nowMs = millis();
  if (lastPingMs_ != 0 && (nowMs - lastPingMs_) < config_.samplePeriodMs) {
    return false;
  }
  return ping(distanceCm, valid);
}
