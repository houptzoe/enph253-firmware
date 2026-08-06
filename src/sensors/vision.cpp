#include "sensors/vision.h"

namespace {

constexpr uint8_t kRequiredFinds = 2;
constexpr uint32_t kDetectMinPulseMs = 50;
constexpr uint32_t kStartReleaseDelayMs = 2;
constexpr uint32_t kPiIdleMs = 1000;
constexpr uint32_t kDetectBlankingMs = 500;
constexpr uint32_t kDetectArmWarnMs = 4000;

}  // namespace

void VisionInference::rearmPins() {
  pinMode(kPiStartPin, OUTPUT);
  digitalWrite(kPiStartPin, LOW);
  pinMode(kPiDetectPin, INPUT_PULLDOWN);

  detectHighSinceMs_ = 0;
  sharedHighSinceMs_ = 0;
  detectArmed_ = false;
  sharedArmed_ = false;
  armWarned_ = false;
  phase_ = Phase::Idle;
  enabled_ = false;
}

void VisionInference::resetMission() {
  rearmPins();
  foundCamera_ = -1;
  detectCount_ = 0;
}

void VisionInference::begin() { resetMission(); }

void VisionInference::reset() { resetMission(); }

void VisionInference::enterPostDetect() {
  pinMode(kPiStartPin, OUTPUT);
  digitalWrite(kPiStartPin, LOW);
  pinMode(kPiDetectPin, INPUT_PULLDOWN);

  detectHighSinceMs_ = 0;
  sharedHighSinceMs_ = 0;
  detectArmed_ = false;
  sharedArmed_ = false;
  postDetectStartedMs_ = millis();
  phase_ = Phase::PostDetect;
  enabled_ = false;
}

void VisionInference::finishPostDetectIfReady() {
  if (phase_ == Phase::PostDetect &&
      millis() - postDetectStartedMs_ >= kPiIdleMs) {
    rearmPins();
  }
}

void VisionInference::startSearch() {
  pinMode(kPiStartPin, OUTPUT);
  digitalWrite(kPiStartPin, LOW);
  delayMicroseconds(100);
  digitalWrite(kPiStartPin, HIGH);
  delay(kStartReleaseDelayMs);

  // Release shared wire so Pi can use it as DETECT cam1.
  pinMode(kPiStartPin, INPUT);
  pinMode(kPiDetectPin, INPUT_PULLDOWN);

  foundCamera_ = -1;
  detectHighSinceMs_ = 0;
  sharedHighSinceMs_ = 0;
  detectArmed_ = false;
  sharedArmed_ = false;
  armWarned_ = false;
  searchStartedMs_ = millis();
  phase_ = Phase::WaitDetect;
  enabled_ = true;

  Serial.printf("[VISION] START -> waiting detect, finds %u/%u\n",
                static_cast<unsigned>(detectCount_),
                static_cast<unsigned>(kRequiredFinds));
}

void VisionInference::enable(bool on) {
  if (on) {
    finishPostDetectIfReady();
    if (phase_ == Phase::PostDetect) {
      const uint32_t elapsed = millis() - postDetectStartedMs_;
      const uint32_t remaining = (elapsed >= kPiIdleMs) ? 0 : (kPiIdleMs - elapsed);
      if (remaining > 0) {
        Serial.printf("[VISION] waiting %lu ms before restart\n",
                      static_cast<unsigned long>(remaining));
        delay(remaining);
      }
      rearmPins();
    } else if (phase_ != Phase::Idle) {
      rearmPins();
    }
    startSearch();
  } else {
    if (phase_ == Phase::PostDetect) {
      pinMode(kPiStartPin, OUTPUT);
      digitalWrite(kPiStartPin, LOW);
    } else if (phase_ == Phase::WaitDetect || foundCamera_ >= 0) {
      enterPostDetect();
    } else {
      rearmPins();
    }
  }
}

void VisionInference::acceptDetect(int8_t camera, VisionDetectResult& out) {
  foundCamera_ = camera;
  ++detectCount_;
  out.found = true;
  out.camera = camera;
  out.detectCount = detectCount_;

  detectHighSinceMs_ = 0;
  sharedHighSinceMs_ = 0;
  detectArmed_ = false;
  sharedArmed_ = false;

  Serial.printf("[VISION] DETECT cam%d #%u/%u\n", static_cast<int>(camera),
                static_cast<unsigned>(detectCount_),
                static_cast<unsigned>(kRequiredFinds));

  enterPostDetect();
}

VisionDetectResult VisionInference::poll() {
  VisionDetectResult out;

  finishPostDetectIfReady();
  if (phase_ != Phase::WaitDetect) {
    return out;
  }

  const uint32_t now = millis();
  const uint32_t sinceStart = now - searchStartedMs_;
  const bool blanking = sinceStart < kDetectBlankingMs;

  const bool detectHigh = digitalRead(kPiDetectPin) == HIGH;
  const bool sharedHigh = digitalRead(kPiStartPin) == HIGH;

  if (!detectHigh) {
    detectArmed_ = true;
    detectHighSinceMs_ = 0;
  }
  if (!sharedHigh) {
    sharedArmed_ = true;
    sharedHighSinceMs_ = 0;
  }

  if (!armWarned_ && sinceStart >= kDetectArmWarnMs &&
      (!detectArmed_ || !sharedArmed_)) {
    armWarned_ = true;
    Serial.printf("[VISION] line still HIGH after %lu ms (detect armed:%d shared armed:%d)\n",
                  static_cast<unsigned long>(sinceStart), detectArmed_ ? 1 : 0,
                  sharedArmed_ ? 1 : 0);
  }

  if (detectHigh && detectArmed_ && !blanking) {
    if (detectHighSinceMs_ == 0) {
      detectHighSinceMs_ = now;
    } else if (now - detectHighSinceMs_ >= kDetectMinPulseMs) {
      acceptDetect(0, out);
      return out;
    }
  }

  if (phase_ == Phase::WaitDetect && sharedHigh && sharedArmed_ && !blanking) {
    if (sharedHighSinceMs_ == 0) {
      sharedHighSinceMs_ = now;
    } else if (now - sharedHighSinceMs_ >= kDetectMinPulseMs) {
      acceptDetect(1, out);
      return out;
    }
  }

  return out;
}
