#include "sensors/vision.h"

#include "hardware/mission_config.h"

void VisionInference::rearmPins() {
  // Drive multiplexed Pi GPIO4 wire LOW so the next START can rise.
  pinMode(kPiCam1StartPin, OUTPUT);
  digitalWrite(kPiCam1StartPin, LOW);
  pinMode(kPiCam0Pin, INPUT_PULLDOWN);

  high3SinceMs_ = 0;
  high4SinceMs_ = 0;
  cam0Armed_ = false;
  cam1Armed_ = false;
  armWarned_ = false;
  phase_ = Phase::Idle;
  enabled_ = false;
}

void VisionInference::resetMission() {
  rearmPins();
  foundCamera_ = -1;
  detectCount_ = 0;
}

void VisionInference::begin() {
  pendingInject_ = false;
  pendingInjectCamera_ = 0;
  postDetectStartedMs_ = 0;
  resetMission();
}

void VisionInference::reset() {
  pendingInject_ = false;
  postDetectStartedMs_ = 0;
  resetMission();
}

void VisionInference::enterPostDetect() {
  // Reclaim the mux wire; Pi returns to idle in-process (typically <1 s).
  pinMode(kPiCam1StartPin, OUTPUT);
  digitalWrite(kPiCam1StartPin, LOW);
  pinMode(kPiCam0Pin, INPUT_PULLDOWN);
  high3SinceMs_ = 0;
  high4SinceMs_ = 0;
  cam0Armed_ = false;
  cam1Armed_ = false;
  postDetectStartedMs_ = millis();
  phase_ = Phase::PostDetect;
  enabled_ = false;
}

void VisionInference::finishPostDetectIfReady() {
  if (phase_ != Phase::PostDetect) {
    return;
  }
  if (millis() - postDetectStartedMs_ >= MissionConfig::kPiIdleMs) {
    rearmPins();
  }
}

void VisionInference::startSearch() {
  // Ensure a clean rising edge from a driven LOW.
  pinMode(kPiCam1StartPin, OUTPUT);
  digitalWrite(kPiCam1StartPin, LOW);
  delayMicroseconds(100);
  digitalWrite(kPiCam1StartPin, HIGH);
  delay(MissionConfig::kStartReleaseDelayMs);

  // Release so the Pi can reclaim GPIO4 as DETECT_CAM1 (~20 ms later).
  pinMode(kPiCam1StartPin, INPUT);
  pinMode(kPiCam0Pin, INPUT_PULLDOWN);

  foundCamera_ = -1;
  high3SinceMs_ = 0;
  high4SinceMs_ = 0;
  cam0Armed_ = false;
  cam1Armed_ = false;
  armWarned_ = false;
  searchStartedMs_ = millis();
  phase_ = Phase::WaitDetect;
  enabled_ = true;
  Serial.printf(
      "[VISION] START rising edge -> dual search (1 detect/session); "
      "finds so far %u/%u; GPIO4 released\n",
      static_cast<unsigned>(detectCount_),
      static_cast<unsigned>(MissionConfig::kRequiredFinds));
}

void VisionInference::enable(bool on) {
  if (on) {
    finishPostDetectIfReady();
    if (phase_ == Phase::PostDetect) {
      const uint32_t elapsed = millis() - postDetectStartedMs_;
      const uint32_t remaining =
          (elapsed >= MissionConfig::kPiIdleMs)
              ? 0
              : (MissionConfig::kPiIdleMs - elapsed);
      Serial.printf("[VISION] waiting %lu ms Pi idle before START\n",
                    static_cast<unsigned long>(remaining));
      delay(remaining);
      rearmPins();
    } else if (phase_ != Phase::Idle) {
      rearmPins();
    }
    startSearch();
  } else {
    // poll() may already have entered PostDetect on DETECT — keep the timer.
    if (phase_ == Phase::PostDetect) {
      pinMode(kPiCam1StartPin, OUTPUT);
      digitalWrite(kPiCam1StartPin, LOW);
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

  // Disarm both lines until we see LOW again — avoids counting the remainder
  // of this ~100 ms pulse (or a simultaneous edge) twice.
  high3SinceMs_ = 0;
  high4SinceMs_ = 0;
  cam0Armed_ = false;
  cam1Armed_ = false;

  Serial.printf(
      "[VISION] DETECT_CAM%d (GPIO%d) — teletubby #%u/%u (session done)\n",
      static_cast<int>(camera), camera == 0 ? 3 : 4,
      static_cast<unsigned>(detectCount_),
      static_cast<unsigned>(MissionConfig::kRequiredFinds));

  // One DETECT per START — Pi returns to idle; re-arm for next START if any.
  enterPostDetect();
}

VisionDetectResult VisionInference::poll() {
  VisionDetectResult out;

  finishPostDetectIfReady();

  if (pendingInject_) {
    pendingInject_ = false;
    if (phase_ == Phase::WaitDetect) {
      acceptDetect(pendingInjectCamera_, out);
    }
    return out;
  }

  if (phase_ != Phase::WaitDetect) {
    return out;
  }

  const uint32_t now = millis();
  const uint32_t sinceStart = now - searchStartedMs_;
  const bool blanking = sinceStart < MissionConfig::kDetectBlankingMs;

  const bool cam0High = digitalRead(kPiCam0Pin) == HIGH;
  const bool cam1High = digitalRead(kPiCam1StartPin) == HIGH;

  if (!cam0High) {
    cam0Armed_ = true;
    high3SinceMs_ = 0;
  }
  if (!cam1High) {
    cam1Armed_ = true;
    high4SinceMs_ = 0;
  }

  if (!armWarned_ && sinceStart >= MissionConfig::kDetectArmWarnMs &&
      (!cam0Armed_ || !cam1Armed_)) {
    armWarned_ = true;
    Serial.printf(
        "[VISION] line still HIGH after %lu ms (cam0 armed:%d cam1 armed:%d) — "
        "is mars-cv running and I2C disabled on the Pi?\n",
        static_cast<unsigned long>(sinceStart), cam0Armed_ ? 1 : 0,
        cam1Armed_ ? 1 : 0);
  }

  // DETECT_CAM0 on Pi GPIO3 (dedicated input).
  if (cam0High && cam0Armed_ && !blanking) {
    if (high3SinceMs_ == 0) {
      high3SinceMs_ = now;
    } else if (now - high3SinceMs_ >= MissionConfig::kDetectMinPulseMs) {
      acceptDetect(0, out);
      return out;
    }
  }

  // DETECT_CAM1 on Pi GPIO4 (same wire we released after START).
  if (phase_ == Phase::WaitDetect && cam1High && cam1Armed_ && !blanking) {
    if (high4SinceMs_ == 0) {
      high4SinceMs_ = now;
    } else if (now - high4SinceMs_ >= MissionConfig::kDetectMinPulseMs) {
      acceptDetect(1, out);
      return out;
    }
  }

  return out;
}

void VisionInference::inject(int8_t camera) {
  pendingInject_ = true;
  pendingInjectCamera_ = (camera == 1) ? 1 : 0;
}

void VisionInference::clearInject() { pendingInject_ = false; }
