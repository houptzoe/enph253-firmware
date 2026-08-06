#include "mission/mission.h"

#include "hardware/mission_config.h"

void MissionController::begin(VisionInference& vision) {
  vision_ = &vision;
  leds_.begin();
  phase_ = MissionPhase::Idle;
  lastDetectedCamera_ = -1;
  detectCount_ = 0;
  setStop();
}

void MissionController::start() {
  if (vision_ == nullptr) {
    return;
  }
  Serial.println("[MISSION] start -> SearchTeletubby");
  leds_.off();
  lastDetectedCamera_ = -1;
  detectCount_ = 0;
  phase_ = MissionPhase::SearchTeletubby;
  vision_->reset();
  vision_->enable(true);
  setTapeFollow();
}

void MissionController::abort() {
  Serial.println("[MISSION] abort");
  if (vision_ != nullptr) {
    vision_->enable(false);
    vision_->clearInject();
    vision_->reset();
  }
  leds_.off();
  lastDetectedCamera_ = -1;
  detectCount_ = 0;
  phase_ = MissionPhase::Idle;
  setStop();
}

const char* MissionController::phaseName() const {
  switch (phase_) {
    case MissionPhase::Idle:
      return "Idle";
    case MissionPhase::SearchTeletubby:
      return "SearchTT";
    case MissionPhase::PauseOnDetect:
      return "PauseTT";
    case MissionPhase::Cruise:
      return "Cruise";
    default:
      return "?";
  }
}

void MissionController::onDetect(const VisionDetectResult& vision) {
  lastDetectedCamera_ = vision.camera;
  detectCount_ = vision.detectCount;
  const bool complete = detectCount_ >= MissionConfig::kRequiredFinds;
  const char* side = (vision.camera == 0) ? "LEFT" : "RIGHT";

  Serial.printf(
      "[MISSION] teletubby DETECT cam%d #%u/%u — stop, blink %s arrow %u×%s\n",
      static_cast<int>(vision.camera),
      static_cast<unsigned>(detectCount_),
      static_cast<unsigned>(MissionConfig::kRequiredFinds), side,
      static_cast<unsigned>(MissionConfig::kArrowBlinkCount),
      complete ? ", vision done" : ", will START again after pause");

  vision_->clearInject();
  if (complete) {
    // Leave Pi idle; GPIO4 already driven LOW in PostDetect.
    vision_->enable(false);
  }

  leds_.startBlink(vision.camera, MissionConfig::kArrowBlinkCount,
                   MissionConfig::kArrowBlinkOnMs,
                   MissionConfig::kArrowBlinkOffMs);
  leds_.update();  // first ON immediately
  phase_ = MissionPhase::PauseOnDetect;
  setStop();
}

void MissionController::finishPause() {
  leds_.off();
  if (detectCount_ < MissionConfig::kRequiredFinds) {
    Serial.println("[MISSION] resume + START second teletubby search");
    phase_ = MissionPhase::SearchTeletubby;
    vision_->enable(true);  // new START; preserves find count
  } else {
    Serial.println("[MISSION] resume tape-follow (Cruise) — no more START");
    phase_ = MissionPhase::Cruise;
  }
  setTapeFollow();
}

void MissionController::update() {
  switch (phase_) {
    case MissionPhase::SearchTeletubby: {
      const VisionDetectResult vision = vision_->poll();
      if (vision.found) {
        onDetect(vision);
      } else {
        setTapeFollow();
      }
      break;
    }

    case MissionPhase::PauseOnDetect: {
      setStop();
      // Pi is idle after one DETECT — do not expect another pulse this session.
      // Keep polling so PostDetect → Idle timing advances.
      (void)vision_->poll();
      if (leds_.update()) {
        finishPause();
      }
      break;
    }

    case MissionPhase::Cruise:
      setTapeFollow();
      break;

    case MissionPhase::Idle:
    default:
      setStop();
      break;
  }
}

void MissionController::setTapeFollow() {
  drive_.mode = MissionDriveCommand::Mode::TapeFollow;
}

void MissionController::setStop() {
  drive_.mode = MissionDriveCommand::Mode::Stop;
}
