#include "mission/mission.h"

#include "hardware/mission_config.h"

void MissionController::begin(VisionInference& vision) {
  vision_ = &vision;
  phase_ = MissionPhase::Idle;
  setStop();
}

void MissionController::start() {
  if (vision_ == nullptr) {
    return;
  }
  Serial.println("[MISSION] start -> SearchTeletubby");
  phase_ = MissionPhase::SearchTeletubby;
  vision_->enable(true);
  setTapeFollow();
}

void MissionController::abort() {
  Serial.println("[MISSION] abort");
  if (vision_ != nullptr) {
    vision_->enable(false);
    vision_->clearInject();
  }
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

void MissionController::update() {
  switch (phase_) {
    case MissionPhase::SearchTeletubby: {
      const VisionDetectResult vision = vision_->poll();
      if (vision.found) {
        Serial.printf("[MISSION] teletubby DETECT — stop %lu ms, START LOW\n",
                      static_cast<unsigned long>(MissionConfig::kTeletubbyStopMs));
        vision_->clearInject();
        vision_->enable(false);
        phase_ = MissionPhase::PauseOnDetect;
        pauseEndMs_ = millis() + MissionConfig::kTeletubbyStopMs;
        setStop();
      } else {
        setTapeFollow();
      }
      break;
    }

    case MissionPhase::PauseOnDetect:
      setStop();
      if (static_cast<int32_t>(millis() - pauseEndMs_) >= 0) {
        Serial.println("[MISSION] resume tape-follow (Cruise)");
        phase_ = MissionPhase::Cruise;
        setTapeFollow();
      }
      break;

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
