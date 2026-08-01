#pragma once

#include <Arduino.h>

#include "motor/motor.h"
#include "sensors/vision.h"

// Dual-cam teletubby handshake test: tape-follow while searching; on DETECT
// (cam0 or cam1) stop 1 s then resume tape-follow.

enum class MissionPhase : uint8_t {
  Idle,
  SearchTeletubby,
  PauseOnDetect,
  Cruise,  // post-detect tape-follow; mission still "active" until abort
};

struct MissionDriveCommand {
  enum class Mode : uint8_t { Stop, TapeFollow };

  Mode mode = Mode::Stop;
};

class MissionController {
 public:
  void begin(VisionInference& vision);

  void start();
  void abort();
  void update();

  MissionPhase phase() const { return phase_; }
  const char* phaseName() const;
  MissionDriveCommand driveCommand() const { return drive_; }
  bool active() const { return phase_ != MissionPhase::Idle; }
  bool searching() const { return phase_ == MissionPhase::SearchTeletubby; }
  // -1 = none yet this mission; 0 = cam0; 1 = cam1.
  int8_t lastDetectedCamera() const { return lastDetectedCamera_; }

 private:
  void setTapeFollow();
  void setStop();

  VisionInference* vision_ = nullptr;
  MissionPhase phase_ = MissionPhase::Idle;
  MissionDriveCommand drive_{};
  uint32_t pauseEndMs_ = 0;
  int8_t lastDetectedCamera_ = -1;
};
