#pragma once

#include <Arduino.h>

#include "hardware/status_leds.h"
#include "motor/motor.h"
#include "sensors/vision.h"

// Dual-cam teletubby handshake: tape-follow while searching; on each DETECT
// stop, blink the camera-side arrow LED 3×, then resume. After the 2nd DETECT
// (Pi shuts off), cruise.

enum class MissionPhase : uint8_t {
  Idle,
  SearchTeletubby,
  PauseOnDetect,
  Cruise,  // after required detects; mission still "active" until abort
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
  // 0 until first pulse; then 1..kRequiredDetects.
  uint8_t detectCount() const { return detectCount_; }

 private:
  void setTapeFollow();
  void setStop();
  void onDetect(const VisionDetectResult& vision);
  void finishPause();

  VisionInference* vision_ = nullptr;
  StatusLeds leds_{};
  MissionPhase phase_ = MissionPhase::Idle;
  MissionDriveCommand drive_{};
  int8_t lastDetectedCamera_ = -1;
  uint8_t detectCount_ = 0;
};
