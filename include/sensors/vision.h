#pragma once

#include <Arduino.h>

#include "hardware/pins.h"

struct VisionDetectResult {
  bool found = false;
  int8_t camera = -1;      // 0 = cam0 (dedicated DETECT), 1 = cam1 (shared wire)
  uint8_t detectCount = 0; // accepted finds this mission
};

class VisionInference {
 public:
  void begin();
  void reset();

  // enable(true): issue START edge and wait for one DETECT.
  // enable(false): return to idle and drive START wire LOW.
  void enable(bool on);
  bool enabled() const { return enabled_; }

  VisionDetectResult poll();
  uint8_t detectCount() const { return detectCount_; }

 private:
  enum class Phase : uint8_t { Idle, WaitDetect, PostDetect };

  void resetMission();
  void rearmPins();
  void startSearch();
  void enterPostDetect();
  void finishPostDetectIfReady();
  void acceptDetect(int8_t camera, VisionDetectResult& out);

  Phase phase_ = Phase::Idle;
  bool enabled_ = false;
  int8_t foundCamera_ = -1;
  uint8_t detectCount_ = 0;

  bool detectArmed_ = false;
  bool sharedArmed_ = false;
  bool armWarned_ = false;
  uint32_t detectHighSinceMs_ = 0;
  uint32_t sharedHighSinceMs_ = 0;
  uint32_t searchStartedMs_ = 0;
  uint32_t postDetectStartedMs_ = 0;
};
