#pragma once

#include <Arduino.h>

#include "hardware/pins.h"

// Teletubby detections via Pi ↔ ESP GPIO handshake (see pi-handshake.md),
// plus optional Serial/bench inject().

struct VisionDetectResult {
  bool found = false;
};

class VisionInference {
 public:
  void begin();

  // enable(true): assert START rising edge so the Pi begins inference.
  // enable(false): drop START LOW to re-arm for the next mission.
  void enable(bool on);
  bool enabled() const { return enabled_; }

  VisionDetectResult poll();

  void inject();
  void clearInject();

 private:
  bool enabled_ = false;
  bool pendingInject_ = false;
  bool detectHigh_ = false;
  bool detectFired_ = false;
  uint32_t detectHighSinceMs_ = 0;
};
