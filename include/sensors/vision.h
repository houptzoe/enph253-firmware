#pragma once

#include <Arduino.h>

#include "hardware/pins.h"

// Dual-cam teletubby detections via Pi ↔ ESP GPIO handshake
// (lib/ESP32-GPIO-HANDSHAKE.md), plus optional Serial/bench inject().
// Pi pulses DETECT once per find and exits after MissionConfig::kRequiredDetects.

struct VisionDetectResult {
  bool found = false;
  int8_t camera = -1;      // 0 = cam0 (GPIO3), 1 = cam1 (GPIO4), -1 = none
  uint8_t detectCount = 0; // 1..kRequiredDetects after this pulse
};

class VisionInference {
 public:
  void begin();

  // enable(true): START rising edge on Pi GPIO4, then release bus for DETECT_CAM1.
  // enable(false): drive GPIO4 LOW (re-arm) after cooldown from last mission.
  void enable(bool on);
  bool enabled() const { return enabled_; }

  VisionDetectResult poll();

  // Detects accepted this mission (0 until first pulse).
  uint8_t detectCount() const { return detectCount_; }

  // Bench inject without the Pi. camera: 0 or 1. Each inject counts as one find.
  void inject(int8_t camera = 0);
  void clearInject();

 private:
  enum class Phase : uint8_t { Idle, WaitDetect, Cooldown };

  void armIdle();
  void startSearch();
  void enterCooldown();
  void finishCooldownIfReady();
  void acceptDetect(int8_t camera, VisionDetectResult& out);

  Phase phase_ = Phase::Idle;
  bool enabled_ = false;
  bool pendingInject_ = false;
  int8_t pendingInjectCamera_ = 0;
  int8_t foundCamera_ = -1;
  uint8_t detectCount_ = 0;

  // A line only counts as a DETECT once the Pi has held it LOW at least once;
  // both idle HIGH via pull-ups before mars-cv drives them. After each accept,
  // that line is disarmed until LOW again so the rest of the 100 ms pulse
  // is not counted twice.
  bool cam0Armed_ = false;
  bool cam1Armed_ = false;
  bool armWarned_ = false;

  uint32_t high3SinceMs_ = 0;
  uint32_t high4SinceMs_ = 0;
  uint32_t searchStartedMs_ = 0;
  uint32_t cooldownStartedMs_ = 0;
};
