#pragma once

#include <Arduino.h>

// Tunables for the dual-cam teletubby handshake (lib/ESP32-GPIO-HANDSHAKE.md).

namespace MissionConfig {

// Floor for left/right base speed (telemetry + search cruise).
constexpr float kMinBaseSpeed = 70.0f;

// Base speed while searching for teletubby (tape-follow).
constexpr float kTeletubbySearchBaseSpeed = kMinBaseSpeed;

// Hold motors stopped this long after DETECT before resuming tape-follow.
constexpr uint32_t kTeletubbyStopMs = 1000;

// Ignore DETECT glitches shorter than this (Pi pulse is ~100 ms active HIGH).
constexpr uint32_t kDetectMinPulseMs = 50;

// Hold START HIGH this long before releasing GPIO4 for DETECT_CAM1 (1–5 ms).
constexpr uint32_t kStartReleaseDelayMs = 2;

// Pi systemd RestartSec=3 — wait before next START after DETECT.
constexpr uint32_t kPiCooldownMs = 3500;

// Both Pi lines idle HIGH (BCM3 has a 1.8k hardware pull-up, BCM4 a default
// internal one) until mars-cv claims them as LOW outputs. Ignore detects until
// then, and until the Pi finishes its ~20-frame warmup.
constexpr uint32_t kDetectBlankingMs = 1500;

// Warn once if a line never goes LOW — mars-cv likely is not running.
constexpr uint32_t kDetectArmWarnMs = 4000;

}  // namespace MissionConfig
