#pragma once

#include <Arduino.h>

// Tunables for the dual-cam teletubby handshake (lib/ESP32-GPIO-HANDSHAKE.md).

namespace MissionConfig {

// Floor for left/right base speed (telemetry + search cruise).
constexpr float kMinBaseSpeed = 70.0f;

// Base speed while searching for teletubby (tape-follow).
constexpr float kTeletubbySearchBaseSpeed = kMinBaseSpeed;

// Arrow LED blink on each DETECT (cam0 → left LED, cam1 → right LED).
// Motors stay stopped until the blink sequence finishes, then resume.
constexpr uint8_t kArrowBlinkCount = 3;
constexpr uint32_t kArrowBlinkOnMs = 200;
constexpr uint32_t kArrowBlinkOffMs = 200;

// Robot-side teletubby count across two independent STARTs (one DETECT each).
constexpr uint8_t kRequiredFinds = 2;

// Ignore DETECT glitches shorter than this (Pi pulse is ~100 ms active HIGH).
constexpr uint32_t kDetectMinPulseMs = 50;

// Hold START HIGH this long before releasing GPIO4 for DETECT_CAM1 (1–5 ms).
constexpr uint32_t kStartReleaseDelayMs = 2;

// After DETECT the Pi returns to idle in-process (typically <1 s). Wait at
// least this long before re-arming GPIO4 LOW and issuing the next START.
constexpr uint32_t kPiIdleMs = 1000;

// Both Pi lines may idle HIGH (BCM3 has a 1.8k hardware pull-up, BCM4 a
// default internal one) until mars-cv claims them as LOW outputs. Ignore
// detects until then, and until the Pi finishes its ~5-frame warmup (~0.3 s).
constexpr uint32_t kDetectBlankingMs = 500;

// Warn once if a line never goes LOW — mars-cv likely is not running.
constexpr uint32_t kDetectArmWarnMs = 4000;

}  // namespace MissionConfig
