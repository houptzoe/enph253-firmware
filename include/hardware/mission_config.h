#pragma once

#include <Arduino.h>

// Tunables for the teletubby handshake test branch (see pi-handshake.md).

namespace MissionConfig {

// Base speed while searching for teletubby (tape-follow).
constexpr float kTeletubbySearchBaseSpeed = 65.0f;

// Hold motors stopped this long after DETECT before resuming tape-follow.
constexpr uint32_t kTeletubbyStopMs = 1000;

// Pi DETECT polarity (handshake pulse is active-high, ~100 ms).
constexpr bool kPiDetectActiveHigh = true;

// Ignore DETECT glitches shorter than this (Pi default pulse is 100 ms).
constexpr uint32_t kDetectMinPulseMs = 50;

}  // namespace MissionConfig
