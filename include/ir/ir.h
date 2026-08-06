#pragma once

#include <Arduino.h>

#include "hardware/pins.h"

// IR detector — pulse frequency on the sense pin, band select on a switch pin.
// Switch HIGH → low band (0–4 kHz expected). LOW → high band (6–12 kHz).

enum class IrBand { Low, High };

struct IrConfig {
  int sensePin = kIrDetectorPin;   // frequency input (PCNT)
  int switchPin = kIrSwitchPin;    // band select
  uint32_t gateTimeMs = 100;       // measurement window

  // Beacon present if measured Hz falls in the active band window.
  float lowBandMinHz = 200.0f;     // ignore noise floor in low band
  float lowBandMaxHz = 4000.0f;
  float highBandMinHz = 6000.0f;
  float highBandMaxHz = 12000.0f;
};

struct IrState {
  float hz = 0.0f;
  IrBand band = IrBand::Low;
  bool switchHigh = true;  // raw switch level (HIGH = low band)
  bool beaconDetected = false;
};

class IrSensor {
 public:
  void begin(const IrConfig& config);

  // Rate-limited gate. Returns true when a new reading is ready.
  bool update(IrState& state);

  void setEnabled(bool enabled) { enabled_ = enabled; }
  bool isEnabled() const { return enabled_; }

 private:
  bool setupPcnt(int pin);
  uint32_t readAndClearPcnt();

  IrConfig config_{};
  IrState state_{};
  bool enabled_ = false;
  bool pcntReady_ = false;
  uint32_t gateStartUs_ = 0;
};
