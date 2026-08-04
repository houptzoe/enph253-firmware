#pragma once

#include <Arduino.h>

#include "hardware/pins.h"

// Tape-following controller: samples reflectance sensors, computes steering correction.

// Tunable parameters passed to TapeFollowPid::begin().
struct TapeFollowConfig {
  int leftReflectancePin = kLeftReflectancePin;
  int rightReflectancePin = kRightReflectancePin;
  int reflectanceThreshold = 650;    // hi-lo cutoff between off-tape and on-tape
  // 2 kHz sampling: two analogReads take ~100 us, so a faster period saturates
  // the esp_timer task on CPU 0 and trips the task watchdog (reboot loop).
  uint32_t samplePeriodUs = 500;
  uint16_t samplesPerUpdate = 10;    // average samples → 200 Hz control loop
  float kp = 45.0f;
  float ki = 0.0f;
  float kd = 10.0f;
  float integralMax = 10.0f;         // anti-windup clamp
};

// Latest sensor readings and PID output, produced each control tick.
struct TapeFollowState {
  int leftAvg = 0;
  int rightAvg = 0;
  bool leftOnTape = false;
  bool rightOnTape = false;
  float error = 0.0f;       // -1, 0, or +1 from thresholded sensor pair
  float correction = 0.0f;  // PID output applied to differential drive
};

class TapeFollowPid {
 public:
  void begin(const TapeFollowConfig& config);
  bool update(TapeFollowState& state);  // returns true when a new tick is ready
  void reset();
  float controlPeriodSec() const;

  // Live tuning — updates gains and clears integral windup.
  void setGains(float kp, float ki, float kd, float integralMax);
  TapeFollowConfig getConfig() const { return config_; }

 private:
  // Standard PID with integral clamping.
  class PidController {
   public:
    PidController(float kp, float ki, float kd, float integralMax);

    float update(float error, float dtSec);
    void reset();
    void setGains(float kp, float ki, float kd, float integralMax);

   private:
    float kp_, ki_, kd_, integralMax_;
    float integral_ = 0.0f;
    float lastError_ = 0.0f;
  };

  // esp_timer callback — accumulates ADC reads at 10 kHz.
  static void sampleTimerCallback(void* arg);

  bool onTape(int reading) const;
  float digitalLineError(bool leftOn, bool rightOn);

  TapeFollowConfig config_{};
  PidController pid_{0.0f, 0.0f, 0.0f, 0.0f};
  float lastKnownError_ = 0.0f;  // held when both sensors lose the tape

  // Shared between timer callback and main loop.
  portMUX_TYPE sensorMux_ = portMUX_INITIALIZER_UNLOCKED;
  volatile bool updateReady_ = false;
  TapeFollowState snapshot_{};

  volatile uint32_t leftSum_ = 0;
  volatile uint32_t rightSum_ = 0;
  volatile uint16_t sampleCount_ = 0;
};
