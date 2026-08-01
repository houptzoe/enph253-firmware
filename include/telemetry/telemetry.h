#pragma once

#include "motor/motor.h"
#include "pid/pid.h"

// SoftAP web UI for live PID tuning, drive enable, and motor PWM telemetry.

struct TelemetrySnapshot {
  float error = 0.0f;
  float correction = 0.0f;
};

struct DriveSettings {
  bool running = true;  // motors off until Start is pressed
  float leftBaseSpeed = 80.0f;
  float rightBaseSpeed = 80.0f;
  float maxSpeed = 150.0f;
};

class TelemetryServer {
 public:
  void begin(TapeFollowPid& pid, MotorDriver& motors);
  void updateSnapshot(const TelemetrySnapshot& snapshot);
  void poll();

  const DriveSettings& drive() const { return drive_; }

 private:
  void handleRoot();
  void handleStatus();
  void handlePid();
  void handleDrive();

  static float parseJsonFloat(const String& body, const char* key,
                              float fallback);
  static bool parseJsonBool(const String& body, const char* key, bool fallback);

  TapeFollowPid* pid_ = nullptr;
  MotorDriver* motors_ = nullptr;
  TelemetrySnapshot snapshot_{};
  DriveSettings drive_{};
};
