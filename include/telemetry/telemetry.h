#pragma once

#include "motor/motor.h"
#include "pid/pid.h"

// SoftAP web UI for live PID tuning, drive enable, and motor PWM telemetry.

struct TelemetrySnapshot {
  float error = 0.0f;
  float correction = 0.0f;
};

struct DriveSettings {
  bool running = true;  // armed at boot; SoftAP Stop still disables
  float leftBaseSpeed = 90.0f;
  float rightBaseSpeed = 90.0f;
  float maxSpeed = 150.0f;
};

// Pending web request for the Pi vision search, consumed by the main loop.
enum class VisionCommand : uint8_t { None, Start, Stop };

class TelemetryServer {
 public:
  void begin(TapeFollowPid& pid, MotorDriver& motors);
  void updateSnapshot(const TelemetrySnapshot& snapshot);
  void updateMissionStatus(const char* phaseName, int8_t detectedCamera,
                           uint8_t detectCount = 0);
  void poll();

  const DriveSettings& drive() const { return drive_; }
  void setDriveRunning(bool running);

  VisionCommand takeVisionCommand();

 private:
  void handleRoot();
  void handleStatus();
  void handlePid();
  void handleDrive();
  void handleVision();

  static float parseJsonFloat(const String& body, const char* key,
                              float fallback);
  static bool parseJsonBool(const String& body, const char* key, bool fallback);

  TapeFollowPid* pid_ = nullptr;
  MotorDriver* motors_ = nullptr;
  TelemetrySnapshot snapshot_{};
  DriveSettings drive_{};
  VisionCommand visionCommand_ = VisionCommand::None;
  const char* missionPhase_ = "Idle";
  int8_t detectedCamera_ = -1;
  uint8_t detectCount_ = 0;
};
