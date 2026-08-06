#pragma once

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

#include "hardware/pins.h"
#include "metal/metal.h"  // MetalSide — which way to reach for the hit

class UltrasonicSonar;  // sonar-guided rotate at rock level
class ReflectanceDisplay;

// Pickup arm — three stepper axes (rotate, vertical, horizontal-extend) plus
// a servo gripper.
//
// After a metal hit (left or right):
//   Extend → Rotate → Lower → RotateScan → post-scan yaw → extend-to-switch →
//   grip → retract grab → raise → recenter → retract initial → open grip.
//
// Each call to update() advances exactly one phase. Phases themselves are
// short blocking step trains, so call update() repeatedly from the main loop.

// Tunable parameters passed to PickupArm::begin().
struct PickupArmConfig {
  int rotationDirPin = kRotationDirPin;
  int rotationStepPin = kRotationStepPin;
  int verticalDirPin = kVerticalDirPin;
  int verticalStepPin = kVerticalStepPin;
  int horizontalDirPin = kHorizontalDirPin;
  int horizontalStepPin = kHorizontalStepPin;
  int switch0Pin = kSwitch0Pin;
  int servoPin = kServoMotorPin;

  // Primary pickup step counts (tuned on hardware).
  int initialExtendSteps = 200;   // first reach before rotate/lower
  int rotateSteps = 280;          // yaw toward the metal-hit side
  int lowerSteps = 5000;          // drop to rock level
  int raiseSteps = 5500;          // stow raise after grab (may exceed lower)
  // maxExtendSteps is total horizontal travel from home (all extends combined).
  int maxExtendSteps = 300;
  int rotateScanMaxSteps = 1200;  // safety stop if sonar never trips
  int rotateStepsBetweenPings = 8;

  // Stop RotateScan when a valid echo is within this window (cm).
  // After lock, a short yaw continue corrects metal vs sonar offset,
  // then Extend uses the microswitch only — sonar is ignored.
  float sonarDetectMaxCm = 20.0f;
  float sonarDetectMinCm = 4.0f;
  uint32_t postScanYawAdjustMs = 500;

  // Recenter fractions of total yaw after raise (per side).
  // Left: 1/4, Right: 5/8 of rotateStepsTaken_.
  int recenterLeftNum = 1;
  int recenterLeftDen = 4;
  int recenterRightNum = 5;
  int recenterRightDen = 8;

  // Half-period delay between step edges (us). Higher = slower.
  int rotationStepDelayUs = 1000;
  int verticalStepDelayUs = 1000;
  int horizontalStepDelayUs = 2000;

  int servoOpenDeg = 110;
  int servoClosedDeg = 180;

  // Direction polarity per axis (HIGH/LOW on the DIR pin).
  // Rotate, RotateScan, and post-scan continue all share these per side.
  bool rotateDirLeft = false;
  bool rotateDirRight = true;
  bool lowerDirDown = true;
  bool raiseDirUp = false;
  bool extendDirOut = true;
  bool retractDirIn = false;
};

enum class PickupPhase {
  Idle,              // not running
  InitialExtend,     // fixed first reach from home
  Rotate,            // yaw toward the detected metal side
  Lower,             // drop to rock level
  RotateScan,        // rotate toward side until sonar sees an object
  PostScanYawAdjust, // continue same side yaw briefly after sonar lock
  Extend,            // reach out until limit switch or max from home
  Grip,              // close the gripper
  Retract,           // pull back grab reach only
  Raise,             // bring the vertical axis back up
  Recenter,          // undo part of yaw toward starting heading
  RetractInitial,    // undo the first/home extend after recenter
  OpenGrip,          // open the gripper once stowed
  Done,              // sequence finished — arm stowed, gripper open
};

class PickupArm {
 public:
  void begin(const PickupArmConfig& config, UltrasonicSonar* sonar = nullptr,
             ReflectanceDisplay* display = nullptr);

  // Kick off a new pickup sequence toward `side`. Ignored if already busy.
  void startPickup(MetalSide side);

  // Advances one phase per call. Returns true once phase() == Done.
  bool update(PickupPhase& phaseOut);

  bool isBusy() const {
    return phase_ != PickupPhase::Idle && phase_ != PickupPhase::Done;
  }
  PickupPhase phase() const { return phase_; }

 private:
  void stepAxis(int stepPin, int dirPin, bool dir, int steps, int pulseUs);
  int extendUntilStop(int maxSteps);
  int rotateUntilSonar(bool dir, int maxSteps);
  int rotateForDurationMs(bool dir, uint32_t durationMs);
  bool switchPressed() const;
  void setGripperOpen();
  void setGripperClosed();

  PickupArmConfig config_{};
  UltrasonicSonar* sonar_ = nullptr;
  ReflectanceDisplay* display_ = nullptr;
  Servo gripper_;
  PickupPhase phase_ = PickupPhase::Idle;
  MetalSide side_ = MetalSide::None;
  int extendStepsTaken_ = 0;    // total horizontal out from home
  int initialExtendSteps_ = 0;  // first extend (undone after recenter)
  int rotateStepsTaken_ = 0;    // rotate + scan + post-scan yaw
  int stowRaiseSteps_ = 0;      // vertical steps to return toward start height
};
