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
// After a metal hit (full sequence when tuneMode is false):
//   1) Setup from stow: raise a little, rotate toward the hit side, extend a
//      little, then lower to rock level.
//   2) RotateScan until HC-SR04 sees an object, then extend/grab/stow.
//
// When tuneMode is true, runs Extend → Rotate → Lower → RotateScan →
// yaw-adjust → extend-until-switch → grip → retract grab → raise →
// recenter → retract initial extend → open grip, then Done.
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

  // One-move-at-a-time tuning. When true, runs Extend → Rotate → Lower →
  // RotateScan → extend-until-switch, then Done (full sequence skipped).
  bool tuneMode = false;
  int tuneExtendSteps = 200;
  int tuneRotateSteps = 280;  // toward the metal-hit side
  int tuneLowerSteps = 5000;
  int tuneRaiseSteps = 5500;  // stow raise after grab (may exceed lower)
  int tuneLowerStepDelayUs = 1000;  // lower axis pulse delay in tuneMode

  // Setup-from-stow step counts — tune on hardware.
  int setupRaiseSteps = 300;         // small lift off the starting pose
  int setupRotateStepsLeft = 400;    // initial yaw toward left hit
  int setupRotateStepsRight = 400;   // initial yaw toward right hit
  int setupExtendSteps = 80;         // short reach before dropping to rock
  int setupLowerSteps = 2300;        // from raised pose down to rock level

  // Pickup step counts — tune on hardware.
  // maxExtendSteps is total horizontal travel from home (all extends combined).
  int maxExtendSteps = 300;
  int rotateScanMaxSteps = 1200;       // safety stop if sonar never trips
  int rotateStepsBetweenPings = 8;     // ping sonar every N steps while scanning

  // Stop RotateScan when a valid echo is within this window (cm).
  // After lock, a short yaw bias corrects metal-detector vs sonar offset,
  // then Extend uses the microswitch only — sonar is ignored.
  float sonarDetectMaxCm = 20.0f;
  float sonarDetectMinCm = 4.0f;
  // After sonar lock, continue the same side yaw this long (L and R).
  uint32_t postScanYawAdjustMs = 500;

  // Half-period delay between step edges (us). Higher = slower.
  int rotationStepDelayUs = 1000;
  int verticalStepDelayUs = 1000;
  int horizontalStepDelayUs = 2000;

  int servoOpenDeg = 110;
  int servoClosedDeg = 180;

  // Direction polarity per axis (HIGH/LOW on the DIR pin).
  // SetupRotate, RotateScan, and post-scan continue all share these per side.
  bool rotateDirLeft = false;
  bool rotateDirRight = true;
  bool lowerDirDown = true;
  bool raiseDirUp = false;
  bool extendDirOut = true;
  bool retractDirIn = false;
};

enum class PickupPhase {
  Idle,         // not running
  SetupRaise,   // small lift from starting pose
  SetupRotate,  // yaw toward the detected metal side
  SetupExtend,  // short horizontal reach
  SetupLower,   // drop to rock level
  RotateScan,   // rotate toward side until sonar sees an object
  PostScanYawAdjust,  // continue same side yaw briefly after sonar lock
  Extend,       // reach out until a limit switch trips
  Grip,         // close the gripper
  Retract,      // pull the horizontal axis back in
  Raise,        // bring the vertical axis back up to stow height
  Recenter,     // undo setup+scan rotation back to starting yaw
  RetractInitial, // tuneMode: undo the first/home extend after recenter
  OpenGrip,     // open the gripper once recentered
  Done,         // sequence finished — arm stowed, gripper open
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
  bool isTuneMode() const { return config_.tuneMode; }

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
  int extendStepsTaken_ = 0;   // total horizontal out from home
  int initialExtendSteps_ = 0; // first extend (undone after recenter in tune)
  int rotateStepsTaken_ = 0;   // setup rotate + scan rotate
  int stowRaiseSteps_ = 0;     // vertical steps to return to start height
  bool tuneFixedExtendPending_ = false;  // first tune Extend uses fixed steps
};
