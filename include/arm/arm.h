#pragma once

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

#include "hardware/pins.h"
#include "metal/metal.h"  // MetalSide — which way to reach for the hit

class UltrasonicSonar;  // sonar-guided rotate at rock level
class ReflectanceDisplay;
class ImuTracker;       // claw-mounted IMU for home-yaw recenter

// Pickup arm — three stepper axes (rotate, vertical, horizontal-extend) plus
// a servo gripper.
//
// Metal hit: Extend → Rotate → Lower → RotateScan → … → IMU recenter → open.
// After second course ~180°: halt chassis, then Extend → 90° left yaw →
// Lower → Extend to max, then stay deployed.

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

  // Recenter: rotate opposite the hit side until claw IMU yaw ≈ homeYaw
  // recorded at pickup start. Safety stop if tolerance never reached.
  float homeYawToleranceDeg = 4.0f;
  int recenterMaxSteps = 2500;

  // After second course 180°: ~90° yaw left (blocking; chassis is halted).
  int postCourseLeftYawSteps = 350;

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
  Recenter,          // yaw until claw IMU matches home recorded at pickup start
  RetractInitial,    // undo the first/home extend after recenter
  OpenGrip,          // open the gripper once stowed
  Done,              // sequence finished — arm stowed, gripper open
};

class PickupArm {
 public:
  void begin(const PickupArmConfig& config, UltrasonicSonar* sonar = nullptr,
             ReflectanceDisplay* display = nullptr, ImuTracker* imu = nullptr);

  // Kick off pickup toward `side`. Pass claw IMU yaw at start as home for
  // recenter when homeYawValid is true; otherwise undo rotateStepsTaken_.
  void startPickup(MetalSide side, float homeYawDeg = 0.0f,
                   bool homeYawValid = false);

  // After second ~180°: initial extend → ~90° left → lower → full extend.
  void startPostCourseDeploy();

  // Advances one phase per call. Returns true once phase() == Done.
  bool update(PickupPhase& phaseOut);

  bool isBusy() const {
    return phase_ != PickupPhase::Idle && phase_ != PickupPhase::Done;
  }
  PickupPhase phase() const { return phase_; }
  bool isPostCourseDeploy() const { return postCourseDeploy_; }

 private:
  void stepAxis(int stepPin, int dirPin, bool dir, int steps, int pulseUs);
  int extendUntilStop(int maxSteps);
  int rotateUntilSonar(bool dir, int maxSteps);
  int rotateForDurationMs(bool dir, uint32_t durationMs);
  // Closed-loop yaw to homeYawDeg; direction from signed angle error only.
  int rotateUntilHomeYaw(float homeYawDeg, int maxSteps);
  bool switchPressed() const;
  void setGripperOpen();
  void setGripperClosed();

  PickupArmConfig config_{};
  UltrasonicSonar* sonar_ = nullptr;
  ReflectanceDisplay* display_ = nullptr;
  ImuTracker* imu_ = nullptr;
  Servo gripper_;
  PickupPhase phase_ = PickupPhase::Idle;
  MetalSide side_ = MetalSide::None;
  bool postCourseDeploy_ = false;
  int extendStepsTaken_ = 0;    // total horizontal out from home
  int initialExtendSteps_ = 0;  // first extend (undone after recenter)
  int rotateStepsTaken_ = 0;    // rotate + scan + post-scan yaw
  int stowRaiseSteps_ = 0;      // vertical steps to return toward start height
  float homeYawDeg_ = 0.0f;     // claw IMU yaw at pickup start
  bool homeYawValid_ = false;
};
