#pragma once

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

#include "hardware/pins.h"
#include "metal/metal.h"  // MetalSide — which way to reach for the hit

// Pickup arm — three stepper axes (rotate, vertical, horizontal-extend) plus
// a servo gripper. Runs a fixed mechanical sequence to reach toward whichever
// side the metal detector fired on, grab, and stow back out of the way.
//
// Each call to update() advances exactly one phase. Phases themselves are
// short blocking step trains (same as a bare stepper driver loop), so call
// update() repeatedly from the main loop while phase() != Done — the whole
// sequence takes roughly a second or two of wall-clock time.

// Tunable parameters passed to PickupArm::begin().
struct PickupArmConfig {
  int rotationDirPin = kRotationDirPin;
  int rotationStepPin = kRotationStepPin;
  int verticalDirPin = kVerticalDirPin;
  int verticalStepPin = kVerticalStepPin;
  int horizontalDirPin = kHorizontalDirPin;
  int horizontalStepPin = kHorizontalStepPin;
  int switch0Pin = kSwitch0Pin;
  int switch1Pin = kSwitch1Pin;
  int servoPin = kServoMotorPin;

  // Step counts — tune on hardware.
  int rotateStepsLeft = 600;
  int rotateStepsRight = 600;
  int lowerSteps = 2000;
  int maxExtendSteps = 300;

  // Half-period delay between step edges (us). Higher = slower.
  int rotationStepDelayUs = 1000;
  int verticalStepDelayUs = 1000;
  int horizontalStepDelayUs = 2000;

  int servoOpenDeg = 80;
  int servoClosedDeg = 130;

  // Direction polarity per axis (HIGH/LOW on the DIR pin).
  bool rotateDirLeft = false;
  bool rotateDirRight = true;
  bool lowerDirDown = true;
  bool raiseDirUp = false;
  bool extendDirOut = true;
  bool retractDirIn = false;
};

enum class PickupPhase {
  Idle,      // not running
  Rotate,    // turn toward the detected side
  Lower,     // drop the arm, gripper open
  Extend,    // reach out until a limit switch trips
  Grip,      // close the gripper
  Retract,   // pull the horizontal axis back in
  Raise,     // bring the vertical axis back up
  Recenter,  // undo the rotation so driving can resume straight
  Done,      // sequence finished — object gripped and arm stowed
};

class PickupArm {
 public:
  void begin(const PickupArmConfig& config);

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
  bool switchPressed() const;
  void setGripperOpen();
  void setGripperClosed();

  PickupArmConfig config_{};
  Servo gripper_;
  PickupPhase phase_ = PickupPhase::Idle;
  MetalSide side_ = MetalSide::None;
  int extendStepsTaken_ = 0;
};
