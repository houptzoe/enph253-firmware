#include "arm/arm.h"

#include "display/display.h"
#include "sonar/sonar.h"

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void PickupArm::begin(const PickupArmConfig& config, UltrasonicSonar* sonar,
                      ReflectanceDisplay* display) {
  config_ = config;
  sonar_ = sonar;
  display_ = display;

  pinMode(config_.rotationDirPin, OUTPUT);
  pinMode(config_.rotationStepPin, OUTPUT);
  pinMode(config_.verticalDirPin, OUTPUT);
  pinMode(config_.verticalStepPin, OUTPUT);
  pinMode(config_.horizontalDirPin, OUTPUT);
  pinMode(config_.horizontalStepPin, OUTPUT);
  pinMode(config_.switch0Pin, INPUT_PULLUP);

  digitalWrite(config_.rotationStepPin, LOW);
  digitalWrite(config_.rotationDirPin, LOW);
  digitalWrite(config_.verticalStepPin, LOW);
  digitalWrite(config_.verticalDirPin, LOW);
  digitalWrite(config_.horizontalDirPin, LOW);
  digitalWrite(config_.horizontalStepPin, LOW);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  gripper_.setPeriodHertz(50);
  gripper_.attach(config_.servoPin, 500, 2400);
  setGripperOpen();

  phase_ = PickupPhase::Idle;
}

// ---------------------------------------------------------------------------
// Low-level stepper helpers
// ---------------------------------------------------------------------------

void PickupArm::stepAxis(int stepPin, int dirPin, bool dir, int steps,
                          int pulseUs) {
  digitalWrite(dirPin, dir ? HIGH : LOW);
  delayMicroseconds(10);

  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseUs);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseUs);
  }
}

bool PickupArm::switchPressed() const {
  return digitalRead(config_.switch0Pin) == LOW;
}

int PickupArm::extendUntilStop(int maxSteps) {
  digitalWrite(config_.horizontalDirPin, config_.extendDirOut ? HIGH : LOW);
  delayMicroseconds(10);

  int taken = 0;
  for (; taken < maxSteps; taken++) {
    if (switchPressed()) {
      break;
    }
    digitalWrite(config_.horizontalStepPin, HIGH);
    delayMicroseconds(config_.horizontalStepDelayUs);
    digitalWrite(config_.horizontalStepPin, LOW);
    delayMicroseconds(config_.horizontalStepDelayUs);
  }
  return taken;
}

int PickupArm::rotateUntilSonar(bool dir, int maxSteps) {
  digitalWrite(config_.rotationDirPin, dir ? HIGH : LOW);
  delayMicroseconds(10);

  int taken = 0;
  for (; taken < maxSteps; taken++) {
    digitalWrite(config_.rotationStepPin, HIGH);
    delayMicroseconds(config_.rotationStepDelayUs);
    digitalWrite(config_.rotationStepPin, LOW);
    delayMicroseconds(config_.rotationStepDelayUs);

    if (sonar_ == nullptr) {
      continue;
    }
    if ((taken + 1) % config_.rotateStepsBetweenPings != 0) {
      continue;
    }

    float cm = 0.0f;
    bool valid = false;
    sonar_->ping(cm, valid);
    if (display_ != nullptr) {
      display_->showSonarScan(cm, valid, config_.sonarDetectMinCm,
                              config_.sonarDetectMaxCm);
    }
    if (valid && cm >= config_.sonarDetectMinCm &&
        cm <= config_.sonarDetectMaxCm) {
      Serial.printf("[ARM] sonar lock at %.1f cm after %d rotate steps\n", cm,
                    taken + 1);
      return taken + 1;
    }
  }

  if (taken >= maxSteps) {
    Serial.printf("[ARM] rotate scan hit max steps (%d) without sonar lock\n",
                  maxSteps);
  }
  return taken;
}

int PickupArm::rotateForDurationMs(bool dir, uint32_t durationMs) {
  digitalWrite(config_.rotationDirPin, dir ? HIGH : LOW);
  delayMicroseconds(10);

  const uint32_t startMs = millis();
  int taken = 0;
  while (millis() - startMs < durationMs) {
    digitalWrite(config_.rotationStepPin, HIGH);
    delayMicroseconds(config_.rotationStepDelayUs);
    digitalWrite(config_.rotationStepPin, LOW);
    delayMicroseconds(config_.rotationStepDelayUs);
    taken++;
  }
  return taken;
}

void PickupArm::setGripperOpen() { gripper_.write(config_.servoOpenDeg); }
void PickupArm::setGripperClosed() { gripper_.write(config_.servoClosedDeg); }

// ---------------------------------------------------------------------------
// Sequence control
// ---------------------------------------------------------------------------

void PickupArm::startPickup(MetalSide side) {
  if (isBusy()) {
    return;
  }
  side_ = side;
  extendStepsTaken_ = 0;
  initialExtendSteps_ = 0;
  rotateStepsTaken_ = 0;
  stowRaiseSteps_ = 0;
  // Tune series ends with full stow + open gripper.
  tuneFixedExtendPending_ = config_.tuneMode;
  phase_ = config_.tuneMode ? PickupPhase::Extend : PickupPhase::SetupRaise;
}

bool PickupArm::update(PickupPhase& phaseOut) {
  switch (phase_) {
    case PickupPhase::Idle:
    case PickupPhase::Done:
      break;

    case PickupPhase::SetupRaise:
      setGripperOpen();
      stepAxis(config_.verticalStepPin, config_.verticalDirPin,
                config_.raiseDirUp, config_.setupRaiseSteps,
                config_.verticalStepDelayUs);
      phase_ = PickupPhase::SetupRotate;
      break;

    case PickupPhase::SetupRotate: {
      const bool dir = (side_ == MetalSide::Left) ? config_.rotateDirLeft
                                                    : config_.rotateDirRight;
      const int steps = config_.tuneMode
                            ? config_.tuneRotateSteps
                            : ((side_ == MetalSide::Left)
                                   ? config_.setupRotateStepsLeft
                                   : config_.setupRotateStepsRight);
      Serial.printf("[ARM] SetupRotate side=%s dir=%d\n",
                    (side_ == MetalSide::Left)    ? "L"
                    : (side_ == MetalSide::Right) ? "R"
                                                  : "?",
                    dir ? 1 : 0);
      stepAxis(config_.rotationStepPin, config_.rotationDirPin, dir, steps,
                config_.rotationStepDelayUs);
      rotateStepsTaken_ += steps;
      phase_ = config_.tuneMode ? PickupPhase::SetupLower
                                : PickupPhase::SetupExtend;
      break;
    }

    case PickupPhase::SetupExtend:
      stepAxis(config_.horizontalStepPin, config_.horizontalDirPin,
                config_.extendDirOut, config_.setupExtendSteps,
                config_.horizontalStepDelayUs);
      extendStepsTaken_ += config_.setupExtendSteps;
      initialExtendSteps_ = config_.setupExtendSteps;
      phase_ = PickupPhase::SetupLower;
      break;

    case PickupPhase::SetupLower: {
      const int steps =
          config_.tuneMode ? config_.tuneLowerSteps : config_.setupLowerSteps;
      const int pulseUs = config_.tuneMode ? config_.tuneLowerStepDelayUs
                                             : config_.verticalStepDelayUs;
      stepAxis(config_.verticalStepPin, config_.verticalDirPin,
                config_.lowerDirDown, steps, pulseUs);
      if (config_.tuneMode) {
        stowRaiseSteps_ = config_.tuneRaiseSteps;
        phase_ = PickupPhase::RotateScan;
        break;
      }
      // Net drop from start = setupLower - setupRaise; stow by raising that.
      stowRaiseSteps_ = config_.setupLowerSteps - config_.setupRaiseSteps;
      if (stowRaiseSteps_ < 0) {
        stowRaiseSteps_ = 0;
      }
      phase_ = PickupPhase::RotateScan;
      break;
    }

    case PickupPhase::RotateScan: {
      // Same yaw dir as SetupRotate for this side.
      const bool dir = (side_ == MetalSide::Left) ? config_.rotateDirLeft
                                                    : config_.rotateDirRight;
      Serial.printf("[ARM] RotateScan side=%s dir=%d\n",
                    (side_ == MetalSide::Left)    ? "L"
                    : (side_ == MetalSide::Right) ? "R"
                                                  : "?",
                    dir ? 1 : 0);
      rotateStepsTaken_ +=
          rotateUntilSonar(dir, config_.rotateScanMaxSteps);
      phase_ = PickupPhase::PostScanYawAdjust;
      break;
    }

    case PickupPhase::PostScanYawAdjust: {
      // Metal vs sonar offset: continue in the same side yaw for both L and R.
      const bool dir = (side_ == MetalSide::Left) ? config_.rotateDirLeft
                                                    : config_.rotateDirRight;
      const int steps =
          rotateForDurationMs(dir, config_.postScanYawAdjustMs);
      rotateStepsTaken_ += steps;
      Serial.printf("[ARM] post-scan yaw continue %d steps (%lu ms)\n", steps,
                    static_cast<unsigned long>(config_.postScanYawAdjustMs));
      phase_ = PickupPhase::Extend;
      break;
    }

    case PickupPhase::Extend:
      if (config_.tuneMode && tuneFixedExtendPending_) {
        // Initial fixed-step extend for tuning — then rotate/lower/scan.
        tuneFixedExtendPending_ = false;
        stepAxis(config_.horizontalStepPin, config_.horizontalDirPin,
                  config_.extendDirOut, config_.tuneExtendSteps,
                  config_.horizontalStepDelayUs);
        extendStepsTaken_ += config_.tuneExtendSteps;
        initialExtendSteps_ = config_.tuneExtendSteps;
        phase_ = PickupPhase::SetupRotate;
        break;
      }
      // Remaining budget from home; stop early on microswitch.
      {
        const int remaining = config_.maxExtendSteps - extendStepsTaken_;
        if (remaining > 0) {
          extendStepsTaken_ += extendUntilStop(remaining);
        }
        Serial.printf("[ARM] extend done total=%d / max=%d\n", extendStepsTaken_,
                      config_.maxExtendSteps);
      }
      phase_ = PickupPhase::Grip;
      break;

    case PickupPhase::Grip:
      // Close after extend stopped (switch or max from home).
      setGripperClosed();
      delay(500);  // let the servo finish closing before moving the arm
      phase_ = PickupPhase::Retract;
      break;

    case PickupPhase::Retract: {
      // Retract grab reach (everything beyond the initial/home extend).
      // Full sequence has no separate initial retract — undo all at once.
      const int grabSteps = config_.tuneMode
                                ? (extendStepsTaken_ - initialExtendSteps_)
                                : extendStepsTaken_;
      if (grabSteps > 0) {
        stepAxis(config_.horizontalStepPin, config_.horizontalDirPin,
                  config_.retractDirIn, grabSteps,
                  config_.horizontalStepDelayUs);
      }
      if (!config_.tuneMode) {
        extendStepsTaken_ = 0;
        initialExtendSteps_ = 0;
      }
      phase_ = PickupPhase::Raise;
      break;
    }

    case PickupPhase::Raise: {
      const int pulseUs = config_.tuneMode ? config_.tuneLowerStepDelayUs
                                             : config_.verticalStepDelayUs;
      stepAxis(config_.verticalStepPin, config_.verticalDirPin,
                config_.raiseDirUp, stowRaiseSteps_, pulseUs);
      phase_ = PickupPhase::Recenter;
      break;
    }

    case PickupPhase::Recenter: {
      // Undo yaw after raise. Tune: Left 1/4 of total, Right 5/8; full: all.
      const bool dir = (side_ == MetalSide::Left) ? config_.rotateDirRight
                                                    : config_.rotateDirLeft;
      int steps = rotateStepsTaken_;
      if (config_.tuneMode) {
        steps = (side_ == MetalSide::Left) ? (rotateStepsTaken_ / 4)
                                           : ((rotateStepsTaken_ * 5) / 8);
      }
      if (steps > 0) {
        stepAxis(config_.rotationStepPin, config_.rotationDirPin, dir, steps,
                  config_.rotationStepDelayUs);
      }
      Serial.printf("[ARM] recenter %d / %d yaw steps\n", steps,
                    rotateStepsTaken_);
      rotateStepsTaken_ = 0;
      phase_ = config_.tuneMode ? PickupPhase::RetractInitial
                                : PickupPhase::OpenGrip;
      break;
    }

    case PickupPhase::RetractInitial:
      // Undo the first extend after yaw is home again.
      if (initialExtendSteps_ > 0) {
        stepAxis(config_.horizontalStepPin, config_.horizontalDirPin,
                  config_.retractDirIn, initialExtendSteps_,
                  config_.horizontalStepDelayUs);
      }
      extendStepsTaken_ = 0;
      initialExtendSteps_ = 0;
      phase_ = PickupPhase::OpenGrip;
      break;

    case PickupPhase::OpenGrip:
      delay(1000);  // settle before releasing
      setGripperOpen();
      delay(300);  // let the servo finish opening
      phase_ = PickupPhase::Done;
      break;
  }

  phaseOut = phase_;
  return phase_ == PickupPhase::Done;
}
