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
  phase_ = PickupPhase::InitialExtend;
}

bool PickupArm::update(PickupPhase& phaseOut) {
  switch (phase_) {
    case PickupPhase::Idle:
    case PickupPhase::Done:
      break;

    case PickupPhase::InitialExtend:
      setGripperOpen();
      stepAxis(config_.horizontalStepPin, config_.horizontalDirPin,
                config_.extendDirOut, config_.initialExtendSteps,
                config_.horizontalStepDelayUs);
      extendStepsTaken_ += config_.initialExtendSteps;
      initialExtendSteps_ = config_.initialExtendSteps;
      phase_ = PickupPhase::Rotate;
      break;

    case PickupPhase::Rotate: {
      const bool dir = (side_ == MetalSide::Left) ? config_.rotateDirLeft
                                                    : config_.rotateDirRight;
      Serial.printf("[ARM] Rotate side=%s dir=%d\n",
                    (side_ == MetalSide::Left)    ? "L"
                    : (side_ == MetalSide::Right) ? "R"
                                                  : "?",
                    dir ? 1 : 0);
      stepAxis(config_.rotationStepPin, config_.rotationDirPin, dir,
                config_.rotateSteps, config_.rotationStepDelayUs);
      rotateStepsTaken_ += config_.rotateSteps;
      phase_ = PickupPhase::Lower;
      break;
    }

    case PickupPhase::Lower:
      stepAxis(config_.verticalStepPin, config_.verticalDirPin,
                config_.lowerDirDown, config_.lowerSteps,
                config_.verticalStepDelayUs);
      stowRaiseSteps_ = config_.raiseSteps;
      phase_ = PickupPhase::RotateScan;
      break;

    case PickupPhase::RotateScan: {
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

    case PickupPhase::Extend: {
      const int remaining = config_.maxExtendSteps - extendStepsTaken_;
      if (remaining > 0) {
        extendStepsTaken_ += extendUntilStop(remaining);
      }
      Serial.printf("[ARM] extend done total=%d / max=%d\n", extendStepsTaken_,
                    config_.maxExtendSteps);
      phase_ = PickupPhase::Grip;
      break;
    }

    case PickupPhase::Grip:
      setGripperClosed();
      delay(500);
      phase_ = PickupPhase::Retract;
      break;

    case PickupPhase::Retract: {
      const int grabSteps = extendStepsTaken_ - initialExtendSteps_;
      if (grabSteps > 0) {
        stepAxis(config_.horizontalStepPin, config_.horizontalDirPin,
                  config_.retractDirIn, grabSteps,
                  config_.horizontalStepDelayUs);
      }
      phase_ = PickupPhase::Raise;
      break;
    }

    case PickupPhase::Raise:
      stepAxis(config_.verticalStepPin, config_.verticalDirPin,
                config_.raiseDirUp, stowRaiseSteps_,
                config_.verticalStepDelayUs);
      phase_ = PickupPhase::Recenter;
      break;

    case PickupPhase::Recenter: {
      const bool dir = (side_ == MetalSide::Left) ? config_.rotateDirRight
                                                    : config_.rotateDirLeft;
      const int steps =
          (side_ == MetalSide::Left)
              ? (rotateStepsTaken_ * config_.recenterLeftNum) /
                    config_.recenterLeftDen
              : (rotateStepsTaken_ * config_.recenterRightNum) /
                    config_.recenterRightDen;
      if (steps > 0) {
        stepAxis(config_.rotationStepPin, config_.rotationDirPin, dir, steps,
                  config_.rotationStepDelayUs);
      }
      Serial.printf("[ARM] recenter %d / %d yaw steps\n", steps,
                    rotateStepsTaken_);
      rotateStepsTaken_ = 0;
      phase_ = PickupPhase::RetractInitial;
      break;
    }

    case PickupPhase::RetractInitial:
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
      delay(1000);
      setGripperOpen();
      delay(300);
      phase_ = PickupPhase::Done;
      break;
  }

  phaseOut = phase_;
  return phase_ == PickupPhase::Done;
}
