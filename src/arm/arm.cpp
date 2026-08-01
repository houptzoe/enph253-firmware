#include "arm/arm.h"

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void PickupArm::begin(const PickupArmConfig& config) {
  config_ = config;

  pinMode(config_.rotationDirPin, OUTPUT);
  pinMode(config_.rotationStepPin, OUTPUT);
  pinMode(config_.verticalDirPin, OUTPUT);
  pinMode(config_.verticalStepPin, OUTPUT);
  pinMode(config_.horizontalDirPin, OUTPUT);
  pinMode(config_.horizontalStepPin, OUTPUT);
  pinMode(config_.switch0Pin, INPUT_PULLUP);
  pinMode(config_.switch1Pin, INPUT_PULLUP);

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
  return digitalRead(config_.switch0Pin) == LOW ||
         digitalRead(config_.switch1Pin) == LOW;
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
  phase_ = PickupPhase::Rotate;
}

bool PickupArm::update(PickupPhase& phaseOut) {
  switch (phase_) {
    case PickupPhase::Idle:
    case PickupPhase::Done:
      break;

    case PickupPhase::Rotate: {
      const bool dir = (side_ == MetalSide::Left) ? config_.rotateDirLeft
                                                    : config_.rotateDirRight;
      const int steps = (side_ == MetalSide::Left) ? config_.rotateStepsLeft
                                                     : config_.rotateStepsRight;
      stepAxis(config_.rotationStepPin, config_.rotationDirPin, dir, steps,
                config_.rotationStepDelayUs);
      phase_ = PickupPhase::Lower;
      break;
    }

    case PickupPhase::Lower:
      setGripperOpen();
      stepAxis(config_.verticalStepPin, config_.verticalDirPin,
                config_.lowerDirDown, config_.lowerSteps,
                config_.verticalStepDelayUs);
      phase_ = PickupPhase::Extend;
      break;

    case PickupPhase::Extend:
      extendStepsTaken_ = extendUntilStop(config_.maxExtendSteps);
      phase_ = PickupPhase::Grip;
      break;

    case PickupPhase::Grip:
      setGripperClosed();
      delay(500);  // let the servo finish closing before moving the arm
      phase_ = PickupPhase::Retract;
      break;

    case PickupPhase::Retract:
      // Back off horizontally by exactly what we extended, so we don't
      // depend on a second limit switch at the retracted end.
      stepAxis(config_.horizontalStepPin, config_.horizontalDirPin,
                config_.retractDirIn, extendStepsTaken_,
                config_.horizontalStepDelayUs);
      phase_ = PickupPhase::Raise;
      break;

    case PickupPhase::Raise:
      stepAxis(config_.verticalStepPin, config_.verticalDirPin,
                config_.raiseDirUp, config_.lowerSteps,
                config_.verticalStepDelayUs);
      phase_ = PickupPhase::Recenter;
      break;

    case PickupPhase::Recenter: {
      // Undo the rotation so the chassis is square with the tape again.
      const bool dir = (side_ == MetalSide::Left) ? config_.rotateDirRight
                                                    : config_.rotateDirLeft;
      const int steps = (side_ == MetalSide::Left) ? config_.rotateStepsLeft
                                                     : config_.rotateStepsRight;
      stepAxis(config_.rotationStepPin, config_.rotationDirPin, dir, steps,
                config_.rotationStepDelayUs);
      phase_ = PickupPhase::Done;
      break;
    }
  }

  phaseOut = phase_;
  return phase_ == PickupPhase::Done;
}
