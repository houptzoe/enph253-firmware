#include "stepper/stepper.h"

void StepperMotor::begin(int dirPin, int stepPin) {
  dirPin_ = dirPin;
  stepPin_ = stepPin;
  pinMode(dirPin_, OUTPUT);
  pinMode(stepPin_, OUTPUT);
  digitalWrite(dirPin_, LOW);
  digitalWrite(stepPin_, LOW);
}

void StepperMotor::step(int steps, bool dirHigh) {
  if (dirPin_ < 0 || stepPin_ < 0 || steps <= 0) {
    return;
  }

  digitalWrite(dirPin_, dirHigh ? HIGH : LOW);
  // Brief settle after DIR change before pulsing STEP.
  delayMicroseconds(5);

  for (int i = 0; i < steps; ++i) {
    digitalWrite(stepPin_, HIGH);
    delayMicroseconds(kPulseHalfPeriodUs);
    digitalWrite(stepPin_, LOW);
    delayMicroseconds(kPulseHalfPeriodUs);
  }
}
