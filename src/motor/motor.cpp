#include "motor/motor.h"

#include "hardware/pins.h"

// ---------------------------------------------------------------------------
// Single motor channel — direction GPIO + LEDC duty from signed speed
// ---------------------------------------------------------------------------

void MotorDriver::setMotor(int pwmPin, int dirPin, float speed) {
  const bool forward = speed >= 0.0f;
  digitalWrite(dirPin, forward ? HIGH : LOW);

  const int duty = constrain(static_cast<int>(fabsf(speed)), 0, kPwmMax);
  ledcWrite(pwmPin, duty);
}

// ---------------------------------------------------------------------------
// Hardware init — configure direction pins and attach LEDC PWM
// ---------------------------------------------------------------------------

void MotorDriver::begin() {
  pinMode(kLeftMotorDirPin, OUTPUT);
  pinMode(kRightMotorDirPin, OUTPUT);
  stop();

  ledcAttach(kLeftMotorPwmPin, kPwmFreqHz, kPwmResolutionBits);
  ledcAttach(kRightMotorPwmPin, kPwmFreqHz, kPwmResolutionBits);
}

// ---------------------------------------------------------------------------
// Differential drive command
// ---------------------------------------------------------------------------

void MotorDriver::applyDrive(float leftSpeed, float rightSpeed) {
  setMotor(kLeftMotorPwmPin, kLeftMotorDirPin, leftSpeed);
  setMotor(kRightMotorPwmPin, kRightMotorDirPin, rightSpeed);
}

void MotorDriver::stop() { applyDrive(0.0f, 0.0f); }
