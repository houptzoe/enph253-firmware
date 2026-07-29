#include "claw/claw.h"

void ClawServo::begin(int pin) {
  pin_ = pin;
  maxDuty_ = (1u << kPwmResolutionBits) - 1u;
  ledcSetup(kPwmChannel, kPwmFreqHz, kPwmResolutionBits);
  ledcAttachPin(pin_, kPwmChannel);
  openClaw();
}

void ClawServo::setAngle(int degrees) {
  if (pin_ < 0) {
    return;
  }
  degrees = constrain(degrees, 0, 180);
  const int pulseUs =
      map(degrees, 0, 180, kMinPulseUs, kMaxPulseUs);
  const uint32_t duty = static_cast<uint32_t>(
      (static_cast<float>(pulseUs) / 20000.0f) * static_cast<float>(maxDuty_));
  ledcWrite(kPwmChannel, duty);
}

void ClawServo::openClaw() { setAngle(kOpenAngleDeg); }

void ClawServo::closeClaw() { setAngle(kCloseAngleDeg); }
