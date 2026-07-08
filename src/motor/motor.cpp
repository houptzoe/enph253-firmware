#include "motor/motor.h"

#ifndef BATTERY_VOLTAGE
#define BATTERY_VOLTAGE 16.0f
#endif
#ifndef MOTOR_MAX_VOLTAGE
#define MOTOR_MAX_VOLTAGE 15.0f
#endif

// ---------------------------------------------------------------------------
// Voltage-limited duty scaling
// ---------------------------------------------------------------------------

uint32_t MotorDriver::percentToDuty(int speedPercent) const {
  return static_cast<uint32_t>((abs(speedPercent) / 100.0f) *
                               static_cast<float>(maxDuty_));
}

int MotorDriver::speedToPercent(float speed) const {
  const float clamped = constrain(fabsf(speed), 0.0f,
                                  static_cast<float>(kSpeedMax));
  return static_cast<int>((clamped / static_cast<float>(kSpeedMax)) * 100.0f);
}

// ---------------------------------------------------------------------------
// Per-bridge PWM control — forward on pin0, reverse on pin1
// ---------------------------------------------------------------------------

void MotorDriver::bridgeAllOff(const Bridge& bridge) {
  ledcWrite(bridge.pin0, 0);
  ledcWrite(bridge.pin1, 0);
}

void MotorDriver::setBridgeSpeed(Bridge& bridge, float speed) {
  speed = constrain(speed, -static_cast<float>(kSpeedMax),
                    static_cast<float>(kSpeedMax));

  if (fabsf(speed) < 0.5f) {
    bridgeAllOff(bridge);
    bridge.direction = 0;
    return;
  }

  const int newDirection = speed > 0.0f ? 1 : -1;
  const int speedPercent = speedToPercent(speed);
  const uint32_t duty = percentToDuty(speedPercent);

  if (newDirection == bridge.direction) {
    if (newDirection > 0) {
      ledcWrite(bridge.pin1, 0);
      ledcWrite(bridge.pin0, duty);
    } else {
      ledcWrite(bridge.pin0, 0);
      ledcWrite(bridge.pin1, duty);
    }
    return;
  }

  const int prevDirection = bridge.direction;
  bridgeAllOff(bridge);

  if (prevDirection != 0 && newDirection != prevDirection) {
    delay(kSwitchDeadtimeMs);
  }

  bridge.direction = newDirection;

  if (newDirection > 0) {
    ledcWrite(bridge.pin1, 0);
    ledcWrite(bridge.pin0, duty);
  } else {
    ledcWrite(bridge.pin0, 0);
    ledcWrite(bridge.pin1, duty);
  }
}

// ---------------------------------------------------------------------------
// Hardware init
// ---------------------------------------------------------------------------

void MotorDriver::initBridge(Bridge& bridge) {
  ledcAttach(bridge.pin0, kPwmFreqHz, kPwmResolutionBits);
  ledcAttach(bridge.pin1, kPwmFreqHz, kPwmResolutionBits);
  bridgeAllOff(bridge);
  bridge.direction = 0;
}

void MotorDriver::begin() {
  const uint32_t pwmDutyMax = (1u << kPwmResolutionBits) - 1u;
  maxDuty_ = static_cast<uint32_t>(
      (MOTOR_MAX_VOLTAGE / BATTERY_VOLTAGE) * static_cast<float>(pwmDutyMax));

  initBridge(left_);
  initBridge(right_);
}

// ---------------------------------------------------------------------------
// Differential drive command
// ---------------------------------------------------------------------------

void MotorDriver::applyDrive(float leftSpeed, float rightSpeed) {
  setBridgeSpeed(left_, leftSpeed);
  setBridgeSpeed(right_, rightSpeed);
}

void MotorDriver::stop() { applyDrive(0.0f, 0.0f); }
