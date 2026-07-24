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
  ledcWrite(bridge.channel0, 0);
  ledcWrite(bridge.channel1, 0);
}

void MotorDriver::setBridgeSpeed(Bridge& bridge, float speed) {
  speed = constrain(speed, -static_cast<float>(kSpeedMax),
                    static_cast<float>(kSpeedMax));
  if (!kAllowReverse && speed < 0.0f) {
    speed = 0.0f;
  }

  if (fabsf(speed) < 0.5f) {
    bridgeAllOff(bridge);
    bridge.direction = 0;
    bridge.lastSpeed = 0.0f;
    bridge.lastDuty = 0;
    return;
  }

  const int newDirection = speed > 0.0f ? 1 : -1;
  const int speedPercent = speedToPercent(speed);
  const uint32_t duty = percentToDuty(speedPercent);
  bridge.lastSpeed = speed;
  bridge.lastDuty = duty;

  if (newDirection == bridge.direction) {
    if (newDirection > 0) {
      ledcWrite(bridge.channel1, 0);
      ledcWrite(bridge.channel0, duty);
    } else {
      ledcWrite(bridge.channel0, 0);
      ledcWrite(bridge.channel1, duty);
    }
    return;
  }

  const int prevDirection = bridge.direction;
  bridgeAllOff(bridge);

  if (prevDirection != 0 && newDirection != prevDirection) {
    delay(kSwitchDeadtimeMs);  // deadtime when flipping FWD <-> REV
  }

  bridge.direction = newDirection;

  if (newDirection > 0) {
    ledcWrite(bridge.channel1, 0);
    ledcWrite(bridge.channel0, duty);
  } else {
    ledcWrite(bridge.channel0, 0);
    ledcWrite(bridge.channel1, duty);
  }
}

// ---------------------------------------------------------------------------
// Hardware init
// ---------------------------------------------------------------------------

void MotorDriver::initBridge(Bridge& bridge) {
  ledcSetup(bridge.channel0, kPwmFreqHz, kPwmResolutionBits);
  ledcSetup(bridge.channel1, kPwmFreqHz, kPwmResolutionBits);
  ledcAttachPin(bridge.pin0, bridge.channel0);
  ledcAttachPin(bridge.pin1, bridge.channel1);
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
