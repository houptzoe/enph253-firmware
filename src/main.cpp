#include <Arduino.h>

#include "display/display.h"
#include "hardware/pins.h"
#include "motor/motor.h"
#include "pid/pid.h"

// Robot application — wires tape-follow PID output to the motor driver.

// ---------------------------------------------------------------------------
// Robot-level tuning
// ---------------------------------------------------------------------------
constexpr float kBaseSpeed = 120.0f;  // forward duty added before steering correction

static MotorDriver motors;
static TapeFollowPid tapeFollow;
static ReflectanceDisplay reflectanceDisplay;

// ---------------------------------------------------------------------------
// Subsystem setup helpers
// ---------------------------------------------------------------------------

static void initTapeFollow() {
  TapeFollowConfig config;
  config.leftReflectancePin = kLeftReflectancePin;
  config.rightReflectancePin = kRightReflectancePin;
  config.reflectanceThreshold = 650;
  tapeFollow.begin(config);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  motors.begin();
  reflectanceDisplay.begin();
  initTapeFollow();
}

void loop() {
  TapeFollowState state;
  if (!tapeFollow.update(state)) {
    return;
  }

  // Differential drive: subtract correction from left, add to right.
  const float leftSpeed = constrain(kBaseSpeed - state.correction,
                                    -MotorDriver::kPwmMax, MotorDriver::kPwmMax);
  const float rightSpeed = constrain(kBaseSpeed + state.correction,
                                     -MotorDriver::kPwmMax, MotorDriver::kPwmMax);
  motors.applyDrive(leftSpeed, rightSpeed);
  reflectanceDisplay.showReadings(state.leftAvg, state.rightAvg);

  // Periodic debug output over USB serial.
  static uint32_t lastLogMs = 0;
  const uint32_t nowMs = millis();
  if (nowMs - lastLogMs >= 200) {
    lastLogMs = nowMs;
    Serial.printf(
        "L:%4d(%d) R:%4d(%d) err:%.1f corr:%.1f Lspd:%.0f Rspd:%.0f\n",
        state.leftAvg, state.leftOnTape, state.rightAvg, state.rightOnTape,
        state.error, state.correction, leftSpeed, rightSpeed);
  }
}
