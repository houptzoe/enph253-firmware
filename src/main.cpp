#include <Arduino.h>

#include "display/display.h"
#include "hardware/pins.h"
#include "motor/motor.h"
#include "pid/pid.h"

// Robot application — wires tape-follow PID output to the motor driver.

static MotorDriver motors;
static TapeFollowPid tapeFollow;
static ReflectanceDisplay reflectanceDisplay;

static constexpr float kBaseSpeed = 90.0f;
static constexpr float kMaxSpeed = 150.0f;

// ---------------------------------------------------------------------------
// Subsystem setup helpers
// ---------------------------------------------------------------------------

static void initTapeFollow() {
  TapeFollowConfig config;
  config.leftReflectancePin = kLeftReflectancePin;
  config.rightReflectancePin = kRightReflectancePin;
  config.reflectanceThreshold = 650;
  config.kp = 45.0f;
  config.ki = 0.0f;
  config.kd = 10.0f;
  config.integralMax = 10.0f;
  tapeFollow.begin(config);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  pinMode(kRotationDirPin, OUTPUT);
  pinMode(kRotationStepPin, OUTPUT);
  pinMode(kVerticalDirPin, OUTPUT);
  pinMode(kVerticalStepPin, OUTPUT);
  pinMode(kHorizontalDirPin, OUTPUT);
  pinMode(kHorizontalStepPin, OUTPUT);
  pinMode(kSwitch0Pin, INPUT_PULLUP);

  digitalWrite(kRotationStepPin, LOW);
  digitalWrite(kRotationDirPin, LOW);
  digitalWrite(kVerticalStepPin, LOW);
  digitalWrite(kVerticalDirPin, LOW);
  digitalWrite(kHorizontalDirPin, LOW);
  digitalWrite(kHorizontalStepPin, LOW);

  Serial.begin(115200);
  delay(200);

  Serial.printf("[BOOT] Reset reason: %d (1=poweron 3=sw 4=panic 5/6/7=wdt "
                "9=brownout)\n",
                esp_reset_reason());

  motors.begin();
  reflectanceDisplay.begin();
  initTapeFollow();
}

void loop() {
  TapeFollowState state;
  if (tapeFollow.update(state)) {
    // Differential drive: subtract correction from left, add to right.
    const float leftSpeed =
        constrain(kBaseSpeed - state.correction, 0.0f, kMaxSpeed);
    const float rightSpeed =
        constrain(kBaseSpeed + state.correction, 0.0f, kMaxSpeed);
    motors.applyDrive(leftSpeed, rightSpeed);

    reflectanceDisplay.showReadings(state.leftAvg, state.rightAvg,
                                    state.leftOnTape, state.rightOnTape);

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
}
