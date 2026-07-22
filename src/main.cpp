#include <Arduino.h>

#include "display/display.h"
#include "hardware/pins.h"
#include "motor/motor.h"
#include "pid/pid.h"
#include "telemetry/telemetry.h"

// Robot application — wires tape-follow PID output to the motor driver.

static MotorDriver motors;
static TapeFollowPid tapeFollow;
static ReflectanceDisplay reflectanceDisplay;
static TelemetryServer telemetry;

// ---------------------------------------------------------------------------
// Subsystem setup helpers
// ---------------------------------------------------------------------------

static void initTapeFollow() {
  TapeFollowConfig config;
  config.leftReflectancePin = kLeftReflectancePin;
  config.rightReflectancePin = kRightReflectancePin;
  config.reflectanceThreshold = 650;
  // Slower sampler so SoftAP beacons are not starved on CPU0 (was 500 us).
  config.samplePeriodUs = 2000;
  config.samplesPerUpdate = 5;  // still ~100 Hz control
  // Gains sized for low base speed (was kp=80, which overpowered cruise).
  config.kp = 25.0f;
  //config.ki = 0.8f;
  //config.kd = 4.0f;
  config.integralMax = 10.0f;
  tapeFollow.begin(config);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);  // USB-CDC ready before we log SoftAP status

  // Diagnostic: distinguish brownout resets (weak supply) from crashes.
  Serial.printf("[BOOT] Reset reason: %d (1=poweron 3=sw 4=panic 5/6/7=wdt "
                "9=brownout)\n", esp_reset_reason());

  // Bring SoftAP up first so a hung OLED/I2C init cannot block WiFi.
  telemetry.begin(tapeFollow, motors);

  motors.begin();
  reflectanceDisplay.begin();
  initTapeFollow();
}

void loop() {
  TapeFollowState state;
  if (tapeFollow.update(state)) {
    const DriveSettings& drive = telemetry.drive();
    float leftSpeed = 0.0f;
    float rightSpeed = 0.0f;

    if (drive.running) {
      // Differential drive: subtract correction from left, add to right.
      leftSpeed = constrain(drive.leftBaseSpeed - state.correction, 0.0f,
                            drive.maxSpeed);
      rightSpeed = constrain(drive.rightBaseSpeed + state.correction, 0.0f,
                             drive.maxSpeed);
      motors.applyDrive(leftSpeed, rightSpeed);
    } else {
      motors.stop();
    }

    reflectanceDisplay.showReadings(state.leftAvg, state.rightAvg,
                                    state.leftOnTape, state.rightOnTape);

    TelemetrySnapshot snap;
    snap.error = state.error;
    snap.correction = state.correction;
    telemetry.updateSnapshot(snap);

    static uint32_t lastLogMs = 0;
    const uint32_t nowMs = millis();
    if (nowMs - lastLogMs >= 200) {
      lastLogMs = nowMs;
      Serial.printf(
          "run:%d L:%4d(%d) R:%4d(%d) err:%.1f corr:%.1f Lspd:%.0f Rspd:%.0f\n",
          drive.running ? 1 : 0, state.leftAvg, state.leftOnTape, state.rightAvg,
          state.rightOnTape, state.error, state.correction, leftSpeed,
          rightSpeed);
    }
  }

  telemetry.poll();
}
