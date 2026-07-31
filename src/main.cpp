#include <Arduino.h>

#include "display/display.h"
#include "hardware/pins.h"
#include "mission/mission.h"
#include "motor/motor.h"
#include "pid/pid.h"
#include "sensors/vision.h"
#include "telemetry/telemetry.h"

// Teletubby handshake test — tape-follow always; Pi START/DETECT via vision.

static MotorDriver motors;
static TapeFollowPid tapeFollow;
static ReflectanceDisplay reflectanceDisplay;
static TelemetryServer telemetry;
static VisionInference vision;
static MissionController mission;

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
  // Gains from feature/pid, sized for SoftAP cruise speeds.
  config.kp = 35.0f;
  config.ki = 0.0f;
  config.kd = 12.0f;
  config.integralMax = 10.0f;
  tapeFollow.begin(config);
}

// Bench inject for teletubby DETECT without the Pi.
static void pollSerialCommands() {
  while (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    line.toUpperCase();
    if (line.length() == 0) {
      continue;
    }

    if (line == "!START" || line == "START") {
      mission.start();
      Serial.println("[CMD] mission start");
    } else if (line == "!ABORT" || line == "ABORT" || line == "!STOP") {
      mission.abort();
      Serial.println("[CMD] mission abort");
    } else if (line == "!TT" || line == "TT" || line == "!DETECT") {
      vision.inject();
      Serial.println("[CMD] inject teletubby DETECT");
    } else if (line == "!CLR" || line == "CLR") {
      vision.clearInject();
      Serial.println("[CMD] clear injects");
    }
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);  // USB-CDC ready before we log SoftAP status

  // Diagnostic: distinguish brownout resets (weak supply) from crashes.
  Serial.printf("[BOOT] Reset reason: %d (1=poweron 3=sw 4=panic 5/6/7=wdt "
                "9=brownout)\n",
                esp_reset_reason());

  // Bring SoftAP up first so a hung OLED/I2C init cannot block WiFi.
  telemetry.begin(tapeFollow, motors);

  motors.begin();
  reflectanceDisplay.begin();
  initTapeFollow();

  vision.begin();
  mission.begin(vision);

  Serial.println(
      "[BOOT] teletubby handshake test — Serial: !START !ABORT !TT !CLR");
}

void loop() {
  pollSerialCommands();
  mission.update();

  TapeFollowState state;
  if (tapeFollow.update(state)) {
    const DriveSettings& drive = telemetry.drive();
    float leftSpeed = 0.0f;
    float rightSpeed = 0.0f;

    // SoftAP Start/Stop also arms/aborts the mission FSM.
    static bool wasRunning = false;
    if (drive.running && !wasRunning && !mission.active()) {
      mission.start();
    } else if (!drive.running && wasRunning && mission.active()) {
      mission.abort();
    }
    wasRunning = drive.running;

    const MissionDriveCommand cmd = mission.driveCommand();
    const bool missionRunning = mission.active();

    // SoftAP left/right base speeds apply in both mission and manual drive.
    if (missionRunning) {
      if (cmd.mode == MissionDriveCommand::Mode::TapeFollow) {
        leftSpeed = constrain(drive.leftBaseSpeed - state.correction, 0.0f,
                              drive.maxSpeed);
        rightSpeed = constrain(drive.rightBaseSpeed + state.correction, 0.0f,
                               drive.maxSpeed);
        motors.applyDrive(leftSpeed, rightSpeed);
      } else {
        motors.stop();
      }
    } else if (drive.running) {
      // Manual tape-follow after abort (or before first start).
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
          "run:%d miss:%s L:%4d(%d) R:%4d(%d) err:%.1f corr:%.1f "
          "Lspd:%.0f Rspd:%.0f\n",
          drive.running ? 1 : 0, mission.phaseName(), state.leftAvg,
          state.leftOnTape, state.rightAvg, state.rightOnTape, state.error,
          state.correction, leftSpeed, rightSpeed);
    }
  }

  telemetry.poll();
}
