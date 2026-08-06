#include <Arduino.h>

#include "display/display.h"
#include "hardware/pins.h"
#include "mission/mission.h"
#include "motor/motor.h"
#include "pid/pid.h"
#include "sensors/vision.h"
#include "telemetry/telemetry.h"

// Dual-cam teletubby handshake — one DETECT per START; second find = new START.
// Line-follow gains/speeds from feature/pid. Drive + vision arm at boot.

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
  config.kp = 45.0f;
  config.ki = 0.0f;
  config.kd = 15.0f;
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
      telemetry.setDriveRunning(true);
      mission.start();
      Serial.println("[CMD] mission start");
    } else if (line == "!ABORT" || line == "ABORT" || line == "!STOP") {
      mission.abort();
      telemetry.setDriveRunning(false);
      motors.stop();
      Serial.println("[CMD] mission abort");
    } else if (line == "!TT" || line == "TT" || line == "!DETECT" ||
               line == "!TT0" || line == "TT0") {
      vision.inject(0);
      Serial.println("[CMD] inject DETECT_CAM0");
    } else if (line == "!TT1" || line == "TT1") {
      vision.inject(1);
      Serial.println("[CMD] inject DETECT_CAM1");
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

  // Competition boot: motors armed + Pi START immediately (no SoftAP tap).
  telemetry.setDriveRunning(true);
  mission.start();

  Serial.println(
      "[BOOT] drive + dual-cam vision armed — Serial: !START !ABORT "
      "!TT0 !TT1 !CLR");
}

void loop() {
  pollSerialCommands();

  switch (telemetry.takeVisionCommand()) {
    case VisionCommand::Start:
      telemetry.setDriveRunning(true);
      mission.start();
      break;
    case VisionCommand::Stop:
      mission.abort();
      telemetry.setDriveRunning(false);
      motors.stop();
      break;
    case VisionCommand::None:
      break;
  }

  mission.update();
  telemetry.updateMissionStatus(mission.phaseName(),
                                mission.lastDetectedCamera(),
                                mission.detectCount());

  const DriveSettings& drive = telemetry.drive();
  const MissionDriveCommand cmd = mission.driveCommand();

  // SoftAP Start/Stop mirrors drive enable and (re)arms vision.
  static bool wasRunning = true;  // matches boot-armed state
  if (drive.running && !wasRunning) {
    if (!mission.active()) {
      mission.start();
    }
  } else if (!drive.running && wasRunning && mission.active()) {
    mission.abort();
  }
  wasRunning = drive.running;

  // Keep PID sampling whenever we are alive so resume stays on tape.
  TapeFollowState state;
  const bool havePid = tapeFollow.update(state);

  // Drive active ⇒ tape-follow PWM. Mission PauseOnDetect ⇒ all PWM off
  // every loop (do not wait for the next PID tick).
  const bool pauseMotors =
      mission.active() && cmd.mode == MissionDriveCommand::Mode::Stop;
  const bool motorsAllowed = drive.running && !pauseMotors;

  float leftSpeed = 0.0f;
  float rightSpeed = 0.0f;

  if (!motorsAllowed) {
    motors.stop();
  } else if (havePid) {
    leftSpeed = constrain(drive.leftBaseSpeed - state.correction, 0.0f,
                          drive.maxSpeed);
    rightSpeed = constrain(drive.rightBaseSpeed + state.correction, 0.0f,
                           drive.maxSpeed);
    motors.applyDrive(leftSpeed, rightSpeed);
  }

  if (havePid) {
    if (mission.lastDetectedCamera() >= 0) {
      reflectanceDisplay.showTeletubbyDetected(mission.lastDetectedCamera(),
                                               mission.detectCount());
    } else {
      reflectanceDisplay.showReadings(state.leftAvg, state.rightAvg,
                                      state.leftOnTape, state.rightOnTape);
    }

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
