#include <Arduino.h>

#include "arm/arm.h"
#include "display/display.h"
#include "hardware/pins.h"
#include "metal/metal.h"
#include "motor/motor.h"
#include "pid/pid.h"
#include "telemetry/telemetry.h"

// Robot application — line-follows via tape-follow PID until the metal
// detector fires, then pauses driving to run the pickup-arm sequence before
// resuming the line.

static MotorDriver motors;
static TapeFollowPid tapeFollow;
static ReflectanceDisplay reflectanceDisplay;
static TelemetryServer telemetry;
static MetalDetector metalDetector;
static PickupArm arm;

enum class RobotMode { LineFollowing, PickingUp };
static RobotMode mode = RobotMode::LineFollowing;

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

static void initMetalDetector() {
  MetalDetectorConfig config;
  config.leftPin = kMetalDetectorLeftPin;
  config.rightPin = kMetalDetectorRightPin;
  config.gateTimeMs = 100;      // Tune on hardware.
  config.thresholdHz = 3000.0f;  // Tune on hardware.
  metalDetector.begin(config);
}

static void initArm() {
  PickupArmConfig config;  // defaults pull pins/steps from pins.h
  arm.begin(config);
}

// ---------------------------------------------------------------------------
// Mode handlers
// ---------------------------------------------------------------------------

static void runLineFollowing() {
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

  // Only chase metal hits while actually driving a run — avoids the arm
  // firing while the robot is parked/idle on the telemetry page.
  MetalDetectorState metalState;
  if (metalDetector.update(metalState) && telemetry.drive().running) {
    if (metalState.side != MetalSide::None) {
      Serial.printf("[METAL] hit side:%d L:%.0f(%.0f) R:%.0f(%.0f)\n",
                    static_cast<int>(metalState.side), metalState.leftHz,
                    metalState.baselineLeft, metalState.rightHz,
                    metalState.baselineRight);
      motors.stop();
      arm.startPickup(metalState.side);
      mode = RobotMode::PickingUp;
      reflectanceDisplay.showMessage(
          "Metal hit!", metalState.side == MetalSide::Left ? "Side: LEFT"
                                                             : "Side: RIGHT");
    }
  }
}

static const char* phaseName(PickupPhase phase) {
  switch (phase) {
    case PickupPhase::Rotate:   return "Rotate";
    case PickupPhase::Lower:    return "Lower";
    case PickupPhase::Extend:   return "Extend";
    case PickupPhase::Grip:     return "Grip";
    case PickupPhase::Retract:  return "Retract";
    case PickupPhase::Raise:    return "Raise";
    case PickupPhase::Recenter: return "Recenter";
    case PickupPhase::Done:     return "Done";
    default:                    return "Idle";
  }
}

static void runPickingUp() {
  motors.stop();  // stay put while the arm works

  PickupPhase phase;
  const bool done = arm.update(phase);

  static PickupPhase lastShownPhase = PickupPhase::Idle;
  if (phase != lastShownPhase) {
    lastShownPhase = phase;
    Serial.printf("[ARM] phase: %s\n", phaseName(phase));
    reflectanceDisplay.showMessage("Picking up...", phaseName(phase));
  }

  if (done) {
    Serial.println("[ARM] pickup complete, resuming line following");
    tapeFollow.reset();  // clear any PID windup accumulated while paused
    lastShownPhase = PickupPhase::Idle;
    mode = RobotMode::LineFollowing;
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
                "9=brownout)\n", esp_reset_reason());

  // Bring SoftAP up first so a hung OLED/I2C init cannot block WiFi.
  telemetry.begin(tapeFollow, motors);

  motors.begin();
  reflectanceDisplay.begin();
  initTapeFollow();
  initMetalDetector();
  initArm();

  // Metal-detector baseline must be taken with the robot stationary and away
  // from any metal target. Blocking (~calibrationSamples * gateTimeMs).
  reflectanceDisplay.showMessage("Calibrating", "metal detector...");
  metalDetector.calibrate();

  mode = RobotMode::LineFollowing;
}

void loop() {
  if (mode == RobotMode::LineFollowing) {
    runLineFollowing();
  } else {
    runPickingUp();
  }

  telemetry.poll();
}
