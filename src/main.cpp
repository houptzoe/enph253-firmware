#include <Arduino.h>
#include <Wire.h>

#include "arm/arm.h"
#include "display/display.h"
#include "hardware/pins.h"
#include "metal/metal.h"
#include "motor/motor.h"
#include "pid/pid.h"
#include "sensors/imu.h"
#include "sonar/sonar.h"
#include "telemetry/telemetry.h"

// Robot application — line-follows via tape-follow PID until the metal
// detector fires, drives straight for a short approach, then runs the
// pickup-arm sequence before resuming the line.

static MotorDriver motors;
static TapeFollowPid tapeFollow;
static ReflectanceDisplay reflectanceDisplay;
static TelemetryServer telemetry;
static MetalDetector metalDetector;
static PickupArm arm;
static UltrasonicSonar sonar;
static ImuTracker imu;

enum class RobotMode { LineFollowing, ApproachAfterMetal, PickingUp, Halted };
static RobotMode mode = RobotMode::LineFollowing;

static bool imuReady = false;
static float imuLastYawDeg = 0.0f;
static float imuTurnedDeg = 0.0f;

static MetalSide pendingPickupSide = MetalSide::None;
static uint32_t approachStartMs = 0;
static constexpr uint32_t kMetalApproachMs = 1800;

namespace {

float wrapDeltaDeg(float deg) {
  while (deg > 180.0f) {
    deg -= 360.0f;
  }
  while (deg < -180.0f) {
    deg += 360.0f;
  }
  return deg;
}

void resetImuTurnTracking(float currentYawDeg) {
  imuLastYawDeg = currentYawDeg;
  imuTurnedDeg = 0.0f;
}

// Updates IMU turn accumulation; returns true when a new sample was fused.
bool updateImuTurnTracking() {
  if (!imuReady) {
    return false;
  }

  ImuPose pose;
  if (!imu.update(pose)) {
    return false;
  }

  const float dyaw = wrapDeltaDeg(pose.yawDeg - imuLastYawDeg);
  imuLastYawDeg = pose.yawDeg;
  imuTurnedDeg += dyaw;
  return true;
}

}  // namespace

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
  // Physical left coil was labeled Right in software (RotateScan followed R yaw).
  config.leftPin = kMetalDetectorRightPin;
  config.rightPin = kMetalDetectorLeftPin;
  config.gateTimeMs = 100;              // Tune on hardware.
  config.thresholdLeftHz = 400.0f;      // Tune on hardware.
  config.thresholdRightHz = 400.0f;     // Tune on hardware.
  config.enableRightDetector = true;
  config.baselineDurationMs = 3000;     // 3 s no-metal baseline at boot.
  metalDetector.begin(config);
}

static void initArm() {
  PickupArmConfig config;  // defaults pull pins/steps from pins.h

  // Gripper angles — always applied (tune and full sequence).
  config.servoOpenDeg = 110;
  config.servoClosedDeg = 180;

  // Per-side yaw for SetupRotate and RotateScan (must match each other).
  config.rotateDirLeft = false;
  config.rotateDirRight = true;

  // Tune series after metal through stow + open gripper.
  // Set tuneMode false for full pickup.
  config.tuneMode = true;
  config.tuneExtendSteps = 200;
  config.tuneRotateSteps = 280;
  config.tuneLowerSteps = 5000;
  config.tuneRaiseSteps = 5500;
  config.tuneLowerStepDelayUs = 1000;
  config.maxExtendSteps = 300;  // total horizontal steps from home (incl. first extend)

  // Setup-from-stow — tune on hardware (used when tuneMode is false).
  config.setupRaiseSteps = 300;
  config.setupRotateStepsLeft = 400;
  config.setupRotateStepsRight = 400;
  config.setupExtendSteps = 80;
  config.setupLowerSteps = 2300;
  // Sonar lock window while rotating at rock level — tune on hardware.
  config.sonarDetectMaxCm = 20.0f;
  config.sonarDetectMinCm = 4.0f;
  config.postScanYawAdjustMs = 500;  // metal vs sonar center offset
  config.rotateScanMaxSteps = 1200;
  arm.begin(config, &sonar, &reflectanceDisplay);
}

static void initSonar() {
  UltrasonicConfig config;
  config.trigPin = kSonarTrigPin;
  config.echoPin = kSonarEchoPin;
  config.samplePeriodMs = 100;
  sonar.begin(config);
}

static void initImu() {
  // OLED already owns Wire on the display pins; IMU uses Wire1.
  Wire1.begin(kImuSdaPin, kImuSclPin);
  Wire1.setClock(400000);

  reflectanceDisplay.showMessage("IMU calib", "keep still");
  if (!imu.begin(Wire1)) {
    reflectanceDisplay.showMessage("IMU failed", "check wiring");
    imuReady = false;
    return;
  }

  ImuPose pose;
  const uint32_t settleStart = millis();
  while (millis() - settleStart < 1500) {
    imu.update(pose);
    delay(5);
  }
  imu.resetPose();
  imu.update(pose);
  resetImuTurnTracking(pose.yawDeg);
  imuReady = true;
  Serial.println(F("[IMU] Ready — turn angle shown on OLED"));
}

// ---------------------------------------------------------------------------
// Mode handlers
// ---------------------------------------------------------------------------

static void runLineFollowing() {
  // Cached so OLED can refresh from tape, metal, sonar, or IMU updates.
  static int leftAnalog = 0;
  static int rightAnalog = 0;
  static float leftHz = 0.0f;
  static float rightHz = 0.0f;
  static float baselineLeft = 0.0f;
  static float baselineRight = 0.0f;
  static bool metalLeftHit = false;
  static bool metalRightHit = false;
  static float distanceCm = 0.0f;
  static bool distanceValid = false;
  static bool oledDirty = false;

  TapeFollowState state;
  if (tapeFollow.update(state)) {
    const DriveSettings& drive = telemetry.drive();
    float leftSpeed = 0.0f;
    float rightSpeed = 0.0f;

    if (drive.running) {
      // Differential drive: add correction to left, subtract from right.
      leftSpeed = constrain(drive.leftBaseSpeed + state.correction, 0.0f,
                            drive.maxSpeed);
      rightSpeed = constrain(drive.rightBaseSpeed - state.correction, 0.0f,
                             drive.maxSpeed);
      motors.applyDrive(leftSpeed, rightSpeed);
    } else {
      motors.stop();
    }

    leftAnalog = state.leftAvg;
    rightAnalog = state.rightAvg;
    oledDirty = true;

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

  // Refresh metal fields every gate; only start a pickup while driving.
  MetalDetectorState metalState;
  if (metalDetector.update(metalState)) {
    leftHz = metalState.leftHz;
    rightHz = metalState.rightHz;
    baselineLeft = metalState.baselineLeft;
    baselineRight = metalState.baselineRight;
    metalLeftHit = metalState.leftHit;
    metalRightHit = metalState.rightHit;
    oledDirty = true;

    if (telemetry.drive().running && metalState.side != MetalSide::None) {
      const bool left = metalState.side == MetalSide::Left;
      const float baselineHz =
          left ? metalState.baselineLeft : metalState.baselineRight;
      const float deltaHz =
          left ? metalState.deltaLeftHz : metalState.deltaRightHz;
      const float freqHz = left ? metalState.leftHz : metalState.rightHz;

      Serial.printf("[METAL] hit %c freq:%.0f baseline:%.0f delta:%.0f Hz — "
                    "approach %lu ms\n",
                    left ? 'L' : 'R', freqHz, baselineHz, deltaHz,
                    static_cast<unsigned long>(kMetalApproachMs));
      pendingPickupSide = metalState.side;
      approachStartMs = millis();
      mode = RobotMode::ApproachAfterMetal;
      reflectanceDisplay.showMetalHit(left ? 'L' : 'R', baselineHz, deltaHz);
      return;
    }
  }

  float sonarCm = 0.0f;
  bool sonarValid = false;
  if (sonar.update(sonarCm, sonarValid)) {
    distanceCm = sonarCm;
    distanceValid = sonarValid;
    oledDirty = true;
  }

  if (updateImuTurnTracking()) {
    oledDirty = true;
  }

  if (oledDirty) {
    oledDirty = false;
    reflectanceDisplay.showStatus(leftHz, rightHz, baselineLeft, baselineRight,
                                  metalLeftHit, metalRightHit, leftAnalog,
                                  rightAnalog, distanceCm, distanceValid,
                                  imuTurnedDeg);
  }
}

static const char* phaseName(PickupPhase phase) {
  switch (phase) {
    case PickupPhase::SetupRaise:   return "SetupRaise";
    case PickupPhase::SetupRotate:  return "SetupRotate";
    case PickupPhase::SetupExtend:  return "SetupExtend";
    case PickupPhase::SetupLower:   return "SetupLower";
    case PickupPhase::RotateScan:   return "RotateScan";
    case PickupPhase::PostScanYawAdjust: return "PostScanYaw";
    case PickupPhase::Extend:       return "Extend";
    case PickupPhase::Grip:         return "Grip";
    case PickupPhase::Retract:      return "Retract";
    case PickupPhase::Raise:        return "Raise";
    case PickupPhase::Recenter:     return "Recenter";
    case PickupPhase::RetractInitial: return "RetractInit";
    case PickupPhase::OpenGrip:     return "OpenGrip";
    case PickupPhase::Done:         return "Done";
    default:                        return "Idle";
  }
}

static void runApproachAfterMetal() {
  // Keep line-following for kMetalApproachMs, then stop and pick up.
  if (millis() - approachStartMs < kMetalApproachMs) {
    TapeFollowState state;
    if (tapeFollow.update(state)) {
      const DriveSettings& drive = telemetry.drive();
      if (drive.running) {
        const float leftSpeed = constrain(drive.leftBaseSpeed + state.correction,
                                          0.0f, drive.maxSpeed);
        const float rightSpeed =
            constrain(drive.rightBaseSpeed - state.correction, 0.0f,
                      drive.maxSpeed);
        motors.applyDrive(leftSpeed, rightSpeed);
      } else {
        motors.stop();
      }
    }
    updateImuTurnTracking();
    return;
  }

  motors.stop();
  Serial.println(F("[METAL] approach done — starting pickup"));
  arm.startPickup(pendingPickupSide);
  mode = RobotMode::PickingUp;
}

static void runPickingUp() {
  motors.stop();  // stay put while the arm works

  PickupPhase phase;
  const bool done = arm.update(phase);

  static PickupPhase lastShownPhase = PickupPhase::Idle;
  if (phase != lastShownPhase) {
    lastShownPhase = phase;
    Serial.printf("[ARM] phase: %s\n", phaseName(phase));
    // OLED left on the metal-hit screen (baseline + delta) while the arm moves.
  }

  updateImuTurnTracking();

  if (done) {
    if (arm.isTuneMode()) {
      Serial.println(F("[ARM] tune series done — halted (power-cycle to retry)"));
      lastShownPhase = PickupPhase::Idle;
      mode = RobotMode::Halted;
      return;
    }
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
  motors.stop();  // stay still through baseline capture
  reflectanceDisplay.begin();
  initTapeFollow();
  initMetalDetector();
  initSonar();
  initArm();

  // Metal-detector baseline must be taken with the robot stationary and away
  // from any metal target. Blocks for baselineDurationMs (~3 s) before any
  // hit detection or line following can run — motors stay stopped the whole time.
  reflectanceDisplay.showMessage("Baseline 3s", "stand still");
  metalDetector.calibrate();

  initImu();
  reflectanceDisplay.showMessage("Baseline OK", "ready to drive");

  mode = RobotMode::LineFollowing;
}

void loop() {
  if (mode == RobotMode::LineFollowing) {
    runLineFollowing();
  } else if (mode == RobotMode::ApproachAfterMetal) {
    runApproachAfterMetal();
  } else if (mode == RobotMode::PickingUp) {
    runPickingUp();
  } else {
    // Halted after tune series — stay stopped until power cycle.
    motors.stop();
  }

  telemetry.poll();
}
