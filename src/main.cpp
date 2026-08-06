#include <Arduino.h>
#include <Wire.h>

#include "arm/arm.h"
#include "display/display.h"
#include "hardware/pins.h"
#include "ir/ir.h"
#include "metal/metal.h"
#include "motor/motor.h"
#include "pid/pid.h"
#include "sensors/imu.h"
#include "sonar/sonar.h"
#include "telemetry/telemetry.h"

// Robot application — line-follows via tape-follow PID, picks up on metal
// until the second IMU ~180° turn, deploys the arm, then line-follows to the
// IR beacon (band set by hardware switch) and pivots/halts.
// Single IMU is on the claw: course yaw is frozen during pickup so claw
// rotation does not count toward the chassis ~180° markers.

static MotorDriver motors;
static TapeFollowPid tapeFollow;
static ReflectanceDisplay reflectanceDisplay;
static TelemetryServer telemetry;
static MetalDetector metalDetector;
static PickupArm arm;
static UltrasonicSonar sonar;
static ImuTracker imu;
static IrSensor irSensor;

enum class RobotMode {
  LineFollowing,
  ApproachAfterMetal,
  PickingUp,
  IrPivot,  // right-wheel spin toward solar panel after IR beacon
  Halted,
};
static RobotMode mode = RobotMode::LineFollowing;

// Course timeline (line-follow drives motion; IMU only detects ~180° completes).
enum class CoursePhase {
  PreFirst180,    // metal on, cruise 90
  AfterFirst180,  // ramp 4.5s; L135/R0 0.4s; L135/R90 0.4s; then LF @ 90
  PreSecond180,   // metal on, cruise 90; watch for second ~180
  PostSecond180,  // arm deploy, then line-follow with IR until beacon
};
static CoursePhase coursePhase = CoursePhase::PreFirst180;

static bool imuReady = false;
static float imuLastYawDeg = 0.0f;
static float imuTurnedDeg = 0.0f;
// IMU rides on the claw: freeze course ~180° accumulation while the arm yaws
// so claw motion (and mid-turn metal pickups) never counts as chassis turn.
static bool courseYawFrozen = false;
static float savedCourseYawDeg = 0.0f;
// Wait until yaw stops drifting, then lock that heading as course origin (0).
static bool imuCourseOriginLocked = false;
static float imuSettleRefYawDeg = 0.0f;
static uint32_t imuSettleStableSinceMs = 0;
static uint32_t imuSettleWatchStartMs = 0;
static constexpr float kImuSettleTolDeg = 2.5f;       // max wander while "stable"
static constexpr uint32_t kImuSettleHoldMs = 1500;    // must hold that long
static constexpr uint32_t kImuSettleTimeoutMs = 12000; // force-lock if never quiet


static bool metalEnabled = false;  // TEST: metal detectors disengaged
static bool secondTurnArmed = false;
static uint32_t secondTurnArmAfterMs = 0;
// After ramp: L135/R0 → L135/R90 open-loop, then resume LF @ 90.
static bool rampStaggerStarted = false;
static bool rampShortcutDone = false;
static bool rampBlendDone = false;
static uint32_t rampShortcutEndAtMs = 0;
static uint32_t rampBlendEndAtMs = 0;

static float irHz = 0.0f;
static bool irHighBand = false;

static MetalSide pendingPickupSide = MetalSide::None;
static uint32_t approachStartMs = 0;
static uint32_t irPivotStartMs = 0;
static constexpr float kCruiseBaseSpeed = 90.0f;
static constexpr float kRampBaseSpeed = 135.0f;
static constexpr float kIrPivotSpeed = 90.0f;
static constexpr uint32_t kIrPivotMs = 2000;
static constexpr uint32_t kMetalApproachMs = 1800;
static constexpr float kTurnDetectDeg = 165.0f;       // second ~180
static constexpr float kFirstTurnDetectDeg = 160.0f;  // start ramp early on first turn
static constexpr float kSecondTurnRearmDeg = 30.0f;
static constexpr uint32_t kSecondTurnCooldownMs = 4500;  // AfterFirst180 ramp duration
static constexpr uint32_t kRampStaggerMs = 400;  // open-loop hard cut (R stopped)
static constexpr uint32_t kRampBlendMs = 400;    // open-loop L135/R90


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

void freezeCourseYawForPickup() {
  savedCourseYawDeg = imuTurnedDeg;
  courseYawFrozen = true;
  Serial.printf("[IMU] course yaw frozen at %.1f deg (claw moving)\n",
                savedCourseYawDeg);
}

void unfreezeCourseYawAfterPickup() {
  imuTurnedDeg = savedCourseYawDeg;
  courseYawFrozen = false;
  // imuLastYawDeg already tracks current claw pose from updates during pickup;
  // with claw back near home, further dyaw is chassis motion again.
  Serial.printf("[IMU] course yaw restored to %.1f deg\n", imuTurnedDeg);
}

void lockImuCourseOrigin(float settledYawDeg, bool timedOut) {
  imu.resetPose();  // settled heading becomes reported 0
  imuTurnedDeg = 0.0f;
  imuLastYawDeg = 0.0f;
  imuCourseOriginLocked = true;
  Serial.printf("[IMU] course origin locked at settled yaw %.1f deg%s\n",
                settledYawDeg, timedOut ? " (timeout)" : "");
  reflectanceDisplay.showMessage(
      timedOut ? "IMU lock (timeout)" : "IMU settled", "ok to drive");
}

// Updates IMU; course accumulator only advances when not frozen for pickup.
// Until yaw settles at boot, drift is ignored — the settled heading becomes
// the start (0) for all ~180° course measurements.
bool updateImuTurnTracking() {
  if (!imuReady) {
    return false;
  }

  ImuPose pose;
  if (!imu.update(pose)) {
    return false;
  }

  if (!imuCourseOriginLocked) {
    const float deltaFromRef =
        fabsf(wrapDeltaDeg(pose.yawDeg - imuSettleRefYawDeg));
    if (deltaFromRef > kImuSettleTolDeg) {
      imuSettleRefYawDeg = pose.yawDeg;
      imuSettleStableSinceMs = millis();
    }

    const bool heldLongEnough =
        (millis() - imuSettleStableSinceMs) >= kImuSettleHoldMs;
    const bool timedOut =
        (millis() - imuSettleWatchStartMs) >= kImuSettleTimeoutMs;

    if (heldLongEnough || timedOut) {
      lockImuCourseOrigin(pose.yawDeg, timedOut && !heldLongEnough);
    } else {
      // Do not count boot drift as course turn; keep OLED near 0.
      imuTurnedDeg = 0.0f;
      imuLastYawDeg = pose.yawDeg;
    }
    return true;
  }

  const float dyaw = wrapDeltaDeg(pose.yawDeg - imuLastYawDeg);
  imuLastYawDeg = pose.yawDeg;
  if (!courseYawFrozen) {
    // Before the first drive, ignore parked drift so it cannot false-trigger 165°.
    if (!telemetry.drive().running &&
        coursePhase == CoursePhase::PreFirst180) {
      imuTurnedDeg = 0.0f;
      imuLastYawDeg = pose.yawDeg;
    } else {
      imuTurnedDeg += dyaw;
    }
  }
  return true;
}

const char* coursePhaseName(CoursePhase phase) {
  switch (phase) {
    case CoursePhase::PreFirst180:   return "PreFirst180";
    case CoursePhase::AfterFirst180: return "AfterFirst180";
    case CoursePhase::PreSecond180:  return "PreSecond180";
    case CoursePhase::PostSecond180: return "PostSecond180";
    default:                         return "?";
  }
}

void enterPostSecond180() {
  coursePhase = CoursePhase::PostSecond180;
  metalEnabled = false;
  irSensor.setEnabled(false);  // enable after arm deploy, then line-follow to IR
  motors.stop();
  reflectanceDisplay.showMessage("2nd 180 done", "arm deploy");
  arm.startPostCourseDeploy();
  mode = RobotMode::PickingUp;
  Serial.println(F("[COURSE] PostSecond180 — stop for arm deploy, then IR run"));
}

// Advances course phases from IMU yaw while line-following (not mid-pickup).
// Hitting ~165° deliberately resets the turn accumulator to measure the next
// leg — that is not a sensor glitch. Detection only runs while motors are on.
void updateCoursePhaseFromImu() {
  if (!imuReady || !imuCourseOriginLocked) {
    return;
  }
  if (!telemetry.drive().running) {
    return;
  }

  const float turned = fabsf(imuTurnedDeg);

  if (coursePhase == CoursePhase::PreFirst180 &&
      turned >= kFirstTurnDetectDeg) {
    coursePhase = CoursePhase::AfterFirst180;
    secondTurnArmed = false;
    secondTurnArmAfterMs = millis() + kSecondTurnCooldownMs;
    rampStaggerStarted = false;
    rampShortcutDone = false;
    rampBlendDone = false;
    resetImuTurnTracking(imuLastYawDeg);  // start next-leg measure from 0
    telemetry.setBaseSpeeds(kRampBaseSpeed, kRampBaseSpeed);
    Serial.printf("[COURSE] %s — first turn >=%.0f (%.1f deg), ramp %.0f for %lu ms\n",
                  coursePhaseName(coursePhase), kFirstTurnDetectDeg, turned,
                  kRampBaseSpeed,
                  static_cast<unsigned long>(kSecondTurnCooldownMs));
    return;
  }

  if (coursePhase == CoursePhase::AfterFirst180) {
    // After ramp: (1) L135/R0 0.4s (2) L135/R90 0.4s (3) resume line-follow.
    if (millis() >= secondTurnArmAfterMs) {
      if (!rampStaggerStarted) {
        rampStaggerStarted = true;
        rampShortcutEndAtMs = millis() + kRampStaggerMs;
        telemetry.setBaseSpeeds(kRampBaseSpeed, 0.0f);
        Serial.println(F("[COURSE] open-loop shortcut 400 ms (L135/R0)"));
      } else if (!rampShortcutDone && millis() >= rampShortcutEndAtMs) {
        rampShortcutDone = true;
        rampBlendEndAtMs = millis() + kRampBlendMs;
        telemetry.setBaseSpeeds(kRampBaseSpeed, kCruiseBaseSpeed);
        Serial.println(F("[COURSE] open-loop blend 400 ms (L135/R90)"));
      } else if (rampShortcutDone && !rampBlendDone &&
                 millis() >= rampBlendEndAtMs) {
        rampBlendDone = true;
        telemetry.setBaseSpeeds(kCruiseBaseSpeed, kCruiseBaseSpeed);
        tapeFollow.reset();
        Serial.println(F("[COURSE] blend done — LF reset, search straight @ 90"));
      } else if (rampBlendDone) {
        telemetry.setBaseSpeeds(kCruiseBaseSpeed, kCruiseBaseSpeed);
      }

      if (rampBlendDone && !secondTurnArmed &&
          turned <= kSecondTurnRearmDeg) {
        secondTurnArmed = true;
        coursePhase = CoursePhase::PreSecond180;
        resetImuTurnTracking(imuLastYawDeg);
        Serial.printf("[COURSE] %s — cruise %.0f, watching second ~180\n",
                      coursePhaseName(coursePhase), kCruiseBaseSpeed);
      }
    }
    return;
  }

  if (coursePhase == CoursePhase::PreSecond180 && turned >= kTurnDetectDeg) {
    Serial.printf("[COURSE] second ~180 done (%.1f deg)\n", turned);
    enterPostSecond180();
    resetImuTurnTracking(imuLastYawDeg);
  }
}

bool isRampShortcutActive() {
  return coursePhase == CoursePhase::AfterFirst180 && rampStaggerStarted &&
         !rampShortcutDone;
}

bool isRampBlendActive() {
  return coursePhase == CoursePhase::AfterFirst180 && rampShortcutDone &&
         !rampBlendDone;
}

bool isRampOpenLoopActive() {
  return isRampShortcutActive() || isRampBlendActive();
}

// Advance open-loop phases on timers (drive loop + IMU path).
void maybeFinishRampOpenLoop() {
  if (coursePhase != CoursePhase::AfterFirst180 || !rampStaggerStarted) {
    return;
  }

  if (!rampShortcutDone && millis() >= rampShortcutEndAtMs) {
    rampShortcutDone = true;
    rampBlendEndAtMs = millis() + kRampBlendMs;
    telemetry.setBaseSpeeds(kRampBaseSpeed, kCruiseBaseSpeed);
    Serial.println(F("[COURSE] open-loop blend 400 ms (L135/R90)"));
  }

  if (rampShortcutDone && !rampBlendDone && millis() >= rampBlendEndAtMs) {
    rampBlendDone = true;
    telemetry.setBaseSpeeds(kCruiseBaseSpeed, kCruiseBaseSpeed);
    tapeFollow.reset();
    Serial.println(F("[COURSE] blend done — LF reset, search straight @ 90"));
  }
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
  // Explicit zeros required — TapeFollowConfig defaults are kp=45, kd=10.
  config.kp = 55.0f;
  config.ki = 0.0f;
  config.kd = 18.0f;
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
  config.baselineDurationMs = 1500;     // 1.5 s no-metal baseline at boot.
  metalDetector.begin(config);
}

static void initArm() {
  PickupArmConfig config;  // defaults pull pins/steps from pins.h

  config.servoOpenDeg = 110;
  config.servoClosedDeg = 180;

  // Per-side yaw for Rotate and RotateScan (must match each other).
  config.rotateDirLeft = false;
  config.rotateDirRight = true;

  // Primary pickup sequence (tuned on hardware) — both L and R metal hits.
  config.initialExtendSteps = 200;
  config.rotateSteps = 280;
  config.lowerSteps = 5000;
  config.raiseSteps = 5500;
  config.verticalStepDelayUs = 1000;
  config.maxExtendSteps = 300;  // total horizontal from home (incl. first extend)
  config.sonarDetectMaxCm = 20.0f;
  config.sonarDetectMinCm = 4.0f;
  config.postScanYawAdjustMs = 500;
  config.rotateScanMaxSteps = 1200;
  config.homeYawToleranceDeg = 4.0f;  // stop when |yaw - home| within this
  config.recenterMaxSteps = 2500;     // safety if IMU never locks
  config.postCourseLeftYawSteps = 350;  // ~90° left — tune on hardware

  arm.begin(config, &sonar, &reflectanceDisplay, &imu);
}

static void initIr() {
  IrConfig config;
  config.sensePin = kIrDetectorPin;
  config.switchPin = kIrSwitchPin;
  config.gateTimeMs = 100;
  irSensor.begin(config);
  // Enabled only after PostSecond180.
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
  imuCourseOriginLocked = false;
  imuSettleRefYawDeg = pose.yawDeg;
  imuSettleStableSinceMs = millis();
  imuSettleWatchStartMs = millis();
  Serial.println(F("[IMU] Ready — waiting for yaw to settle as course origin"));
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

  maybeFinishRampOpenLoop();

  // Open-loop after ramp: L135/R0 → L135/R90 — no PID; then LF @ 90.
  if (telemetry.drive().running && isRampOpenLoopActive()) {
    if (isRampShortcutActive()) {
      motors.applyDrive(kRampBaseSpeed, 0.0f);
    } else {
      motors.applyDrive(kRampBaseSpeed, kCruiseBaseSpeed);
    }
  }

  TapeFollowState state;
  if (tapeFollow.update(state)) {
    const DriveSettings& drive = telemetry.drive();
    float leftSpeed = 0.0f;
    float rightSpeed = 0.0f;

    if (drive.running) {
      if (isRampShortcutActive()) {
        leftSpeed = kRampBaseSpeed;
        rightSpeed = 0.0f;
      } else if (isRampBlendActive()) {
        leftSpeed = kRampBaseSpeed;
        rightSpeed = kCruiseBaseSpeed;
      } else {
        // Differential drive: add correction to left, subtract from right.
        leftSpeed = constrain(drive.leftBaseSpeed - state.correction, 0.0f,
                              drive.maxSpeed);
        rightSpeed = constrain(drive.rightBaseSpeed + state.correction, 0.0f,
                               drive.maxSpeed);
        motors.applyDrive(leftSpeed, rightSpeed);
      }
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

    if (metalEnabled && telemetry.drive().running &&
        metalState.side != MetalSide::None) {
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

  IrState irState;
  if (irSensor.update(irState)) {
    irHz = irState.hz;
    irHighBand = (irState.band == IrBand::High);
    oledDirty = true;

    if (irState.beaconDetected) {
      motors.stop();
      irPivotStartMs = millis();
      mode = RobotMode::IrPivot;
      Serial.printf("[IR] beacon %.0f Hz (%s) — right-wheel pivot %lu ms\n",
                    irState.hz, irHighBand ? "Hi" : "Lo",
                    static_cast<unsigned long>(kIrPivotMs));
      reflectanceDisplay.showMessage("IR beacon", "pivot R");
      return;
    }
  }

  if (updateImuTurnTracking()) {
    oledDirty = true;
    updateCoursePhaseFromImu();
    if (mode != RobotMode::LineFollowing) {
      return;  // second 180 → halted arm deploy
    }
  }

  if (oledDirty) {
    oledDirty = false;
    reflectanceDisplay.showStatus(
        leftHz, rightHz, baselineLeft, baselineRight, metalLeftHit,
        metalRightHit, leftAnalog, rightAnalog, distanceCm, distanceValid,
        imuTurnedDeg, irSensor.isEnabled(), irHz, irHighBand);
  }
}

static const char* phaseName(PickupPhase phase) {
  switch (phase) {
    case PickupPhase::InitialExtend: return "InitialExtend";
    case PickupPhase::Rotate:        return "Rotate";
    case PickupPhase::Lower:         return "Lower";
    case PickupPhase::RotateScan:    return "RotateScan";
    case PickupPhase::PostScanYawAdjust: return "PostScanYaw";
    case PickupPhase::Extend:        return "Extend";
    case PickupPhase::Grip:          return "Grip";
    case PickupPhase::Retract:       return "Retract";
    case PickupPhase::Raise:         return "Raise";
    case PickupPhase::Recenter:      return "Recenter";
    case PickupPhase::RetractInitial: return "RetractInit";
    case PickupPhase::OpenGrip:      return "OpenGrip";
    case PickupPhase::Done:          return "Done";
    default:                         return "Idle";
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
  freezeCourseYawForPickup();
  // Record claw IMU yaw now (still at home) before any arm rotation.
  arm.startPickup(pendingPickupSide, imuLastYawDeg, imuReady);
  mode = RobotMode::PickingUp;
}

static void runIrPivot() {
  // No line-follow — left stopped, right forward at kIrPivotSpeed for kIrPivotMs.
  if (millis() - irPivotStartMs < kIrPivotMs) {
    motors.applyDrive(0.0f, kIrPivotSpeed);
    return;
  }

  motors.stop();
  Serial.println(F("[IR] pivot done — HALT (hook / takeoff)"));
  reflectanceDisplay.showMessage("IR pivot done", "HALT");
  mode = RobotMode::Halted;
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

  // Course phase stays frozen during metal pickup; still fuse IMU.
  updateImuTurnTracking();

  if (done) {
    lastShownPhase = PickupPhase::Idle;
    if (arm.isPostCourseDeploy()) {
      irSensor.setEnabled(true);
      telemetry.setBaseSpeeds(kCruiseBaseSpeed, kCruiseBaseSpeed);
      tapeFollow.reset();
      Serial.println(F("[ARM] post-course deploy done — line-follow, IR ON"));
      reflectanceDisplay.showMessage("Arm deployed", "IR hunt");
      mode = RobotMode::LineFollowing;
      return;
    }
    unfreezeCourseYawAfterPickup();
    Serial.println("[ARM] pickup complete, resuming line following");
    tapeFollow.reset();  // clear any PID windup accumulated while paused
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
  initIr();

  // Metal-detector baseline must be taken with the robot stationary and away
  // from any metal target. Blocks for baselineDurationMs (~1.5 s) before any
  // hit detection or line following can run — motors stay stopped the whole time.
  reflectanceDisplay.showMessage("Baseline 1.5s", "stand still");
  metalDetector.calibrate();

  initImu();
  reflectanceDisplay.showMessage("IMU settling", "wait for lock");

  coursePhase = CoursePhase::PreFirst180;
  metalEnabled = false;  // TEST: metal detectors disengaged
  secondTurnArmed = false;
  mode = RobotMode::LineFollowing;
}

void loop() {
  if (mode == RobotMode::LineFollowing) {
    runLineFollowing();
  } else if (mode == RobotMode::ApproachAfterMetal) {
    runApproachAfterMetal();
  } else if (mode == RobotMode::PickingUp) {
    runPickingUp();
  } else if (mode == RobotMode::IrPivot) {
    runIrPivot();
  } else {
    // Halted after IR pivot — stay stopped for hook / takeoff.
    motors.stop();
  }

  telemetry.poll();
}
