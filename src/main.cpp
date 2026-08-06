#include <Arduino.h>
#include <Wire.h>

#include "arm/arm.h"
#include "display/display.h"
#include "hardware/pins.h"
#include "hardware/status_leds.h"
#include "metal/metal.h"
#include "motor/motor.h"
#include "pid/pid.h"
#include "sensors/imu.h"
#include "sensors/vision.h"
#include "sonar/sonar.h"

// Line-follow + metal pickup + dual-cam teletubby handshake.
// IMU on claw: course yaw freezes during pickup; first ~180° triggers
// ramp speed; open-loop shortcut at ramp exit (L135/R0 → L135/R90).

static MotorDriver motors;
static TapeFollowPid tapeFollow;
static ReflectanceDisplay reflectanceDisplay;
static MetalDetector metalDetector;
static PickupArm arm;
static UltrasonicSonar sonar;
static ImuTracker imu;
static VisionInference vision;
static StatusLeds statusLeds;

enum class RobotMode { LineFollowing, ApproachAfterMetal, PickingUp };
static RobotMode mode = RobotMode::LineFollowing;

enum class CoursePhase {
  PreFirst180,    // cruise @ 90 until first ~180
  AfterFirst180,  // ramp 4.5s; L135/R0 0.4s; L135/R90 0.4s; then LF @ 90
  PreSecond180,   // cruise @ 90; watch for second ~180
};
static CoursePhase coursePhase = CoursePhase::PreFirst180;

static bool imuReady = false;
static float imuLastYawDeg = 0.0f;
static float imuTurnedDeg = 0.0f;
static bool courseYawFrozen = false;
static float savedCourseYawDeg = 0.0f;
static bool imuCourseOriginLocked = false;
static float imuSettleRefYawDeg = 0.0f;
static uint32_t imuSettleStableSinceMs = 0;
static uint32_t imuSettleWatchStartMs = 0;
static constexpr float kImuSettleTolDeg = 2.5f;
static constexpr uint32_t kImuSettleHoldMs = 1500;
static constexpr uint32_t kImuSettleTimeoutMs = 12000;

static bool secondTurnArmed = false;
static uint32_t secondTurnArmAfterMs = 0;
static bool rampStaggerStarted = false;
static bool rampShortcutDone = false;
static bool rampBlendDone = false;
static uint32_t rampShortcutEndAtMs = 0;
static uint32_t rampBlendEndAtMs = 0;

static float driveLeftBaseSpeed = 90.0f;
static float driveRightBaseSpeed = 90.0f;

static MetalSide pendingPickupSide = MetalSide::None;
static uint32_t approachStartMs = 0;
static constexpr uint32_t kMetalApproachMs = 1800;
static constexpr float kCruiseBaseSpeed = 90.0f;
static constexpr float kRampBaseSpeed = 135.0f;
static constexpr float kDriveMaxSpeed = 150.0f;
static constexpr float kFirstTurnDetectDeg = 160.0f;
static constexpr float kSecondTurnRearmDeg = 30.0f;
static constexpr uint32_t kSecondTurnCooldownMs = 4500;
static constexpr uint32_t kRampStaggerMs = 400;
static constexpr uint32_t kRampBlendMs = 400;
static constexpr uint8_t kRequiredTeletubbyFinds = 2;
static constexpr uint8_t kArrowBlinkCount = 3;
static constexpr uint32_t kArrowBlinkOnMs = 200;
static constexpr uint32_t kArrowBlinkOffMs = 200;
static bool restartVisionAfterPickup = false;

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

void setBaseSpeeds(float left, float right) {
  driveLeftBaseSpeed = left;
  driveRightBaseSpeed = right;
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
  Serial.printf("[IMU] course yaw restored to %.1f deg\n", imuTurnedDeg);
}

void lockImuCourseOrigin(float settledYawDeg, bool timedOut) {
  imu.resetPose();
  imuTurnedDeg = 0.0f;
  imuLastYawDeg = 0.0f;
  imuCourseOriginLocked = true;
  Serial.printf("[IMU] course origin locked at settled yaw %.1f deg%s\n",
                settledYawDeg, timedOut ? " (timeout)" : "");
  reflectanceDisplay.showMessage(
      timedOut ? "IMU lock (timeout)" : "IMU settled", "ok to drive");
}

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
      imuTurnedDeg = 0.0f;
      imuLastYawDeg = pose.yawDeg;
    }
    return true;
  }

  const float dyaw = wrapDeltaDeg(pose.yawDeg - imuLastYawDeg);
  imuLastYawDeg = pose.yawDeg;
  if (!courseYawFrozen) {
    imuTurnedDeg += dyaw;
  }
  return true;
}

const char* coursePhaseName(CoursePhase phase) {
  switch (phase) {
    case CoursePhase::PreFirst180:   return "PreFirst180";
    case CoursePhase::AfterFirst180: return "AfterFirst180";
    case CoursePhase::PreSecond180:  return "PreSecond180";
    default:                         return "?";
  }
}

void updateCoursePhaseFromImu() {
  if (!imuReady || !imuCourseOriginLocked ||
      mode != RobotMode::LineFollowing) {
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
    resetImuTurnTracking(imuLastYawDeg);
    setBaseSpeeds(kRampBaseSpeed, kRampBaseSpeed);
    Serial.printf("[COURSE] %s — first turn >=%.0f (%.1f deg), ramp %.0f for %lu ms\n",
                  coursePhaseName(coursePhase), kFirstTurnDetectDeg, turned,
                  kRampBaseSpeed,
                  static_cast<unsigned long>(kSecondTurnCooldownMs));
    return;
  }

  if (coursePhase == CoursePhase::AfterFirst180) {
    if (millis() >= secondTurnArmAfterMs) {
      if (!rampStaggerStarted) {
        rampStaggerStarted = true;
        rampShortcutEndAtMs = millis() + kRampStaggerMs;
        setBaseSpeeds(kRampBaseSpeed, 0.0f);
        Serial.println(F("[COURSE] open-loop shortcut 400 ms (L135/R0)"));
      } else if (!rampShortcutDone && millis() >= rampShortcutEndAtMs) {
        rampShortcutDone = true;
        rampBlendEndAtMs = millis() + kRampBlendMs;
        setBaseSpeeds(kRampBaseSpeed, kCruiseBaseSpeed);
        Serial.println(F("[COURSE] open-loop blend 400 ms (L135/R90)"));
      } else if (rampShortcutDone && !rampBlendDone &&
                 millis() >= rampBlendEndAtMs) {
        rampBlendDone = true;
        setBaseSpeeds(kCruiseBaseSpeed, kCruiseBaseSpeed);
        tapeFollow.reset();
        Serial.println(F("[COURSE] blend done — LF reset, search straight @ 90"));
      } else if (rampBlendDone) {
        setBaseSpeeds(kCruiseBaseSpeed, kCruiseBaseSpeed);
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

void maybeFinishRampOpenLoop() {
  if (coursePhase != CoursePhase::AfterFirst180 || !rampStaggerStarted) {
    return;
  }

  if (!rampShortcutDone && millis() >= rampShortcutEndAtMs) {
    rampShortcutDone = true;
    rampBlendEndAtMs = millis() + kRampBlendMs;
    setBaseSpeeds(kRampBaseSpeed, kCruiseBaseSpeed);
    Serial.println(F("[COURSE] open-loop blend 400 ms (L135/R90)"));
  }

  if (rampShortcutDone && !rampBlendDone && millis() >= rampBlendEndAtMs) {
    rampBlendDone = true;
    setBaseSpeeds(kCruiseBaseSpeed, kCruiseBaseSpeed);
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
  config.samplePeriodUs = 2000;
  config.samplesPerUpdate = 5;
  // Explicit zeros required — TapeFollowConfig defaults are kp=45, kd=10.
  config.kp = 55.0f;
  config.ki = 0.0f;
  config.kd = 18.0f;
  config.integralMax = 10.0f;
  tapeFollow.begin(config);
}

static void initMetalDetector() {
  MetalDetectorConfig config;
  config.leftPin = kMetalDetectorRightPin;
  config.rightPin = kMetalDetectorLeftPin;
  config.gateTimeMs = 100;
  config.thresholdLeftHz = 400.0f;
  config.thresholdRightHz = 400.0f;
  config.enableRightDetector = true;
  config.baselineDurationMs = 3000;
  metalDetector.begin(config);
}

static void initArm() {
  PickupArmConfig config;

  config.servoOpenDeg = 110;
  config.servoClosedDeg = 180;
  config.rotateDirLeft = false;
  config.rotateDirRight = true;
  config.initialExtendSteps = 200;
  config.rotateSteps = 280;
  config.lowerSteps = 5000;
  config.raiseSteps = 5500;
  config.verticalStepDelayUs = 1000;
  config.maxExtendSteps = 300;
  config.sonarDetectMaxCm = 20.0f;
  config.sonarDetectMinCm = 4.0f;
  config.postScanYawAdjustMs = 500;
  config.rotateScanMaxSteps = 1200;
  config.recenterLeftNum = 1;
  config.recenterLeftDen = 4;
  config.recenterRightNum = 5;
  config.recenterRightDen = 8;

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
  Serial.println(F("[IMU] Ready — waiting for course origin lock"));
}

// ---------------------------------------------------------------------------
// Mode handlers
// ---------------------------------------------------------------------------

static void runLineFollowing() {
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

  if (isRampOpenLoopActive()) {
    if (isRampShortcutActive()) {
      motors.applyDrive(kRampBaseSpeed, 0.0f);
    } else {
      motors.applyDrive(kRampBaseSpeed, kCruiseBaseSpeed);
    }
  }

  TapeFollowState state;
  if (tapeFollow.update(state)) {
    float leftSpeed = 0.0f;
    float rightSpeed = 0.0f;

    if (isRampShortcutActive()) {
      leftSpeed = kRampBaseSpeed;
      rightSpeed = 0.0f;
    } else if (isRampBlendActive()) {
      leftSpeed = kRampBaseSpeed;
      rightSpeed = kCruiseBaseSpeed;
    } else {
      leftSpeed = constrain(driveLeftBaseSpeed - state.correction, 0.0f,
                            kDriveMaxSpeed);
      rightSpeed = constrain(driveRightBaseSpeed + state.correction, 0.0f,
                             kDriveMaxSpeed);
      motors.applyDrive(leftSpeed, rightSpeed);
    }

    leftAnalog = state.leftAvg;
    rightAnalog = state.rightAvg;
    oledDirty = true;

    static uint32_t lastLogMs = 0;
    const uint32_t nowMs = millis();
    if (nowMs - lastLogMs >= 200) {
      lastLogMs = nowMs;
      Serial.printf(
          "run:%d L:%4d(%d) R:%4d(%d) err:%.1f corr:%.1f Lspd:%.0f Rspd:%.0f\n",
          1, state.leftAvg, state.leftOnTape, state.rightAvg, state.rightOnTape,
          state.error, state.correction, leftSpeed, rightSpeed);
    }
  }

  MetalDetectorState metalState;
  if (metalDetector.update(metalState)) {
    leftHz = metalState.leftHz;
    rightHz = metalState.rightHz;
    baselineLeft = metalState.baselineLeft;
    baselineRight = metalState.baselineRight;
    metalLeftHit = metalState.leftHit;
    metalRightHit = metalState.rightHit;
    oledDirty = true;

    if (metalState.side != MetalSide::None) {
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
    updateCoursePhaseFromImu();
  }

  if (oledDirty) {
    oledDirty = false;
    reflectanceDisplay.showStatus(leftHz, rightHz, baselineLeft, baselineRight,
                                  metalLeftHit, metalRightHit, leftAnalog,
                                  rightAnalog, distanceCm, distanceValid,
                                  imuTurnedDeg);
  }
}

static void handleVisionDetect(const VisionDetectResult& detect) {
  const bool inPickupFlow =
      mode == RobotMode::ApproachAfterMetal || mode == RobotMode::PickingUp;
  const bool complete = detect.detectCount >= kRequiredTeletubbyFinds;

  if (inPickupFlow) {
    restartVisionAfterPickup = !complete;
    Serial.printf("[VISION] detect cam%d during pickup flow -> defer (restart:%d)\n",
                  static_cast<int>(detect.camera),
                  restartVisionAfterPickup ? 1 : 0);
    if (complete) {
      vision.enable(false);
    }
    return;
  }

  motors.stop();
  statusLeds.startBlink(detect.camera, kArrowBlinkCount, kArrowBlinkOnMs,
                        kArrowBlinkOffMs);
  while (!statusLeds.update()) {
    delay(2);
  }
  statusLeds.off();

  Serial.printf("[VISION] detect cam%d #%u/%u handled\n",
                static_cast<int>(detect.camera),
                static_cast<unsigned>(detect.detectCount),
                static_cast<unsigned>(kRequiredTeletubbyFinds));

  if (!complete) {
    vision.enable(true);
  } else {
    vision.enable(false);
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
  if (millis() - approachStartMs < kMetalApproachMs) {
    TapeFollowState state;
    if (tapeFollow.update(state)) {
      const float leftSpeed =
          constrain(driveLeftBaseSpeed - state.correction, 0.0f, kDriveMaxSpeed);
      const float rightSpeed =
          constrain(driveRightBaseSpeed + state.correction, 0.0f, kDriveMaxSpeed);
      motors.applyDrive(leftSpeed, rightSpeed);
    }
    updateImuTurnTracking();
    return;
  }

  motors.stop();
  Serial.println(F("[METAL] approach done — starting pickup"));
  freezeCourseYawForPickup();
  arm.startPickup(pendingPickupSide);
  mode = RobotMode::PickingUp;
}

static void runPickingUp() {
  motors.stop();

  PickupPhase phase;
  const bool done = arm.update(phase);

  static PickupPhase lastShownPhase = PickupPhase::Idle;
  if (phase != lastShownPhase) {
    lastShownPhase = phase;
    Serial.printf("[ARM] phase: %s\n", phaseName(phase));
  }

  updateImuTurnTracking();

  if (done) {
    unfreezeCourseYawAfterPickup();
    Serial.println("[ARM] pickup complete, resuming line following");
    if (restartVisionAfterPickup) {
      restartVisionAfterPickup = false;
      vision.enable(true);
      Serial.println("[VISION] restarted after pickup-time detect");
    }
    tapeFollow.reset();
    lastShownPhase = PickupPhase::Idle;
    mode = RobotMode::LineFollowing;
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.printf("[BOOT] Reset reason: %d (1=poweron 3=sw 4=panic 5/6/7=wdt "
                "9=brownout)\n", esp_reset_reason());

  motors.begin();
  motors.stop();
  reflectanceDisplay.begin();
  initTapeFollow();
  initMetalDetector();
  initSonar();
  initArm();
  vision.begin();
  vision.enable(true);
  statusLeds.begin();

  setBaseSpeeds(kCruiseBaseSpeed, kCruiseBaseSpeed);
  coursePhase = CoursePhase::PreFirst180;
  secondTurnArmed = false;

  reflectanceDisplay.showMessage("Baseline 3s", "stand still");
  metalDetector.calibrate();

  initImu();
  reflectanceDisplay.showMessage("IMU settling", "wait for lock");

  mode = RobotMode::LineFollowing;
}

void loop() {
  const VisionDetectResult detect = vision.poll();
  if (detect.found) {
    handleVisionDetect(detect);
  }

  if (mode == RobotMode::LineFollowing) {
    runLineFollowing();
  } else if (mode == RobotMode::ApproachAfterMetal) {
    runApproachAfterMetal();
  } else {
    runPickingUp();
  }
}
