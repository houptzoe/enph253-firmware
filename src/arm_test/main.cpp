#include <Arduino.h>

#include "claw/claw.h"
#include "display/display.h"
#include "hardware/pins.h"
#include "metal/metal.h"
#include "stepper/stepper.h"

// ---------------------------------------------------------------------------
// Tunable constants (bench-adjust direction polarity and limits)
// ---------------------------------------------------------------------------

// Phase 1 — motion checkout
constexpr int kCheckoutRotateSteps = 100;
constexpr int kCheckoutVerticalSteps = 100;
constexpr int kCheckoutHorizontalSteps = 100;

// Phase 2 — pick
constexpr int kPickRotateSteps = 800;
constexpr int kPickLowerSteps = 800;
constexpr int kHorizontalMaxExtendSteps = 1200;  // hard mechanical max; tune
constexpr float kMetalDeltaThresholdHz = 50.0f;
constexpr uint32_t kFreqWindowMs = 80;

// DIR levels — flip a bool if an axis runs the wrong way.
constexpr bool kRotateDirLeft = true;       // DIR HIGH = toward left
constexpr bool kRotateDirRight = false;
constexpr bool kVerticalDirUp = true;
constexpr bool kVerticalDirDown = false;
constexpr bool kHorizontalDirOut = true;
constexpr bool kHorizontalDirIn = false;

// Microswitches: INPUT_PULLUP, pressed == LOW. Flip if wiring is opposite.
constexpr bool kSwitchPressedLevel = false;  // LOW when pressed

// ---------------------------------------------------------------------------

static StepperMotor rotation;
static StepperMotor vertical;
static StepperMotor horizontal;
static ClawServo claw;
static MetalDisplay oled;

static bool switchPressed(int pin) {
  return digitalRead(pin) == (kSwitchPressedLevel ? HIGH : LOW);
}

static bool eitherSwitchPressed() {
  return switchPressed(kSwitch0Pin) || switchPressed(kSwitch1Pin);
}

enum class ExtendStopReason { kSwitch, kMaxExtend };

static ExtendStopReason extendUntilSwitchOrMax() {
  for (int i = 0; i < kHorizontalMaxExtendSteps; ++i) {
    if (eitherSwitchPressed()) {
      return ExtendStopReason::kSwitch;
    }
    horizontal.step(1, kHorizontalDirOut);
  }
  return ExtendStopReason::kMaxExtend;
}

static void runMotionCheckout() {
  Serial.println("[arm] checkout: rotate");
  rotation.step(kCheckoutRotateSteps, kRotateDirLeft);

  Serial.println("[arm] checkout: vertical up");
  vertical.step(kCheckoutVerticalSteps, kVerticalDirUp);

  Serial.println("[arm] checkout: horizontal out");
  horizontal.step(kCheckoutHorizontalSteps, kHorizontalDirOut);
  Serial.println("[arm] checkout: horizontal in");
  horizontal.step(kCheckoutHorizontalSteps, kHorizontalDirIn);

  Serial.println("[arm] checkout: vertical down");
  vertical.step(kCheckoutVerticalSteps, kVerticalDirDown);
}

static MetalSide waitForMetal(const MetalBaseline& baseline) {
  oled.show(baseline.leftHz, baseline.rightHz, baseline.leftHz,
            baseline.rightHz, "Waiting");

  for (;;) {
    const float liveL = measureFreqHz(kMetalDetectorLeftPin, kFreqWindowMs);
    const float liveR = measureFreqHz(kMetalDetectorRightPin, kFreqWindowMs);
    const MetalSide side = detectMetalSide(
        baseline.leftHz, baseline.rightHz, liveL, liveR, kMetalDeltaThresholdHz);

    const char* status = "Waiting";
    if (side == MetalSide::kLeft) {
      status = "METAL LEFT";
    } else if (side == MetalSide::kRight) {
      status = "METAL RIGHT";
    }

    oled.show(baseline.leftHz, baseline.rightHz, liveL, liveR, status);
    Serial.printf("[metal] L:%.0f (d%.0f) R:%.0f (d%.0f) %s\n", liveL,
                  fabsf(liveL - baseline.leftHz), liveR,
                  fabsf(liveR - baseline.rightHz), status);

    if (side != MetalSide::kNone) {
      return side;
    }
  }
}

static void runPick(MetalSide side) {
  const bool rotateLeft = (side == MetalSide::kLeft);
  Serial.printf("[arm] pick: rotate %s (%d steps)\n",
                rotateLeft ? "LEFT" : "RIGHT", kPickRotateSteps);
  oled.show(0, 0, 0, 0, rotateLeft ? "Rotate L" : "Rotate R");
  rotation.step(kPickRotateSteps, rotateLeft ? kRotateDirLeft : kRotateDirRight);

  Serial.printf("[arm] pick: lower (%d steps)\n", kPickLowerSteps);
  oled.show(0, 0, 0, 0, "Lower");
  vertical.step(kPickLowerSteps, kVerticalDirDown);

  Serial.printf("[arm] pick: extend (max %d)\n", kHorizontalMaxExtendSteps);
  oled.show(0, 0, 0, 0, "Extend");
  const ExtendStopReason stop = extendUntilSwitchOrMax();
  if (stop == ExtendStopReason::kSwitch) {
    Serial.println("[arm] extend stop: switch");
    oled.show(0, 0, 0, 0, "Stop:switch");
  } else {
    Serial.println("[arm] extend stop: max_extend");
    oled.show(0, 0, 0, 0, "Stop:max_ext");
  }

  Serial.println("[arm] pick: close claw");
  oled.show(0, 0, 0, 0, "Close claw");
  claw.closeClaw();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("[arm_test] boot");

  pinMode(kSwitch0Pin, INPUT_PULLUP);
  pinMode(kSwitch1Pin, INPUT_PULLUP);

  rotation.begin(kRotationDirPin, kRotationStepPin);
  vertical.begin(kVerticalDirPin, kVerticalStepPin);
  horizontal.begin(kHorizontalDirPin, kHorizontalStepPin);
  claw.begin(kServoMotorPin);
  oled.begin();

  runMotionCheckout();

  Serial.println("[arm] sampling metal baseline...");
  oled.show(0, 0, 0, 0, "Baseline...");
  const MetalBaseline baseline =
      sampleBaseline(kMetalDetectorLeftPin, kMetalDetectorRightPin, kFreqWindowMs);
  Serial.printf("[metal] baseline L:%.0f Hz R:%.0f Hz\n", baseline.leftHz,
                baseline.rightHz);
  oled.show(baseline.leftHz, baseline.rightHz, baseline.leftHz, baseline.rightHz,
            "Waiting");

  const MetalSide side = waitForMetal(baseline);
  runPick(side);

  Serial.println("[arm_test] done — idle");
  oled.show(baseline.leftHz, baseline.rightHz, 0, 0, "Done");
}

void loop() { delay(1000); }
