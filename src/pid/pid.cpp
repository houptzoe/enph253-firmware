#include "pid/pid.h"

#include <esp_err.h>
#include <esp_timer.h>

// ---------------------------------------------------------------------------
// PID core
// ---------------------------------------------------------------------------

TapeFollowPid::PidController::PidController(float kp, float ki, float kd,
                                            float integralMax)
    : kp_(kp), ki_(ki), kd_(kd), integralMax_(integralMax) {}

float TapeFollowPid::PidController::update(float error, float dtSec) {
  integral_ += error * dtSec;
  integral_ = constrain(integral_, -integralMax_, integralMax_);

  const float derivative =
      (dtSec > 0.0f) ? (error - lastError_) / dtSec : 0.0f;
  lastError_ = error;

  return kp_ * error + ki_ * integral_ + kd_ * derivative;
}

void TapeFollowPid::PidController::reset() {
  integral_ = 0.0f;
  lastError_ = 0.0f;
}

// ---------------------------------------------------------------------------
// Initialization — ADC setup and 10 kHz sample timer
// ---------------------------------------------------------------------------

void TapeFollowPid::begin(const TapeFollowConfig& config) {
  config_ = config;
  pid_ = PidController(config.kp, config.ki, config.kd, config.integralMax);
  lastKnownError_ = 0.0f;
  updateReady_ = false;
  leftSum_ = 0;
  rightSum_ = 0;
  sampleCount_ = 0;

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  const esp_timer_create_args_t timerArgs = {
      .callback = &TapeFollowPid::sampleTimerCallback,
      .arg = this,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "reflectance_sampler",
      .skip_unhandled_events = true,
  };

  esp_timer_handle_t sampleTimer = nullptr;
  ESP_ERROR_CHECK(esp_timer_create(&timerArgs, &sampleTimer));
  ESP_ERROR_CHECK(
      esp_timer_start_periodic(sampleTimer, config_.samplePeriodUs));
}

float TapeFollowPid::controlPeriodSec() const {
  return static_cast<float>(config_.samplePeriodUs) *
         static_cast<float>(config_.samplesPerUpdate) / 1'000'000.0f;
}

void TapeFollowPid::reset() { pid_.reset(); }

// ---------------------------------------------------------------------------
// Thresholding and line-error from pseudo-digital sensor states
// ---------------------------------------------------------------------------

bool TapeFollowPid::onTape(int reading) const {
  return reading > config_.reflectanceThreshold;
}

float TapeFollowPid::digitalLineError(bool leftOn, bool rightOn) {
  // Sensors at front, wheels at rear — pivot mid-chassis.
  // +1: left on tape, right off → veered right → steer left.
  // -1: right on tape, left off → veered left → steer right.
  //  0: both on tape (centered) or both off (lost — hold last correction).
  if (leftOn && rightOn) {
    return 0.0f;
  }
  if (leftOn && !rightOn) {
    return 1.0f;
  }
  if (!leftOn && rightOn) {
    return -1.0f;
  }
  return lastKnownError_;
}

// ---------------------------------------------------------------------------
// 10 kHz sampler — averages readings, then publishes one control tick
// ---------------------------------------------------------------------------

void TapeFollowPid::sampleTimerCallback(void* arg) {
  auto* self = static_cast<TapeFollowPid*>(arg);

  const int left = analogRead(self->config_.leftReflectancePin);
  const int right = analogRead(self->config_.rightReflectancePin);

  portENTER_CRITICAL(&self->sensorMux_);
  self->leftSum_ += static_cast<uint32_t>(left);
  self->rightSum_ += static_cast<uint32_t>(right);
  self->sampleCount_++;

  if (self->sampleCount_ >= self->config_.samplesPerUpdate) {
    const uint16_t n = self->config_.samplesPerUpdate;
    const int leftAvg = static_cast<int>(self->leftSum_ / n);
    const int rightAvg = static_cast<int>(self->rightSum_ / n);
    const bool leftOn = self->onTape(leftAvg);
    const bool rightOn = self->onTape(rightAvg);
    const float error = self->digitalLineError(leftOn, rightOn);

    if (leftOn != rightOn || (leftOn && rightOn)) {
      self->lastKnownError_ = error;
    }

    self->snapshot_.leftAvg = leftAvg;
    self->snapshot_.rightAvg = rightAvg;
    self->snapshot_.leftOnTape = leftOn;
    self->snapshot_.rightOnTape = rightOn;
    self->snapshot_.error = error;

    self->leftSum_ = 0;
    self->rightSum_ = 0;
    self->sampleCount_ = 0;
    self->updateReady_ = true;
  }
  portEXIT_CRITICAL(&self->sensorMux_);
}

// ---------------------------------------------------------------------------
// Control loop tick — run PID and hand result to main
// ---------------------------------------------------------------------------

bool TapeFollowPid::update(TapeFollowState& state) {
  if (!updateReady_) {
    return false;
  }

  TapeFollowState snap;
  portENTER_CRITICAL(&sensorMux_);
  snap = snapshot_;
  updateReady_ = false;
  portEXIT_CRITICAL(&sensorMux_);

  snap.correction = pid_.update(snap.error, controlPeriodSec());
  state = snap;
  return true;
}
