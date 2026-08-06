#include "metal/metal.h"

#include "driver/pcnt.h"

namespace {

constexpr int16_t kPcntHighLimit = 30000;
// ~1.25 µs glitch filter at 80 MHz APB (ignores sub-µs noise spikes).
constexpr uint16_t kPcntFilterApbCycles = 100;

float countsToHz(uint32_t counts, uint32_t elapsedUs) {
  if (elapsedUs == 0) {
    return 0.0f;
  }
  return static_cast<float>(counts) * (1000000.0f / static_cast<float>(elapsedUs));
}

pcnt_unit_t unitFromIndex(int unitIndex) {
  return unitIndex == 0 ? PCNT_UNIT_0 : PCNT_UNIT_1;
}

}  // namespace

// ---------------------------------------------------------------------------
// Hardware PCNT setup / read
// ---------------------------------------------------------------------------

bool MetalDetector::setupPcnt(int pin, int unitIndex) {
  const pcnt_unit_t unit = unitFromIndex(unitIndex);

  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = pin;
  cfg.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  cfg.channel = PCNT_CHANNEL_0;
  cfg.unit = unit;
  cfg.pos_mode = PCNT_COUNT_INC;  // rising edges
  cfg.neg_mode = PCNT_COUNT_DIS;
  cfg.lctrl_mode = PCNT_MODE_KEEP;
  cfg.hctrl_mode = PCNT_MODE_KEEP;
  cfg.counter_h_lim = kPcntHighLimit;
  cfg.counter_l_lim = 0;

  if (pcnt_unit_config(&cfg) != ESP_OK) {
    Serial.printf("[METAL] PCNT unit %d config failed on GPIO %d\n", unitIndex,
                  pin);
    return false;
  }

  pcnt_set_filter_value(unit, kPcntFilterApbCycles);
  pcnt_filter_enable(unit);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
  return true;
}

uint32_t MetalDetector::readAndClearPcnt(int unitIndex) {
  const pcnt_unit_t unit = unitFromIndex(unitIndex);
  int16_t count = 0;
  pcnt_get_counter_value(unit, &count);
  pcnt_counter_clear(unit);
  if (count < 0) {
    count = 0;
  }
  return static_cast<uint32_t>(count);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void MetalDetector::begin(const MetalDetectorConfig& config) {
  config_ = config;
  state_ = MetalDetectorState{};
  pcntReady_ = false;

  pinMode(config_.leftPin, INPUT);
  pinMode(config_.rightPin, INPUT);

  // Hardware counters — keep counting through Wire/OLED critical sections that
  // previously caused GPIO-ISR pulse loss (~2–3 kHz live under-read).
  const bool leftOk = setupPcnt(config_.leftPin, 0);
  const bool rightOk = setupPcnt(config_.rightPin, 1);
  pcntReady_ = leftOk && rightOk;

  if (!pcntReady_) {
    Serial.println(F("[METAL] PCNT init failed — frequencies will be zero"));
  } else {
    Serial.println(F("[METAL] PCNT ready on both coils"));
  }

  gateStartUs_ = micros();
}

// ---------------------------------------------------------------------------
// Blocking calibration — robot must be stationary and away from metal.
// ---------------------------------------------------------------------------

void MetalDetector::calibrate() {
  delay(500);

  readAndClearPcnt(0);
  readAndClearPcnt(1);
  gateStartUs_ = micros();

  float sumL = 0.0f;
  float sumR = 0.0f;
  uint16_t samples = 0;

  const uint32_t calibStartMs = millis();
  while (millis() - calibStartMs < config_.baselineDurationMs) {
    MetalDetectorState sample;
    if (update(sample)) {
      sumL += sample.leftHz;
      sumR += sample.rightHz;
      samples++;
    }
  }

  if (samples == 0) {
    samples = 1;
  }

  state_.baselineLeft = sumL / static_cast<float>(samples);
  state_.baselineRight = sumR / static_cast<float>(samples);
  state_.deltaLeftHz = 0.0f;
  state_.deltaRightHz = 0.0f;
  state_.leftHit = false;
  state_.rightHit = false;
  state_.side = MetalSide::None;

  Serial.printf("[METAL] baseline ready after %lu ms (%u samples): "
                "L=%.0f Hz R=%.0f Hz\n",
                static_cast<unsigned long>(config_.baselineDurationMs), samples,
                state_.baselineLeft, state_.baselineRight);

  readAndClearPcnt(0);
  readAndClearPcnt(1);
  gateStartUs_ = micros();
}

// ---------------------------------------------------------------------------
// Non-blocking gate — one reading per gateTimeMs, driven off micros().
// ---------------------------------------------------------------------------

bool MetalDetector::update(MetalDetectorState& state) {
  const uint32_t gateUs = config_.gateTimeMs * 1000UL;
  if (static_cast<uint32_t>(micros() - gateStartUs_) < gateUs) {
    return false;
  }

  uint32_t countL = 0;
  uint32_t countR = 0;
  if (pcntReady_) {
    countL = readAndClearPcnt(0);
    countR = readAndClearPcnt(1);
  }

  const uint32_t endUs = micros();
  const uint32_t elapsedUs = endUs - gateStartUs_;
  gateStartUs_ = endUs;

  state_.leftHz = countsToHz(countL, elapsedUs);
  state_.rightHz = countsToHz(countR, elapsedUs);

  // Hits only on a frequency rise above baseline (drops are ignored).
  const float deltaL = state_.leftHz - state_.baselineLeft;
  const float deltaR = state_.rightHz - state_.baselineRight;
  state_.deltaLeftHz = deltaL > 0.0f ? deltaL : 0.0f;
  state_.deltaRightHz = deltaR > 0.0f ? deltaR : 0.0f;
  state_.leftHit = state_.deltaLeftHz > config_.thresholdLeftHz;
  // Right coil temporarily ignored — still measured for OLED, never triggers.
  state_.rightHit = config_.enableRightDetector &&
                    (state_.deltaRightHz > config_.thresholdRightHz);

  if (state_.leftHit && state_.rightHit) {
    state_.side = (state_.deltaLeftHz >= state_.deltaRightHz) ? MetalSide::Left
                                                              : MetalSide::Right;
  } else if (state_.leftHit) {
    state_.side = MetalSide::Left;
  } else if (state_.rightHit) {
    state_.side = MetalSide::Right;
  } else {
    state_.side = MetalSide::None;
  }

  state = state_;
  return true;
}
