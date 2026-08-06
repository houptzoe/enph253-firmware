#include "ir/ir.h"

#include "driver/pcnt.h"

namespace {

// Metal coils already use PCNT_UNIT_0 and PCNT_UNIT_1.
constexpr pcnt_unit_t kIrPcntUnit = PCNT_UNIT_2;
constexpr int16_t kPcntHighLimit = 30000;
constexpr uint16_t kPcntFilterApbCycles = 100;

float countsToHz(uint32_t counts, uint32_t elapsedUs) {
  if (elapsedUs == 0) {
    return 0.0f;
  }
  return static_cast<float>(counts) * (1000000.0f / static_cast<float>(elapsedUs));
}

}  // namespace

bool IrSensor::setupPcnt(int pin) {
  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = pin;
  cfg.ctrl_gpio_num = PCNT_PIN_NOT_USED;
  cfg.channel = PCNT_CHANNEL_0;
  cfg.unit = kIrPcntUnit;
  cfg.pos_mode = PCNT_COUNT_INC;
  cfg.neg_mode = PCNT_COUNT_DIS;
  cfg.lctrl_mode = PCNT_MODE_KEEP;
  cfg.hctrl_mode = PCNT_MODE_KEEP;
  cfg.counter_h_lim = kPcntHighLimit;
  cfg.counter_l_lim = 0;

  if (pcnt_unit_config(&cfg) != ESP_OK) {
    Serial.printf("[IR] PCNT config failed on GPIO %d\n", pin);
    return false;
  }

  pcnt_set_filter_value(kIrPcntUnit, kPcntFilterApbCycles);
  pcnt_filter_enable(kIrPcntUnit);
  pcnt_counter_pause(kIrPcntUnit);
  pcnt_counter_clear(kIrPcntUnit);
  pcnt_counter_resume(kIrPcntUnit);
  return true;
}

uint32_t IrSensor::readAndClearPcnt() {
  int16_t count = 0;
  pcnt_get_counter_value(kIrPcntUnit, &count);
  pcnt_counter_clear(kIrPcntUnit);
  if (count < 0) {
    count = 0;
  }
  return static_cast<uint32_t>(count);
}

void IrSensor::begin(const IrConfig& config) {
  config_ = config;
  state_ = IrState{};
  enabled_ = false;
  pcntReady_ = false;

  pinMode(config_.switchPin, INPUT_PULLUP);
  pinMode(config_.sensePin, INPUT);

  pcntReady_ = setupPcnt(config_.sensePin);
  if (!pcntReady_) {
    Serial.println(F("[IR] PCNT init failed — frequency will be zero"));
  } else {
    Serial.println(F("[IR] PCNT ready"));
  }

  gateStartUs_ = micros();
}

bool IrSensor::update(IrState& state) {
  if (!enabled_) {
    return false;
  }

  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - gateStartUs_;
  if (elapsedUs < config_.gateTimeMs * 1000UL) {
    return false;
  }

  const uint32_t counts = pcntReady_ ? readAndClearPcnt() : 0;
  state_.hz = countsToHz(counts, elapsedUs);
  state_.switchHigh = digitalRead(config_.switchPin) == HIGH;
  // HIGH → low band (0–4 kHz); LOW → high band (6–12 kHz).
  state_.band = state_.switchHigh ? IrBand::Low : IrBand::High;

  if (state_.band == IrBand::Low) {
    state_.beaconDetected =
        state_.hz >= config_.lowBandMinHz && state_.hz <= config_.lowBandMaxHz;
  } else {
    state_.beaconDetected = state_.hz >= config_.highBandMinHz &&
                            state_.hz <= config_.highBandMaxHz;
  }

  gateStartUs_ = nowUs;
  state = state_;
  return true;
}
