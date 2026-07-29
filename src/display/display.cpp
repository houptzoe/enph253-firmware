#include "display/display.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

#include "hardware/pins.h"

namespace {

constexpr int kScreenWidth = 128;
constexpr int kScreenHeight = 64;
constexpr int kOledResetPin = -1;  // not used on I2C modules
constexpr uint8_t kOledI2cAddress = 0x3C;

Adafruit_SSD1306 display(kScreenWidth, kScreenHeight, &Wire, kOledResetPin);
bool gOledReady = false;

bool ensureOled() {
  if (gOledReady) {
    return true;
  }

  Wire.begin(kOledSdaPin, kOledSclPin);

  if (!display.begin(SSD1306_SWITCHCAPVCC, kOledI2cAddress)) {
    Serial.println("OLED init failed");
    return false;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();
  gOledReady = true;
  return true;
}

}  // namespace

void ReflectanceDisplay::begin() { ensureOled(); }

void ReflectanceDisplay::showReadings(int leftAvg, int rightAvg, bool leftOnTape,
                                      bool rightOnTape) {
  if (!gOledReady) {
    return;
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastUpdateMs_ < kMinUpdateMs) {
    return;
  }
  lastUpdateMs_ = nowMs;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println(F("Reflectance"));

  display.setCursor(0, 16);
  display.printf("L analog: %4d", leftAvg);
  display.setCursor(0, 28);
  display.printf("L digital: %s", leftOnTape ? "ON " : "OFF");

  display.setCursor(0, 44);
  display.printf("R analog: %4d", rightAvg);
  display.setCursor(0, 56);
  display.printf("R digital: %s", rightOnTape ? "ON " : "OFF");

  display.display();
}

void MetalDisplay::begin() {
  ensureOled();
  forceNext_ = true;
}

void MetalDisplay::show(float baselineLeftHz, float baselineRightHz,
                        float liveLeftHz, float liveRightHz,
                        const char* status) {
  if (!gOledReady) {
    return;
  }

  const uint32_t nowMs = millis();
  if (!forceNext_ && (nowMs - lastUpdateMs_ < kMinUpdateMs)) {
    return;
  }
  forceNext_ = false;
  lastUpdateMs_ = nowMs;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println(F("Metal detect"));

  display.setCursor(0, 12);
  display.printf("Base L: %4.0f Hz", baselineLeftHz);
  display.setCursor(0, 22);
  display.printf("Base R: %4.0f Hz", baselineRightHz);

  display.setCursor(0, 34);
  display.printf("Live L: %4.0f Hz", liveLeftHz);
  display.setCursor(0, 44);
  display.printf("Live R: %4.0f Hz", liveRightHz);

  display.setCursor(0, 56);
  display.printf("Status: %s", status != nullptr ? status : "?");

  display.display();
}
