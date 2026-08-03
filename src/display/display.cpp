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

}  // namespace

void ReflectanceDisplay::begin() {
  Wire.begin(kOledSdaPin, kOledSclPin);

  if (!display.begin(SSD1306_SWITCHCAPVCC, kOledI2cAddress)) {
    Serial.println("OLED init failed");
    return;
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();
}

void ReflectanceDisplay::showStatus(float leftHz, float rightHz,
                                    float baselineLeft, float baselineRight,
                                    bool metalLeftHit, bool metalRightHit,
                                    bool leftOnTape, bool rightOnTape) {
  const uint32_t nowMs = millis();
  if (nowMs - lastUpdateMs_ < kMinUpdateMs) {
    return;
  }
  lastUpdateMs_ = nowMs;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.printf("Freq L:%.0f", leftHz);
  display.setCursor(0, 10);
  display.printf("Freq R:%.0f", rightHz);

  display.setCursor(0, 22);
  display.printf("Base L:%.0f R:%.0f", baselineLeft, baselineRight);

  display.setCursor(0, 34);
  display.print(F("Metal: "));
  if (metalLeftHit && metalRightHit) {
    display.print(F("L R"));
  } else if (metalLeftHit) {
    display.print(F("L"));
  } else if (metalRightHit) {
    display.print(F("R"));
  } else {
    display.print(F("-"));
  }

  display.setCursor(0, 46);
  display.printf("Tape L:%s R:%s", leftOnTape ? "ON " : "OFF",
                 rightOnTape ? "ON" : "OFF");

  display.display();
}

void ReflectanceDisplay::showMetalHit(char side, float baselineHz, float deltaHz) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.printf("Metal hit: %c", side);
  display.setCursor(0, 12);
  display.printf("Baseline: %.0f Hz", baselineHz);
  display.setCursor(0, 24);
  display.printf("Delta: %.0f Hz", deltaHz);

  display.display();
}

void ReflectanceDisplay::showMessage(const char* line1, const char* line2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  display.setCursor(0, 12);
  display.println(line2);
  display.display();
}
