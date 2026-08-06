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
                                    int leftAnalog, int rightAnalog,
                                    float distanceCm, bool distanceValid,
                                    float turnedDeg, bool irEnabled,
                                    float irHz, bool irHighBand) {
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
  display.setCursor(0, 8);
  display.printf("Freq R:%.0f", rightHz);

  display.setCursor(0, 16);
  display.printf("Base L:%.0f R:%.0f", baselineLeft, baselineRight);

  display.setCursor(0, 24);
  display.print(F("Metal: "));
  if (!irEnabled && (metalLeftHit || metalRightHit)) {
    if (metalLeftHit && metalRightHit) {
      display.print(F("L R"));
    } else if (metalLeftHit) {
      display.print(F("L"));
    } else {
      display.print(F("R"));
    }
  } else if (irEnabled) {
    display.print(F("off"));
  } else {
    display.print(F("-"));
  }

  display.setCursor(0, 32);
  display.printf("Ref L:%4d R:%4d", leftAnalog, rightAnalog);

  display.setCursor(0, 40);
  if (irEnabled) {
    display.printf("IR:%.0fHz %s", irHz, irHighBand ? "Hi" : "Lo");
  } else if (distanceValid) {
    display.printf("Dist: %.1f cm", distanceCm);
  } else {
    display.print(F("Dist: ----"));
  }

  display.setCursor(0, 48);
  display.printf("Turn: %.1f deg", turnedDeg);

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

void ReflectanceDisplay::showSonarScan(float distanceCm, bool valid,
                                       float lockMinCm, float lockMaxCm) {
  const uint32_t now = millis();
  if (now - lastUpdateMs_ < kMinUpdateMs) {
    return;
  }
  lastUpdateMs_ = now;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.println("RotateScan");
  display.setCursor(0, 16);
  if (valid) {
    display.printf("Dist: %.1f cm", distanceCm);
  } else {
    display.println("Dist: ----");
  }
  display.setCursor(0, 32);
  display.printf("Lock: %.0f-%.0f cm", lockMinCm, lockMaxCm);

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
