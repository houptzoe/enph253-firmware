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

void ReflectanceDisplay::showReadings(int leftAvg, int rightAvg, bool leftOnTape,
                                      bool rightOnTape) {
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

void ReflectanceDisplay::showMessage(const char* line1, const char* line2) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  display.setCursor(0, 20);
  display.println(line2);
  display.display();
}
