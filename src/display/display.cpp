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
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.display();
}

void ReflectanceDisplay::showReadings(int leftAvg, int rightAvg) {
  const uint32_t nowMs = millis();
  if (nowMs - lastUpdateMs_ < kMinUpdateMs) {
    return;
  }
  lastUpdateMs_ = nowMs;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println(F("Reflectance"));
  display.printf("L: %4d\n", leftAvg);
  display.printf("R: %4d", rightAvg);
  display.display();
}
