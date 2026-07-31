#include "display/imu_display.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

namespace {

constexpr int kScreenWidth = 128;
constexpr int kScreenHeight = 64;
constexpr int kOledResetPin = -1;
constexpr uint8_t kOledI2cAddress = 0x3C;

constexpr int kMapLeft = 72;
constexpr int kMapTop = 8;
constexpr int kMapW = 56;
constexpr int kMapH = 56;

// Constructed in begin() against the OLED bus (Wire1 on the bench setup).
Adafruit_SSD1306* gDisplay = nullptr;

void drawHeadingGlyph(Adafruit_SSD1306& d, float yawDeg, float xM, float yM) {
  constexpr int cx = kMapLeft + kMapW / 2;
  constexpr int cy = kMapTop + kMapH / 2;

  d.drawRect(kMapLeft, kMapTop, kMapW, kMapH, SSD1306_WHITE);

  constexpr float scale = 40.0f;  // px per meter (~±0.5 m window)
  const int px = cx + static_cast<int>(constrain(xM * scale, -24.0f, 24.0f));
  const int py = cy - static_cast<int>(constrain(yM * scale, -24.0f, 24.0f));

  const float rad = yawDeg * PI / 180.0f;
  const float c = cosf(rad);
  const float s = sinf(rad);

  // Yaw is CCW-positive; the screen y-flip below is what makes it look CW.
  auto rot = [&](float lx, float ly, int& ox, int& oy) {
    const float wx = lx * c - ly * s;
    const float wy = lx * s + ly * c;
    ox = px + static_cast<int>(wx);
    oy = py - static_cast<int>(wy);
  };

  int x0, y0, x1, y1, x2, y2;
  rot(0.0f, 8.0f, x0, y0);    // nose (forward)
  rot(-5.0f, -5.0f, x1, y1);  // left rear
  rot(5.0f, -5.0f, x2, y2);   // right rear

  d.fillTriangle(x0, y0, x1, y1, x2, y2, SSD1306_WHITE);
  d.drawPixel(cx, cy, SSD1306_WHITE);
}

}  // namespace

void ImuPoseDisplay::begin(TwoWire& oledWire) {
  if (gDisplay == nullptr) {
    gDisplay =
        new Adafruit_SSD1306(kScreenWidth, kScreenHeight, &oledWire, kOledResetPin);
  }

  if (!gDisplay->begin(SSD1306_SWITCHCAPVCC, kOledI2cAddress)) {
    Serial.println(F("[OLED] init failed"));
    ready_ = false;
    return;
  }

  gDisplay->clearDisplay();
  gDisplay->setTextSize(1);
  gDisplay->setTextColor(SSD1306_WHITE);
  gDisplay->setCursor(0, 0);
  gDisplay->println(F("IMU pose test"));
  gDisplay->display();
  ready_ = true;
}

void ImuPoseDisplay::showStatus(const char* line1, const char* line2) {
  if (!ready_ || gDisplay == nullptr) {
    return;
  }
  gDisplay->clearDisplay();
  gDisplay->setTextSize(1);
  gDisplay->setTextColor(SSD1306_WHITE);
  gDisplay->setCursor(0, 16);
  gDisplay->println(line1);
  if (line2 != nullptr) {
    gDisplay->setCursor(0, 32);
    gDisplay->println(line2);
  }
  gDisplay->display();
}

void ImuPoseDisplay::showPose(const ImuPose& pose) {
  if (!ready_ || gDisplay == nullptr) {
    return;
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastUpdateMs_ < kMinUpdateMs) {
    return;
  }
  lastUpdateMs_ = nowMs;

  gDisplay->clearDisplay();
  gDisplay->setTextSize(1);
  gDisplay->setTextColor(SSD1306_WHITE);

  gDisplay->setCursor(0, 0);
  gDisplay->printf("Yaw %6.1f", pose.yawDeg);
  if (pose.zuptHeld) {
    gDisplay->setCursor(kMapLeft, 0);
    gDisplay->print(F("STILL"));
  }

  gDisplay->setCursor(0, 14);
  gDisplay->printf("X %6.2fm", pose.xM);
  gDisplay->setCursor(0, 26);
  gDisplay->printf("Y %6.2fm", pose.yM);

  gDisplay->setCursor(0, 42);
  gDisplay->printf("R %5.1f", pose.rollDeg);
  gDisplay->setCursor(0, 54);
  gDisplay->printf("P %5.1f", pose.pitchDeg);

  drawHeadingGlyph(*gDisplay, pose.yawDeg, pose.xM, pose.yM);
  gDisplay->display();
}
