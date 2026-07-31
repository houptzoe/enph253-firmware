#include <Arduino.h>
#include <Wire.h>

#include "display/imu_display.h"
#include "hardware/pins.h"
#include "sensors/imu.h"

// Bench test for MPU6050 + SSD1306 using the robot pin map in pins.h.
//
// Wiring:
//   MPU6050: SDA=kImuSdaPin (GPIO 12), SCL=kImuSclPin (GPIO 18)  — Wire
//   OLED:    SDA=kOledSdaPin (GPIO 46), SCL=kOledSclPin (GPIO 45) — Wire1

namespace {

ImuTracker imu;
ImuPoseDisplay oled;

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("[BOOT] IMU pose bench test"));

  Wire.begin(kImuSdaPin, kImuSclPin);
  Wire.setClock(400000);

  Wire1.begin(kOledSdaPin, kOledSclPin);
  Wire1.setClock(400000);

  oled.begin(Wire1);
  oled.showStatus("Calibrating IMU", "Keep still...");

  if (!imu.begin(Wire)) {
    oled.showStatus("IMU init failed", "Check wiring");
    while (true) {
      delay(1000);
    }
  }

  // Let Madgwick settle and the bias estimator converge, then zero the pose.
  oled.showStatus("Settling filter", "Hold still...");
  ImuPose pose;
  const uint32_t settleStart = millis();
  while (millis() - settleStart < 3000) {
    imu.update(pose);
    delay(5);
  }
  imu.resetPose();

  oled.showStatus("Ready", "Move the IMU");
  delay(500);
  Serial.println(F("[IMU] Ready — move the board around"));
  Serial.println(F("[IMU] Send 'z' over serial to zero pose"));
}

void loop() {
  ImuPose pose;
  if (imu.update(pose)) {
    oled.showPose(pose);

    static uint32_t lastLogMs = 0;
    const uint32_t nowMs = millis();
    if (nowMs - lastLogMs >= 200) {
      lastLogMs = nowMs;
      Serial.printf(
          "yaw:%6.1f  x:%6.2f  y:%6.2f  vx:%6.2f  vy:%6.2f  %s\n", pose.yawDeg,
          pose.xM, pose.yM, pose.vxMs, pose.vyMs, pose.zuptHeld ? "STILL" : "");
    }
  }

  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == 'z' || c == 'Z') {
      imu.resetPose();
      Serial.println(F("[IMU] Pose zeroed"));
    }
  }
}
