#pragma once

// GPIO assignments for the ESP32-S3 DevKitM-1 (see platformio.ini).
// Every assigned GPIO appears exactly once — see uniqueness check below.

// Reflectance sensors — ADC1 (GPIO 1–10); avoid ADC2 when Wi-Fi is on.
// Left: GPIO 1 (ADC1_CH0), right: GPIO 2 (ADC1_CH1).
constexpr int kLeftReflectancePin = 1;
constexpr int kRightReflectancePin = 2;

// Drive motors — two-wire H-bridge: PWM the active direction pin per side.
// Left:  pwmL0 forward (GPIO 3), pwmL1 reverse (GPIO 4)
// Right: pwmR0 forward (GPIO 5), pwmR1 reverse (GPIO 6)
constexpr int kLeftMotorPwm0Pin = 5;
constexpr int kLeftMotorPwm1Pin = 6;
constexpr int kRightMotorPwm0Pin = 3;
constexpr int kRightMotorPwm1Pin = 4;

// IR detector — GPIO 7 (ADC1_CH6)
constexpr int kIrDetectorPin = 7;
constexpr int kIrSwitchPin = 18;

// Sonar sensor — GPIO 8, 9
constexpr int kSonarTrig = 8;
constexpr int kSonarEcho = 9;

// Spare ADC1 inputs — GPIO 8 (ADC1_CH7), GPIO 9 (ADC1_CH8). Unassigned.

// Pi ↔ ESP dual-cam handshake (see lib/ESP32-GPIO-HANDSHAKE.md).
// ESP GPIO 11 → Pi BCM GPIO3 (physical 5): DETECT_CAM0
// ESP GPIO 10 → Pi BCM GPIO4 (physical 7): START out + DETECT_CAM1 (multiplexed)
constexpr int kPiCam1StartPin = 10;
constexpr int kPiCam0Pin = 11;

// Metal detectors — GPIO 13, 14
constexpr int kMetalDetectorLeftPin = 13;
constexpr int kMetalDetectorRightPin = 14;

// Status LEDs — GPIO 16, 17
constexpr int kLedLeftPin = 16;
constexpr int kLedRightPin = 17;

// Switches — GPIO 15, 41
constexpr int kSwitch0Pin = 41;

// IMU (MPU6050) — dedicated I2C bus (Wire), separate from OLED.
// Uses freed GPIO 12 + spare GPIO 18 so it never shares reflectance or OLED pins.
constexpr int kImuSdaPin = 12;
constexpr int kImuSclPin = 15;

// Stepper motors (DIR + STEP each): rotation, vertical, horizontal
constexpr int kRotationDirPin = 21;
constexpr int kVerticalStepPin = 38;
constexpr int kVerticalDirPin = 39;
constexpr int kRotationStepPin = 40;
constexpr int kHorizontalDirPin = 47;
constexpr int kHorizontalStepPin = 48;

// Claw servo — GPIO 42
constexpr int kServoMotorPin = 42;

// OLED display — I2C (SSD1306) on Wire1 / second bus pins.
constexpr int kOledSclPin = 45;
constexpr int kOledSdaPin = 46;