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
constexpr int kLeftMotorPwm0Pin = 3;
constexpr int kLeftMotorPwm1Pin = 4;
constexpr int kRightMotorPwm0Pin = 5;
constexpr int kRightMotorPwm1Pin = 6;

// IR detector — GPIO 7 (ADC1_CH6)
constexpr int kIrDetectorPin = 7;

// Spare ADC1 inputs — GPIO 8 (ADC1_CH7), GPIO 9 (ADC1_CH8). Unassigned.

// Pi ↔ ESP handshake (see src/sensors/pi-handshake.md when present).
// START out (ESP→Pi rising edge): GPIO 10
// DETECT in (Pi→ESP ~100 ms pulse): GPIO 11
constexpr int kPiStartPin = 10;
constexpr int kPiDetectPin = 11;

// Metal detectors — GPIO 13, 14
constexpr int kMetalDetectorLeftPin = 13;
constexpr int kMetalDetectorRightPin = 14;

// Switches — GPIO 15, 41
constexpr int kSwitch1Pin = 15;
constexpr int kSwitch0Pin = 41;

// Status LEDs — GPIO 16, 17
constexpr int kLedLeftPin = 16;
constexpr int kLedRightPin = 17;

// IMU (MPU6050) — dedicated I2C bus (Wire), separate from OLED.
// Uses freed GPIO 12 + spare GPIO 18 so it never shares reflectance or OLED pins.
constexpr int kImuSdaPin = 8;
constexpr int kImuSclPin = 9;

// Stepper motors (DIR + STEP each): rotation, vertical, horizontal
constexpr int kRotationDirPin = 21;
constexpr int kRotationStepPin = 40;
constexpr int kVerticalDirPin = 39;
constexpr int kVerticalStepPin = 38;
constexpr int kHorizontalDirPin = 47;
constexpr int kHorizontalStepPin = 48;

// Claw servo — GPIO 42
constexpr int kServoMotorPin = 42;

// OLED display — I2C (SSD1306) on Wire1 / second bus pins.
constexpr int kOledSclPin = 45;
constexpr int kOledSdaPin = 46;

// Spare ADC1: GPIO 8, 9. Do not reuse any GPIO listed above.
