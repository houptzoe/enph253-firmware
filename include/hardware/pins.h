#pragma once

// GPIO assignments for the ESP32-S3 DevKitM-1 (see platformio.ini).

// Reflectance sensors — ADC1 (GPIO 1–10); avoid ADC2 when Wi-Fi is on.
// Left on GPIO 1 (ADC1_CH0), right on GPIO 2 (ADC1_CH1).
constexpr int kLeftReflectancePin = 1;
constexpr int kRightReflectancePin = 2;

// Drive motors — two-wire H-bridge: PWM the active direction pin per side.
// Left:  pwmL0 forward (GPIO 4), pwmL1 reverse (GPIO 5)
// Right: pwmR0 forward (GPIO 7), pwmR1 reverse (GPIO 8)
constexpr int kLeftMotorPwm0Pin = 4;
constexpr int kLeftMotorPwm1Pin = 5;
constexpr int kRightMotorPwm0Pin = 7;
constexpr int kRightMotorPwm1Pin = 8;

// OLED display — I2C (SSD1306).
constexpr int kOledSclPin = 42;
constexpr int kOledSdaPin = 45;
