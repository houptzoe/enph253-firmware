#pragma once

// GPIO assignments for the ESP32-S3 DevKitM-1 (see platformio.ini).

// Reflectance sensors — ADC1 (GPIO 1–10); avoid ADC2 when Wi-Fi is on.
// GPIO 9 = ADC1_CH8, GPIO 10 = ADC1_CH9. Kept off motor PWM pins (4/5/7/8).
constexpr int kLeftReflectancePin = 9;
constexpr int kRightReflectancePin = 10;

// Drive motors — two-wire H-bridge: PWM the active direction pin per side.
constexpr int kLeftMotorPwm0Pin = 4;
constexpr int kLeftMotorPwm1Pin = 5;
constexpr int kRightMotorPwm0Pin = 7;
constexpr int kRightMotorPwm1Pin = 8;

// OLED display — I2C (SSD1306).
constexpr int kOledSclPin = 45;
constexpr int kOledSdaPin = 46;
