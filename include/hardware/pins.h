#pragma once

// GPIO assignments for the ESP32-S3 DevKitM-1 (see platformio.ini).

// Reflectance sensors — ADC1 pins (GPIO 1–10); avoid ADC2 when Wi-Fi is on.
constexpr int kLeftReflectancePin = 4;
constexpr int kRightReflectancePin = 5;

// Drive motors — PWM speed + direction for each H-bridge channel.
constexpr int kLeftMotorPwmPin = 6;
constexpr int kRightMotorPwmPin = 7;
constexpr int kLeftMotorDirPin = 8;
constexpr int kRightMotorDirPin = 9;

// OLED display — I2C (SSD1306).
constexpr int kOledSclPin = 10;
constexpr int kOledSdaPin = 11;
