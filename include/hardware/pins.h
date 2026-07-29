#pragma once

// GPIO assignments for the ESP32-S3 DevKitM-1 (see platformio.ini).

// Reflectance sensors — ADC1 (GPIO 1–10); avoid ADC2 when Wi-Fi is on.
// Left on GPIO 1 (ADC1_CH0), right on GPIO 2 (ADC1_CH1).
constexpr int kLeftReflectancePin = 1;
constexpr int kRightReflectancePin = 2;

// Drive motors — two-wire H-bridge: PWM the active direction pin per side.
// Left:  pwmL0 forward (GPIO 4), pwmL1 reverse (GPIO 5)
// Right: pwmR0 forward (GPIO 7), pwmR1 reverse (GPIO 8) -- DOUBLE CHECK THIS
constexpr int kLeftMotorPwm0Pin = 3;
constexpr int kLeftMotorPwm1Pin = 4;
constexpr int kRightMotorPwm0Pin = 5;
constexpr int kRightMotorPwm1Pin = 6;

// OLED display — I2C (SSD1306).
constexpr int kOledSclPin = 45;
constexpr int kOledSdaPin = 46;

// IR detector - GPIO 7 (ADC1_CH6)
constexpr int kIrDetectorPin = 7;

// saved for future use - GPIO 8, 9 

// 10 (ADC1_CH7, ADC1_CH8, ADC1_CH9)

//pi comms - GPIO 11 & GPIO 12
constexpr int kPiCommsLeftPin = 11;
constexpr int kPiCommsRightPin = 12;

//metal detector - GPIO 13, 14
constexpr int kMetalDetectorLeftPin = 13; //empty
constexpr int kMetalDetectorRightPin = 14;

//servo motor - GPIO 42
constexpr int kServoMotorPin = 42;

//switches - GPIO 41, 15
constexpr int kSwitch0Pin = 41;
constexpr int kSwitch1Pin = 15;

//leds - GPIO 16, 17
constexpr int kLedLeftPin = 16;
constexpr int kLedRightPin = 17;

// GPIO 18 is empty

//stepper motors: rotation, vertical, horizontal
// each motor has 2 pins - direction and step
constexpr int kRotationDirPin = 21;
constexpr int kRotationStepPin = 40;
constexpr int kVerticalDirPin = 39;
constexpr int kVerticalStepPin = 38;
constexpr int kHorizontalDirPin = 47;
constexpr int kHorizontalStepPin = 48;

