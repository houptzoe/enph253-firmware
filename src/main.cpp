#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_SCL 16
#define OLED_SDA 15
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#ifndef PWM_L0_PIN
#define PWM_L0_PIN 17
#endif
#ifndef PWM_L1_PIN
#define PWM_L1_PIN 5
#endif
#ifndef PWM_R0_PIN
#define PWM_R0_PIN 7
#endif
#ifndef PWM_R1_PIN
#define PWM_R1_PIN 8
#endif
#ifndef BATTERY_VOLTAGE
#define BATTERY_VOLTAGE 9.0f
#endif
#ifndef MOTOR_MAX_VOLTAGE
#define MOTOR_MAX_VOLTAGE 6.0f
#endif

static const int PWM_L0_CHANNEL = 0;
static const int PWM_L1_CHANNEL = 1;
static const int PWM_R0_CHANNEL = 2;
static const int PWM_R1_CHANNEL = 3;
static const uint32_t MOTOR_SWITCH_DEADTIME_MS = 10;
static const uint32_t MOTOR_PWM_FREQ_HZ = 200;
static const uint8_t MOTOR_PWM_RESOLUTION_BITS = 10;
static const uint32_t MOTOR_PWM_MAX =
    (1u << MOTOR_PWM_RESOLUTION_BITS) - 1u;
static const uint32_t MOTOR_MAX_DUTY = static_cast<uint32_t>(
    (MOTOR_MAX_VOLTAGE / BATTERY_VOLTAGE) * static_cast<float>(MOTOR_PWM_MAX));

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static int motorLDirection = 0;  // -1 = reverse, 0 = stopped, +1 = forward
static int motorRDirection = 0;

void motorBridgeAllOff(int channel0, int channel1) {
  ledcWrite(channel0, 0);
  ledcWrite(channel1, 0);
}

void motorBridgeInit(int pin0, int pin1, int channel0, int channel1) {
  ledcSetup(channel0, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
  ledcSetup(channel1, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
  ledcAttachPin(pin0, channel0);
  ledcAttachPin(pin1, channel1);
  motorBridgeAllOff(channel0, channel1);
}

// speedPercent: -100 (reverse) .. 0 (coast) .. +100 (forward).
void motorBridgeSetSpeed(int channel0, int channel1, int &direction,
                         int speedPercent) {
  speedPercent = constrain(speedPercent, -100, 100);

  if (speedPercent == 0) {
    motorBridgeAllOff(channel0, channel1);
    direction = 0;
    return;
  }

  const int newDirection = speedPercent > 0 ? 1 : -1;
  const uint32_t duty =
      static_cast<uint32_t>((abs(speedPercent) / 100.0f) * MOTOR_MAX_DUTY);

  if (newDirection == direction) {
    if (newDirection > 0) {
      ledcWrite(channel1, 0);
      ledcWrite(channel0, duty);
    } else {
      ledcWrite(channel0, 0);
      ledcWrite(channel1, duty);
    }
    return;
  }

  const int prevDirection = direction;
  motorBridgeAllOff(channel0, channel1);

  if (prevDirection != 0 && newDirection != prevDirection) {
    delay(MOTOR_SWITCH_DEADTIME_MS);
  }

  direction = newDirection;

  if (newDirection > 0) {
    ledcWrite(channel1, 0);
    ledcWrite(channel0, duty);
  } else {
    ledcWrite(channel0, 0);
    ledcWrite(channel1, duty);
  }
}

void motorLeftStop() {
  motorBridgeAllOff(PWM_L0_CHANNEL, PWM_L1_CHANNEL);
  motorLDirection = 0;
}

void motorRightStop() {
  motorBridgeAllOff(PWM_R0_CHANNEL, PWM_R1_CHANNEL);
  motorRDirection = 0;
}

void motorLeftInit() {
  motorBridgeInit(PWM_L0_PIN, PWM_L1_PIN, PWM_L0_CHANNEL, PWM_L1_CHANNEL);
  motorLeftStop();
}

void motorRightInit() {
  motorBridgeInit(PWM_R0_PIN, PWM_R1_PIN, PWM_R0_CHANNEL, PWM_R1_CHANNEL);
  motorRightStop();
}

void motorLeftSetSpeed(int speedPercent) {
  motorBridgeSetSpeed(PWM_L0_CHANNEL, PWM_L1_CHANNEL, motorLDirection, speedPercent);
}

void motorRightSetSpeed(int speedPercent) {
  motorBridgeSetSpeed(PWM_R0_CHANNEL, PWM_R1_CHANNEL, motorRDirection, speedPercent);
}

void showMotorLeftStatus(int speedPercent) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("Left Motor"));

  display.setTextSize(2);
  display.setCursor(0, 20);
  if (speedPercent > 0) {
    display.printf("FWD %d%%", speedPercent);
  } else if (speedPercent < 0) {
    display.printf("REV %d%%", -speedPercent);
  } else {
    display.println(F("STOP"));
  }

  display.setTextSize(1);
  display.setCursor(0, 52);
  display.printf("L0/L1 max %.1fV", MOTOR_MAX_VOLTAGE);
}

void showMotorRightStatus(int speedPercent) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("Right Motor"));

  display.setTextSize(2);
  display.setCursor(0, 20);
  if (speedPercent > 0) {
    display.printf("FWD %d%%", speedPercent);
  } else if (speedPercent < 0) {
    display.printf("REV %d%%", -speedPercent);
  } else {
    display.println(F("STOP"));
  }

  display.setTextSize(1);
  display.setCursor(0, 52);
  display.printf("R0/R1 max %.1fV", MOTOR_MAX_VOLTAGE);
}

void rampMotorLeftSpeed(int fromPercent, int toPercent, int stepPercent, int stepMs) {
  const int direction = (toPercent >= fromPercent) ? 1 : -1;
  for (int speed = fromPercent; direction > 0 ? speed <= toPercent : speed >= toPercent;
       speed += stepPercent * direction) {
    motorLeftSetSpeed(speed);
    showMotorLeftStatus(speed);
    display.display();
    Serial.printf("Left motor: %d%% (duty %lu / %lu)\n", speed,
                  static_cast<unsigned long>(
                      (abs(speed) / 100.0f) * MOTOR_MAX_DUTY),
                  static_cast<unsigned long>(MOTOR_MAX_DUTY));
    delay(stepMs);
  }
}

void rampMotorRightSpeed(int fromPercent, int toPercent, int stepPercent, int stepMs) {
  const int direction = (toPercent >= fromPercent) ? 1 : -1;
  for (int speed = fromPercent; direction > 0 ? speed <= toPercent : speed >= toPercent;
       speed += stepPercent * direction) {
    motorRightSetSpeed(speed);
    showMotorRightStatus(speed);
    display.display();
    Serial.printf("Right motor: %d%% (duty %lu / %lu)\n", speed,
                  static_cast<unsigned long>(
                      (abs(speed) / 100.0f) * MOTOR_MAX_DUTY),
                  static_cast<unsigned long>(MOTOR_MAX_DUTY));
    delay(stepMs);
  }
}

void setup() {
  Serial.begin(115200);
  motorLeftInit();
  motorRightInit();

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;) {}
  }

  Serial.println(F("Drivetrain PWM ready"));
  Serial.printf("pwmL0=%d pwmL1=%d pwmR0=%d pwmR1=%d\n", PWM_L0_PIN, PWM_L1_PIN,
                PWM_R0_PIN, PWM_R1_PIN);
  Serial.printf("Max duty capped to %.0f%% (~%.1f V effective)\n",
                (MOTOR_MAX_DUTY * 100.0f) / MOTOR_PWM_MAX, MOTOR_MAX_VOLTAGE);
  showMotorLeftStatus(0);
  display.display();
}

void loop() {
  motorLeftSetSpeed(100);
  showMotorLeftStatus(100);
  display.display();
  delay(1000);
}
