#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const int trigPin = 15;
const int echoPin = 16;

#define OLED_SCL 10
#define OLED_SDA 11
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

#define SOUND_SPEED 0.034f
#define CM_TO_INCH 0.393701f
#define MIN_DISTANCE_CM 2.0f
#define MAX_DISTANCE_CM 30.0f
#define PULSE_TIMEOUT_US 30000

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

float readDistanceCm() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, PULSE_TIMEOUT_US);
  if (duration == 0) {
    return -1.0f;
  }

  float distanceCm = duration * SOUND_SPEED / 2.0f;
  if (distanceCm < MIN_DISTANCE_CM || distanceCm > MAX_DISTANCE_CM) {
    return -1.0f;
  }

  return distanceCm;
}

void showDistance(float distanceCm) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("HC-SR04"));

  if (distanceCm < 0.0f) {
    display.setTextSize(2);
    display.setCursor(0, 20);
    display.println(F("---"));
    display.setTextSize(1);
    display.setCursor(0, 48);
    display.println(F("No object"));
    return;
  }

  display.setTextSize(3);
  display.setCursor(0, 18);
  display.printf("%.1f", distanceCm);
  display.setTextSize(2);
  display.print(F(" cm"));

  display.setTextSize(1);
  display.setCursor(0, 52);
  display.printf("%.1f in", distanceCm * CM_TO_INCH);
}

void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;) {}
  }

  showDistance(-1.0f);
  display.display();
}

void loop() {
  float distanceCm = readDistanceCm();
  showDistance(distanceCm);
  display.display();

  if (distanceCm < 0.0f) {
    Serial.println(F("Distance: out of range"));
  } else {
    Serial.printf("Distance: %.1f cm (%.1f in)\n", distanceCm, distanceCm * CM_TO_INCH);
  }

  delay(100);
}
