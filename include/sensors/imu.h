#pragma once

#include <Arduino.h>
#include <Wire.h>

// MPU6050 pose estimator: Madgwick orientation + simple XY dead reckoning.
// Yaw drifts without a magnetometer; position drifts without external aids.

struct ImuPose {
  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;
  float yawDeg = 0.0f;
  float xM = 0.0f;
  float yM = 0.0f;
  float zM = 0.0f;
  float vxMs = 0.0f;
  float vyMs = 0.0f;
  bool zuptHeld = false;  // velocity is being pinned to zero right now
};

class ImuTracker {
 public:
  // imuWire: Wire or Wire1 already begun on the IMU I2C pins.
  bool begin(TwoWire& imuWire, uint8_t i2cAddress = 0x68);

  // Call as often as possible. Returns true when a new sample was fused.
  bool update(ImuPose& pose);

  // Zero yaw and XY at the current pose (useful after holding still).
  void resetPose();

 private:
  static constexpr float kGravity = 9.80665f;
  // Stillness is judged on raw specific force, which reads 1 g at rest no
  // matter the orientation or bias. Testing the bias-corrected residual
  // instead deadlocks: bias is only learned while still.
  static constexpr float kStillAccelDev = 0.25f;    // m/s^2 away from 1 g
  static constexpr float kGyroStillThresh = 0.08f;  // rad/s
  static constexpr uint32_t kStillMsForZupt = 250;
  // Nothing observes velocity, so cap it at a sane physical speed to keep a
  // bad estimate from ramping away without bound.
  static constexpr float kMaxSpeed = 2.5f;  // m/s
  // Accel only tells us "down" when it reads ~1 g; past this deviation it is
  // measuring motion, so it must not be allowed to tilt the attitude estimate.
  static constexpr float kAccelRejectDev = 2.0f;  // m/s^2
  static constexpr float kBiasLearnAlpha = 0.02f;

  bool calibrateGyro(uint16_t samples);
  float yawDegFromQuat() const;
  void fuseOrientation(float ax, float ay, float az, float gx, float gy,
                       float gz, float dt);
  void integratePosition(float ax, float ay, float az, float gx, float gy,
                         float gz, float dt);

  uint8_t address_ = 0x68;
  bool ready_ = false;

  float gyroBiasX_ = 0.0f;
  float gyroBiasY_ = 0.0f;
  float gyroBiasZ_ = 0.0f;

  // Madgwick quaternion (w, x, y, z)
  float q0_ = 1.0f;
  float q1_ = 0.0f;
  float q2_ = 0.0f;
  float q3_ = 0.0f;
  float beta_ = 0.1f;

  // World-frame residual acceleration, relearned whenever we sit still.
  float biasWx_ = 0.0f;
  float biasWy_ = 0.0f;
  float biasWz_ = 0.0f;

  float yawOffsetDeg_ = 0.0f;
  float velX_ = 0.0f;
  float velY_ = 0.0f;
  float velZ_ = 0.0f;
  float posX_ = 0.0f;
  float posY_ = 0.0f;
  float posZ_ = 0.0f;

  uint32_t lastUpdateUs_ = 0;
  uint32_t stillSinceMs_ = 0;
  bool wasStill_ = false;
  bool zuptHeld_ = false;
};
