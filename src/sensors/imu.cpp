#include "sensors/imu.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

namespace {

Adafruit_MPU6050 gMpu;

float invSqrt(float x) { return 1.0f / sqrtf(x); }

float wrapDeg180(float deg) {
  while (deg > 180.0f) {
    deg -= 360.0f;
  }
  while (deg < -180.0f) {
    deg += 360.0f;
  }
  return deg;
}

}  // namespace

bool ImuTracker::begin(TwoWire& imuWire, uint8_t i2cAddress) {
  address_ = i2cAddress;

  if (!gMpu.begin(address_, &imuWire)) {
    Serial.println(F("[IMU] MPU6050 not found"));
    ready_ = false;
    return false;
  }

  gMpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  gMpu.setGyroRange(MPU6050_RANGE_500_DEG);
  gMpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println(F("[IMU] Keep still for gyro calibration..."));
  if (!calibrateGyro(400)) {
    Serial.println(F("[IMU] Gyro calibration failed"));
    ready_ = false;
    return false;
  }
  Serial.printf("[IMU] Gyro bias: %.4f %.4f %.4f rad/s\n", gyroBiasX_,
                gyroBiasY_, gyroBiasZ_);

  q0_ = 1.0f;
  q1_ = q2_ = q3_ = 0.0f;
  yawOffsetDeg_ = 0.0f;
  resetPose();
  lastUpdateUs_ = micros();
  ready_ = true;
  return true;
}

bool ImuTracker::calibrateGyro(uint16_t samples) {
  double sumX = 0.0;
  double sumY = 0.0;
  double sumZ = 0.0;

  for (uint16_t i = 0; i < samples; ++i) {
    sensors_event_t a, g, temp;
    gMpu.getEvent(&a, &g, &temp);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(5);
  }

  gyroBiasX_ = static_cast<float>(sumX / samples);
  gyroBiasY_ = static_cast<float>(sumY / samples);
  gyroBiasZ_ = static_cast<float>(sumZ / samples);
  return true;
}

float ImuTracker::yawDegFromQuat() const {
  const float yaw =
      atan2f(q1_ * q2_ + q0_ * q3_, 0.5f - q2_ * q2_ - q3_ * q3_);
  return yaw * 180.0f / PI;
}

void ImuTracker::resetPose() {
  // Make reported yaw 0 at the current orientation (safe to call more than once).
  yawOffsetDeg_ = yawDegFromQuat();
  velX_ = velY_ = velZ_ = 0.0f;
  posX_ = posY_ = posZ_ = 0.0f;
}

bool ImuTracker::update(ImuPose& pose) {
  if (!ready_) {
    return false;
  }

  sensors_event_t a, g, temp;
  gMpu.getEvent(&a, &g, &temp);

  const uint32_t nowUs = micros();
  float dt = (nowUs - lastUpdateUs_) * 1e-6f;
  lastUpdateUs_ = nowUs;
  if (dt <= 0.0f || dt > 0.1f) {
    dt = 0.01f;
  }

  const float gx = g.gyro.x - gyroBiasX_;
  const float gy = g.gyro.y - gyroBiasY_;
  const float gz = g.gyro.z - gyroBiasZ_;
  const float ax = a.acceleration.x;
  const float ay = a.acceleration.y;
  const float az = a.acceleration.z;

  fuseOrientation(ax, ay, az, gx, gy, gz, dt);
  integratePosition(ax, ay, az, gx, gy, gz, dt);

  const float roll =
      atan2f(q0_ * q1_ + q2_ * q3_, 0.5f - q1_ * q1_ - q2_ * q2_);
  const float pitch = asinf(-2.0f * (q1_ * q3_ - q0_ * q2_));
  const float yaw =
      atan2f(q1_ * q2_ + q0_ * q3_, 0.5f - q2_ * q2_ - q3_ * q3_);

  pose.rollDeg = roll * 180.0f / PI;
  pose.pitchDeg = pitch * 180.0f / PI;
  pose.yawDeg = wrapDeg180(yaw * 180.0f / PI - yawOffsetDeg_);

  // Position is integrated in the raw Madgwick world frame, so rotate it into
  // the same yaw-zeroed frame the heading is reported in.
  const float offRad = yawOffsetDeg_ * PI / 180.0f;
  const float offC = cosf(offRad);
  const float offS = sinf(offRad);
  pose.xM = posX_ * offC + posY_ * offS;
  pose.yM = -posX_ * offS + posY_ * offC;
  pose.zM = posZ_;
  pose.vxMs = velX_ * offC + velY_ * offS;
  pose.vyMs = -velX_ * offS + velY_ * offC;
  pose.zuptHeld = zuptHeld_;
  return true;
}

void ImuTracker::fuseOrientation(float ax, float ay, float az, float gx,
                                 float gy, float gz, float dt) {
  // Madgwick IMU update (gyro in rad/s, accel in m/s^2).
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2,
      q3q3;

  qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
  qDot2 = 0.5f * (q0_ * gx + q2_ * gz - q3_ * gy);
  qDot3 = 0.5f * (q0_ * gy - q1_ * gz + q3_ * gx);
  qDot4 = 0.5f * (q0_ * gz + q1_ * gy - q2_ * gx);

  // Fade out the gravity correction while the sensor is being accelerated,
  // otherwise translation gets mistaken for tilt and leaks straight into XY.
  const float accelMag = sqrtf(ax * ax + ay * ay + az * az);
  const float dev = fabsf(accelMag - kGravity);
  const float betaEff =
      beta_ * constrain(1.0f - dev / kAccelRejectDev, 0.0f, 1.0f);

  if (betaEff > 0.0f && !((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    _2q0 = 2.0f * q0_;
    _2q1 = 2.0f * q1_;
    _2q2 = 2.0f * q2_;
    _2q3 = 2.0f * q3_;
    _4q0 = 4.0f * q0_;
    _4q1 = 4.0f * q1_;
    _4q2 = 4.0f * q2_;
    _8q1 = 8.0f * q1_;
    _8q2 = 8.0f * q2_;
    q0q0 = q0_ * q0_;
    q1q1 = q1_ * q1_;
    q2q2 = q2_ * q2_;
    q3q3 = q3_ * q3_;

    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1_ - _2q0 * ay - _4q1 +
         _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2_ + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 +
         _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3_ - _2q1 * ax + 4.0f * q2q2 * q3_ - _2q2 * ay;

    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    qDot1 -= betaEff * s0;
    qDot2 -= betaEff * s1;
    qDot3 -= betaEff * s2;
    qDot4 -= betaEff * s3;
  }

  q0_ += qDot1 * dt;
  q1_ += qDot2 * dt;
  q2_ += qDot3 * dt;
  q3_ += qDot4 * dt;

  recipNorm = invSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  q0_ *= recipNorm;
  q1_ *= recipNorm;
  q2_ *= recipNorm;
  q3_ *= recipNorm;
}

void ImuTracker::integratePosition(float ax, float ay, float az, float gx,
                                   float gy, float gz, float dt) {
  // Rotate body-frame acceleration into world frame: v' = q * v * q^-1
  const float qw = q0_;
  const float qx = q1_;
  const float qy = q2_;
  const float qz = q3_;

  const float tw = -qx * ax - qy * ay - qz * az;
  const float tx = qw * ax + qy * az - qz * ay;
  const float ty = qw * ay + qz * ax - qx * az;
  const float tz = qw * az + qx * ay - qy * ax;

  const float wx = -tw * qx + tx * qw - ty * qz + tz * qy;
  const float wy = -tw * qy + tx * qz + ty * qw - tz * qx;
  const float wz = -tw * qz - tx * qy + ty * qx + tz * qw;

  // Subtract gravity (world Z up) and the learned residual bias.
  const float rawX = wx;
  const float rawY = wy;
  const float rawZ = wz - kGravity;

  float linX = rawX - biasWx_;
  float linY = rawY - biasWy_;
  float linZ = rawZ - biasWz_;

  const float specificForce = sqrtf(ax * ax + ay * ay + az * az);
  const float gyroMag = sqrtf(gx * gx + gy * gy + gz * gz);
  const bool still = (fabsf(specificForce - kGravity) < kStillAccelDev) &&
                     (gyroMag < kGyroStillThresh);

  const uint32_t nowMs = millis();
  zuptHeld_ = false;
  if (still) {
    if (!wasStill_) {
      stillSinceMs_ = nowMs;
      wasStill_ = true;
    }
    if (nowMs - stillSinceMs_ >= kStillMsForZupt) {
      zuptHeld_ = true;
      velX_ = velY_ = velZ_ = 0.0f;
      // Anything left over while parked is bias, not motion.
      biasWx_ += (rawX - biasWx_) * kBiasLearnAlpha;
      biasWy_ += (rawY - biasWy_) * kBiasLearnAlpha;
      biasWz_ += (rawZ - biasWz_) * kBiasLearnAlpha;
      linX = linY = linZ = 0.0f;
    }
  } else {
    wasStill_ = false;
  }

  // No velocity damping here on purpose. Decaying velocity mid-trip unbalances
  // the acceleration and deceleration impulses, and the leftover reverse
  // velocity walks the position backwards. Only a ZUPT may zero velocity.
  velX_ += linX * dt;
  velY_ += linY * dt;
  velZ_ += linZ * dt;

  velX_ = constrain(velX_, -kMaxSpeed, kMaxSpeed);
  velY_ = constrain(velY_, -kMaxSpeed, kMaxSpeed);
  velZ_ = constrain(velZ_, -kMaxSpeed, kMaxSpeed);

  posX_ += velX_ * dt;
  posY_ += velY_ * dt;
  posZ_ += velZ_ * dt;
}
