#include "imu.h"
#include "config.h"
#include <Wire.h>
namespace {
bool ready = false;
float gyroBiasZ = 0.0f, gyroZ = 0.0f, yaw = 0.0f;
uint32_t lastSample = 0, lastMicros = 0;
constexpr uint8_t MPU_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU_GYRO_CONFIG = 0x1B;
constexpr uint8_t MPU_GYRO_XOUT_H = 0x43;
void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDRESS); Wire.write(reg); Wire.write(value); Wire.endTransmission();
}
int16_t readInt16() { return static_cast<int16_t>((Wire.read() << 8) | Wire.read()); }
void calibrateGyro() {
  int32_t total = 0;
  constexpr uint16_t samples = 300;
  for (uint16_t i = 0; i < samples; ++i) {
    Wire.beginTransmission(MPU6050_ADDRESS); Wire.write(MPU_GYRO_XOUT_H); Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, static_cast<uint8_t>(6));
    if (Wire.available() >= 6) { readInt16(); readInt16(); total += readInt16(); }
    delay(3);
  }
  gyroBiasZ = static_cast<float>(total) / samples;
}
}
void imuInit() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); Wire.setClock(400000);
  Wire.beginTransmission(MPU6050_ADDRESS); ready = Wire.endTransmission() == 0;
  if (!ready) { Serial.println("IMU ERROR: MPU6050 not found at 0x68"); return; }
  writeRegister(MPU_PWR_MGMT_1, 0x00); writeRegister(MPU_GYRO_CONFIG, 0x00);
  Serial.println("IMU CALIBRATING: keep robot still"); calibrateGyro();
  lastMicros = micros(); Serial.println("IMU READY");
}
void imuTask() {
  if (!ready) return;
  const uint32_t now = millis(); if (now - lastSample < CONTROL_PERIOD_MS) return; lastSample = now;
  Wire.beginTransmission(MPU6050_ADDRESS); Wire.write(MPU_GYRO_XOUT_H);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(MPU6050_ADDRESS, static_cast<uint8_t>(6)) != 6) return;
  readInt16(); readInt16(); const int16_t rawZ = readInt16();
  const uint32_t currentMicros = micros(); const float dt = (currentMicros - lastMicros) / 1000000.0f; lastMicros = currentMicros;
  gyroZ = (static_cast<float>(rawZ) - gyroBiasZ) / GYRO_Z_SCALE; yaw += gyroZ * dt;
}
bool imuIsReady() { return ready; }
float imuYawDegrees() { return yaw; }
float imuGyroZDegreesPerSecond() { return gyroZ; }
void imuResetYaw() { yaw = 0.0f; }
void imuPrintStatus() {
  Serial.print("IMU ready="); Serial.print(ready ? 1 : 0); Serial.print(" yaw="); Serial.print(yaw, 2);
  Serial.print(" gyroZ="); Serial.println(gyroZ, 2);
}