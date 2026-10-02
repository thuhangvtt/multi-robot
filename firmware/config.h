#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#include <Arduino.h>
#include "config_private.h"
constexpr uint8_t MOTOR_LEFT_IN1 = 26;
constexpr uint8_t MOTOR_LEFT_IN2 = 25;
constexpr uint8_t MOTOR_RIGHT_IN1 = 33;
constexpr uint8_t MOTOR_RIGHT_IN2 = 32;
constexpr uint8_t MOTOR_ENABLE = 19;
constexpr bool MOTOR_LEFT_INVERTED = true;
constexpr bool MOTOR_RIGHT_INVERTED = true;
constexpr uint8_t SERVO_A_PIN = 12;
constexpr uint8_t SERVO_B_PIN = 14;
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t MPU6050_ADDRESS = 0x68;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t CONTROL_PERIOD_MS = 10;
constexpr uint32_t STATUS_PERIOD_MS = 500;
constexpr uint32_t COMMAND_TIMEOUT_MS = 800;
constexpr int16_t PWM_LIMIT = 50;
constexpr int16_t PWM_RAMP_STEP = 1;
constexpr int16_t TURN_PWM = 38;
constexpr float TURN_ANGLE_DEGREES = 90.0f;
constexpr float YAW_KP = 1.0f;
constexpr float YAW_KD = 0.12f;
constexpr float MAX_YAW_CORRECTION = 16.0f;
constexpr float GYRO_Z_SCALE = 131.0f;
constexpr bool WIFI_USE_ACCESS_POINT = false;
constexpr uint8_t ROBOT_ID = 1;
#endif