#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include "config_private.h"
#include <Arduino.h>

constexpr uint8_t ROBOT_ID = 1;

constexpr bool USE_STATIC_IP = true;
const IPAddress STATIC_IP(192, 168, 100, 100 + ROBOT_ID);
const IPAddress GATEWAY_IP(192, 168, 100, 1);
const IPAddress SUBNET_MASK(255, 255, 255, 0);
const IPAddress DNS_IP(192, 168, 100, 1);

constexpr bool WIFI_USE_ACCESS_POINT = false;

constexpr uint8_t MOTOR_LEFT_IN1 = 26;
constexpr uint8_t MOTOR_LEFT_IN2 = 25;
constexpr uint8_t MOTOR_RIGHT_IN1 = 33;
constexpr uint8_t MOTOR_RIGHT_IN2 = 32;
constexpr uint8_t DRV8833_EEP_PIN = 15;
constexpr uint8_t DRV8833_FAULT_PIN = 5;
constexpr bool DRV8833_USE_FAULT_PIN = true;

constexpr bool MOTOR_LEFT_INVERTED = true;
constexpr bool MOTOR_RIGHT_INVERTED = true;

constexpr int DC_MOTOR_MIN_PWM = 40;
constexpr int DC_MOTOR_MAX_PWM = 100;

constexpr int MOTOR_LEFT_TRIM  = 0;
constexpr int MOTOR_RIGHT_TRIM = 0;

constexpr uint8_t SERVO_A_PIN = 13;
constexpr uint8_t SERVO_B_PIN = 14;

constexpr int SERVO_A_LOWER_ANGLE = 0;
constexpr int SERVO_A_LIFT_ANGLE = 45;
constexpr int SERVO_B_LOWER_ANGLE = 90;
constexpr int SERVO_B_LIFT_ANGLE = 45;
constexpr uint32_t SERVO_MOVE_DURATION_MS = 1500;
constexpr uint32_t SERVO_UPDATE_INTERVAL_MS = 20;

constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t MPU6050_ADDRESS = 0x68;

constexpr bool USE_IMU_STABILIZER = true;
constexpr float IMU_STABILIZER_KP = 0.20f;
constexpr int IMU_MAX_CORRECTION_PWM = 8;

constexpr uint8_t LED_RED_PIN = 4;
constexpr uint8_t LED_BLUE_PIN = 2;

constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint32_t COMMAND_TIMEOUT_MS = 600;
constexpr uint32_t STATUS_PERIOD_MS = 500;
constexpr uint32_t CONTROL_PERIOD_MS = 10;
constexpr float GYRO_Z_SCALE = 131.0f;

constexpr int MOTOR_LEFT_START_PWM = 44;
constexpr int MOTOR_RIGHT_START_PWM = 48;
constexpr int MOTOR_LEFT_REFERENCE_PWM = 61;
constexpr int MOTOR_RIGHT_REFERENCE_PWM = 63;
constexpr int MOTOR_COMMAND_REFERENCE_DELTA = 22;

// Calibrated physical drive parameters (robot_1_measurements.json)
constexpr float WHEEL_BASE_MM = 82.0f;
constexpr float MOTOR_REF_SPEED_MM_S = 280.0f;       // Measured 1400mm / 5s at PWM 61/63
constexpr float MOTOR_MIN_LINEAR_SPEED_MM_S = 20.0f;
constexpr int MOTOR_LEFT_MIN_RUN_PWM = 44;
constexpr int MOTOR_RIGHT_MIN_RUN_PWM = 48;
constexpr int MOTOR_KICK_PWM_BOOST = 14;
constexpr uint32_t MOTOR_KICK_DURATION_MS = 50;

// 100 Hz MPU6050 Yaw-Rate PI (+gyroZ = left/CCW turn)
constexpr float IMU_YAW_RATE_KP = 1.20f;
constexpr float IMU_YAW_RATE_KI = 5.00f;
constexpr float IMU_YAW_RATE_MAX_DIFF_MM_S = 180.0f;
constexpr float IMU_STRAIGHT_HEADING_KP = 1.50f;
constexpr float IMU_STRAIGHT_HEADING_MAX_DPS = 20.0f;
constexpr float IMU_YAW_DEADBAND_DPS = 0.8f;

#endif
