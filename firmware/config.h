/**
 * @file config.h
 * @brief Cấu hình chân kết nối và thông số hoạt động của robot ESP32.
 */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>
#include "config_private.h"

constexpr uint8_t ROBOT_ID = 1;

constexpr bool USE_STATIC_IP = true;
const IPAddress STATIC_IP(192, 168, 100, 100 + ROBOT_ID);
const IPAddress GATEWAY_IP(192, 168, 100, 1);
const IPAddress SUBNET_MASK(255, 255, 255, 0);
const IPAddress DNS_IP(192, 168, 100, 1);

constexpr bool WIFI_USE_ACCESS_POINT = false;

constexpr uint8_t MOTOR_LEFT_IN1    = 26;
constexpr uint8_t MOTOR_LEFT_IN2    = 25;
constexpr uint8_t MOTOR_RIGHT_IN1   = 33;
constexpr uint8_t MOTOR_RIGHT_IN2   = 32;
constexpr uint8_t DRV8833_EEP_PIN   = 15;
constexpr uint8_t DRV8833_FAULT_PIN = 5;

constexpr bool MOTOR_LEFT_INVERTED  = false;
constexpr bool MOTOR_RIGHT_INVERTED = false;

constexpr int DC_MOTOR_MIN_PWM = 40;
constexpr int DC_MOTOR_MAX_PWM = 120;  // Giảm từ 255 → 120 để robot chạy chậm hơn (~47% tốc độ)

constexpr uint8_t SERVO_A_PIN = 13;
constexpr uint8_t SERVO_B_PIN = 14;

constexpr int SERVO_A_LOWER_ANGLE = 0;
constexpr int SERVO_A_LIFT_ANGLE  = 90;
constexpr int SERVO_B_LOWER_ANGLE = 0;
constexpr int SERVO_B_LIFT_ANGLE  = 90;

constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 23;
constexpr uint8_t MPU6050_ADDRESS = 0x68;

constexpr uint8_t LED_RED_PIN  = 4;
constexpr uint8_t LED_BLUE_PIN = 2;

constexpr uint32_t SERIAL_BAUD        = 115200;
constexpr uint32_t COMMAND_TIMEOUT_MS = 800;
constexpr uint32_t STATUS_PERIOD_MS   = 500;
constexpr uint32_t CONTROL_PERIOD_MS  = 10;
constexpr float GYRO_Z_SCALE          = 131.0f;

#endif