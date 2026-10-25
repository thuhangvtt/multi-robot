/**
 * @file motor.h
 * @brief Điều khiển 2 động cơ DC qua IC cầu H DRV8833.
 */

#ifndef ROBOT_MOTOR_H
#define ROBOT_MOTOR_H

#include <Arduino.h>

/** Khởi tạo các chân PWM và chân kích hoạt nSLEEP của DRV8833. */
void motorInit();

/** Thiết lập tốc độ 2 bánh xe. */
void motorSetSpeeds(int leftSpeed, int rightSpeed);

/** Dừng khẩn cấp cả hai động cơ. */
void motorStop();

/** Vòng lặp giám sát lỗi phần cứng nFAULT. */
void motorTask();

/** Lấy tốc độ hiện tại bánh trái. */
int motorGetLeftSpeed();

/** Lấy tốc độ hiện tại bánh phải. */
int motorGetRightSpeed();

/** Kiểm tra cờ báo lỗi từ chân nFAULT. */
bool drv8833IsFault();

#endif