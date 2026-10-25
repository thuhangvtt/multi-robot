/**
 * @file servo_control.h
 * @brief Điều khiển cơ cấu nâng / gắp pallet qua 2 động cơ RC Servo.
 */

#ifndef ROBOT_SERVO_CONTROL_H
#define ROBOT_SERVO_CONTROL_H

#include <Arduino.h>

/** Khởi tạo xung PWM điều khiển servo. */
void servoInit();

/** Thực thi hành động nâng (1), hạ (2) hoặc giữ nguyên (0). */
void servoSetPalletAction(int action);

/** Đặt góc quay trực tiếp cho ServoA và ServoB. */
void servoSetAngles(int angleA, int angleB);

/** Nâng hoặc gắp pallet. */
void servoLiftPallet();

/** Hạ hoặc nhả pallet. */
void servoLowerPallet();

/** Kiểm tra trạng thái nâng/hạ. */
bool servoIsLifted();

/** Lấy góc hiện tại ServoA. */
int servoGetAngleA();

/** Lấy góc hiện tại ServoB. */
int servoGetAngleB();

/** Vòng lặp cập nhật servo định kỳ. */
void servoTask();

#endif