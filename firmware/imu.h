/**
 * @file imu.h
 * @brief Đọc cảm biến con quay hồi chuyển MPU6050 tính góc Yaw.
 */

#ifndef ROBOT_IMU_H
#define ROBOT_IMU_H

#include <Arduino.h>

/** Khởi tạo kết nối I2C và hiệu chuẩn cảm biến. */
void imuInit();

/** Vòng lặp lấy mẫu dữ liệu và tích phân góc quay. */
void imuTask();

/** Kiểm tra cảm biến đã sẵn sàng. */
bool imuIsReady();

/** Lấy góc Yaw theo độ. */
float imuYawDegrees();

/** Lấy vận tốc góc trục Z (độ/giây). */
float imuGyroZDegreesPerSecond();

/** Reset góc Yaw về 0. */
void imuResetYaw();

/** In thông số IMU ra Serial. */
void imuPrintStatus();

#endif