/**
 * @file robot_control.h
 * @brief Điều phối lệnh điều khiển, an toàn timeout và đèn LED tín hiệu.
 */

#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <Arduino.h>

/** Khởi tạo hệ thống điều khiển và đèn LED. */
void robotControlInit();

/** Vòng lặp giám sát an toàn watchdog và đèn LED. */
void robotControlTask();

/** Thực thi lệnh điều khiển từ server gồm vận tốc và hành động servo. */
void robotApplyCommand(int leftSpeed, int rightSpeed, int palletAction);

/** Dừng robot. */
void robotStop();

/** Bật hoặc tắt LED đỏ. */
void robotSetLedRed(bool on);

/** Bật hoặc tắt LED xanh. */
void robotSetLedBlue(bool on);

/** Chớp nhanh LED xanh khi nhận gói tin. */
void robotPulseLedBlue();

/** Kiểm tra trạng thái timeout của kết nối. */
bool robotIsTimedOut();

#endif