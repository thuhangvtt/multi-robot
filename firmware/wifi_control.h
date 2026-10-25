/**
 * @file wifi_control.h
 * @brief Quản lý kết nối Wi-Fi và WebServer HTTP nhận lệnh điều khiển.
 */

#ifndef ROBOT_WIFI_CONTROL_H
#define ROBOT_WIFI_CONTROL_H

/** Khởi tạo kết nối Wi-Fi và đăng ký các endpoint HTTP. */
void wifiControlInit();

/** Vòng lặp xử lý request HTTP và duy trì kết nối Wi-Fi. */
void wifiControlTask();

#endif