/**
 * @file command.h
 * @brief Giao diện dòng lệnh qua cổng Serial USB để kiểm thử robot.
 */

#ifndef ROBOT_COMMAND_H
#define ROBOT_COMMAND_H

/** Khởi tạo bộ đệm lệnh Serial. */
void commandInit();

/** Vòng lặp đọc và xử lý lệnh từ Serial. */
void commandTask();

#endif