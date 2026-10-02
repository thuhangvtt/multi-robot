#ifndef ROBOT_SERVO_CONTROL_H
#define ROBOT_SERVO_CONTROL_H
#include <Arduino.h>
void servoInit();
void servoSetAngles(int left, int right);
void servoTask();
#endif