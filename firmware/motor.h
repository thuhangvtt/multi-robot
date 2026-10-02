#ifndef ROBOT_MOTOR_H
#define ROBOT_MOTOR_H
#include <Arduino.h>
void motorInit();
void motorSetTarget(int16_t left, int16_t right);
void motorStop();
void motorTask();
int16_t motorLeftOutput();
int16_t motorRightOutput();
#endif