#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H
#include <Arduino.h>
void robotControlInit();
void robotControlTask();
void robotSetMotion(int16_t left, int16_t right);
void robotSetStraight(bool enabled);
void robotTurn(int8_t direction);
void robotStop();
bool robotStraightEnabled();
#endif