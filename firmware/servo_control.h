#ifndef ROBOT_SERVO_CONTROL_H
#define ROBOT_SERVO_CONTROL_H

#include <Arduino.h>

void servoInit();

void servoSetPalletAction(int action);

void servoSetAngles(int angleA, int angleB);

void servoLiftPallet();

void servoLowerPallet();

bool servoIsLifted();

int servoGetAngleA();

int servoGetAngleB();

void servoTask();

#endif
