#ifndef ROBOT_MOTOR_H
#define ROBOT_MOTOR_H

#include <Arduino.h>

void motorInit();

void motorSetSpeeds(int leftSpeed, int rightSpeed);

void motorSetRawPwm(int leftPwm, int rightPwm);
void motorSetStabilizer(bool enabled);

void motorStop();

void motorTask();

int motorGetLeftSpeed();

int motorGetRightSpeed();

bool drv8833IsFault();

#endif
