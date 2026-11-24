#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <Arduino.h>

void robotControlInit();

void robotControlTask();

void robotApplyCommand(int leftSpeed, int rightSpeed, int palletAction);

void robotApplyVelocity(float linearMmPerSec, float angularDegPerSec, int palletAction);

void robotStop();
void robotEmergencyStop();
void robotResetEmergencyStop();

void robotSetLedRed(bool on);

void robotSetLedBlue(bool on);

void robotPulseLedBlue();

bool robotIsTimedOut();

#endif
