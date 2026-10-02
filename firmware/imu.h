#ifndef ROBOT_IMU_H
#define ROBOT_IMU_H
#include <Arduino.h>
void imuInit();
void imuTask();
bool imuIsReady();
float imuYawDegrees();
float imuGyroZDegreesPerSecond();
void imuResetYaw();
void imuPrintStatus();
#endif