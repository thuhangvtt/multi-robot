/**
 * @file servo_control.cpp
 * @brief Triển khai điều khiển cơ cấu nâng / gắp pallet.
 */

#include "servo_control.h"
#include "config.h"
#include <ESP32Servo.h>

namespace {
Servo servoA;
Servo servoB;
int currentAngleA = SERVO_A_LOWER_ANGLE;
int currentAngleB = SERVO_B_LOWER_ANGLE;
bool isLifted = false;
}

void servoInit() {
  servoA.setPeriodHertz(50);
  servoB.setPeriodHertz(50);
  servoA.attach(SERVO_A_PIN, 500, 2500);
  servoB.attach(SERVO_B_PIN, 500, 2500);
  servoLowerPallet();
}

void servoSetAngles(int angleA, int angleB) {
  currentAngleA = constrain(angleA, 0, 180);
  currentAngleB = constrain(angleB, 0, 180);

  servoA.write(currentAngleA);
  servoB.write(currentAngleB);
}

void servoLiftPallet() {
  servoSetAngles(SERVO_A_LIFT_ANGLE, SERVO_B_LIFT_ANGLE);
  isLifted = true;
}

void servoLowerPallet() {
  servoSetAngles(SERVO_A_LOWER_ANGLE, SERVO_B_LOWER_ANGLE);
  isLifted = false;
}

void servoSetPalletAction(int action) {
  if (action == 1) {
    servoLiftPallet();
  } else if (action == 2) {
    servoLowerPallet();
  }
}

bool servoIsLifted() {
  return isLifted;
}

int servoGetAngleA() {
  return currentAngleA;
}

int servoGetAngleB() {
  return currentAngleB;
}

void servoTask() {
}