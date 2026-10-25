#include "command.h"
#include "imu.h"
#include "motor.h"
#include "robot_control.h"
#include "servo_control.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

namespace {
char line[80];
uint8_t lineLength = 0;

void printHelp() {
  Serial.println("--- PARROT Robot Serial CLI ---");
  Serial.println("M <left> <right>       Chay 2 dong co DC banh xe (vi du: M 110 110 hoac M 150 150)");
  Serial.println("LIFT                   Nang / Gap pallet (ServoA & ServoB)");
  Serial.println("LOWER                  Ha / Nha pallet (ServoA & ServoB)");
  Serial.println("SERVO <a> <b>          Dat goc servo tuy chinh (0..180)");
  Serial.println("STOP                   Phanh dung banh xe");
  Serial.println("STATUS                 Xem thong so dong co, servo, IMU va loi");
  Serial.println("ZEROYAW                Reset goc Yaw IMU ve 0");
  Serial.println("HELP                   Xem menu huong dan nay");
}

void handleLine(char *input) {
  int left = 0, right = 0, angleA = 0, angleB = 0;
  if (sscanf(input, "M %d %d", &left, &right) == 2) {
    robotApplyCommand(left, right, 0);
    Serial.println("OK M");
  } else if (strcmp(input, "LIFT") == 0) {
    servoLiftPallet();
    Serial.println("OK LIFT (Pallet raised)");
  } else if (strcmp(input, "LOWER") == 0) {
    servoLowerPallet();
    Serial.println("OK LOWER (Pallet lowered)");
  } else if (sscanf(input, "SERVO %d %d", &angleA, &angleB) == 2) {
    servoSetAngles(angleA, angleB);
    Serial.println("OK SERVO");
  } else if (strcmp(input, "STOP") == 0) {
    robotStop();
    Serial.println("OK STOP");
  } else if (strcmp(input, "STATUS") == 0) {
    Serial.print("STATUS motorL="); Serial.print(motorGetLeftSpeed());
    Serial.print(" motorR="); Serial.print(motorGetRightSpeed());
    Serial.print(" lifted="); Serial.print(servoIsLifted() ? "YES" : "NO");
    Serial.print(" angleA="); Serial.print(servoGetAngleA());
    Serial.print(" angleB="); Serial.print(servoGetAngleB());
    Serial.print(" drv8833_fault="); Serial.print(drv8833IsFault() ? "YES" : "NO");
    Serial.print(" timed_out="); Serial.println(robotIsTimedOut() ? "YES" : "NO");
    imuPrintStatus();
  } else if (strcmp(input, "IMU") == 0) {
    imuPrintStatus();
  } else if (strcmp(input, "ZEROYAW") == 0) {
    imuResetYaw();
    Serial.println("OK ZEROYAW");
  } else if (strcmp(input, "HELP") == 0) {
    printHelp();
  } else if (lineLength > 0) {
    Serial.println("ERROR unknown command; type HELP");
  }
}
}

void commandInit() {
  lineLength = 0;
  printHelp();
}

void commandTask() {
  while (Serial.available() > 0) {
    const char character = static_cast<char>(Serial.read());
    if (character == '\n' || character == '\r') {
      if (lineLength == 0) continue;
      line[lineLength] = '\0';
      handleLine(line);
      lineLength = 0;
    } else if (lineLength < sizeof(line) - 1) {
      line[lineLength++] = character;
    } else {
      lineLength = 0;
      Serial.println("ERROR command too long");
    }
  }
}