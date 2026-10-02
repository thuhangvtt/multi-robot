#include "command.h"
#include "imu.h"
#include "motor.h"
#include "robot_control.h"
#include "servo_control.h"
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
namespace {
char line[80]; uint8_t lineLength = 0;
void printHelp() {
  Serial.println("M <left> <right>       motor PWM -255..255");
  Serial.println("STRAIGHT ON|OFF         gyro heading correction");
  Serial.println("TURN LEFT|RIGHT         turn 90 degrees");
  Serial.println("SERVO <a> <b>           servo angles 0..180");
  Serial.println("STATUS / IMU / ZEROYAW / STOP / HELP");
}
void handleLine(char *input) {
  int left = 0, right = 0, first = 0, second = 0;
  if (sscanf(input, "M %d %d", &left, &right) == 2) { robotSetMotion(left, right); Serial.println("OK M"); }
  else if (sscanf(input, "SERVO %d %d", &first, &second) == 2) { servoSetAngles(first, second); Serial.println("OK SERVO"); }
  else if (strcmp(input, "STRAIGHT ON") == 0) { robotSetStraight(true); Serial.println("OK STRAIGHT ON"); }
  else if (strcmp(input, "STRAIGHT OFF") == 0) { robotSetStraight(false); Serial.println("OK STRAIGHT OFF"); }
  else if (strcmp(input, "TURN LEFT") == 0) { robotTurn(-1); Serial.println("OK TURN LEFT"); }
  else if (strcmp(input, "TURN RIGHT") == 0) { robotTurn(1); Serial.println("OK TURN RIGHT"); }
  else if (strcmp(input, "STOP") == 0) { robotStop(); Serial.println("OK STOP"); }
  else if (strcmp(input, "STATUS") == 0) { Serial.print("STATUS motor="); Serial.print(motorLeftOutput()); Serial.print(','); Serial.println(motorRightOutput()); imuPrintStatus(); }
  else if (strcmp(input, "IMU") == 0) { imuPrintStatus(); }
  else if (strcmp(input, "ZEROYAW") == 0) { imuResetYaw(); Serial.println("OK ZEROYAW"); }
  else if (strcmp(input, "HELP") == 0) { printHelp(); }
  else if (lineLength > 0) Serial.println("ERROR unknown command; use HELP");
}
}
void commandInit() { lineLength = 0; printHelp(); }
void commandTask() {
  while (Serial.available() > 0) {
    const char character = static_cast<char>(Serial.read());
    if (character == '\n' || character == '\r') {
      if (lineLength == 0) continue;
      line[lineLength] = '\0'; handleLine(line); lineLength = 0;
    } else if (lineLength < sizeof(line) - 1) line[lineLength++] = character;
    else { lineLength = 0; Serial.println("ERROR command too long"); }
  }
}