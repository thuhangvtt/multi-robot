#include "servo_control.h"
#include "config.h"
#include <ESP32Servo.h>
namespace { Servo servoA; Servo servoB; }
void servoInit() {
  servoA.setPeriodHertz(50); servoB.setPeriodHertz(50);
  servoA.attach(SERVO_A_PIN, 500, 2500); servoB.attach(SERVO_B_PIN, 500, 2500);
  servoSetAngles(90, 90);
}
void servoSetAngles(int left, int right) { servoA.write(constrain(left, 0, 180)); servoB.write(constrain(right, 0, 180)); }
void servoTask() {}