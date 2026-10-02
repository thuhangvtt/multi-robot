#include "config.h"
#include "motor.h"
#include "imu.h"
#include "servo_control.h"
#include "command.h"
#include "robot_control.h"
#include "wifi_control.h"

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);
  motorInit();
  servoInit();
  imuInit();
  robotControlInit();
  commandInit();
  wifiControlInit();
  Serial.println("READY");
}

void loop() {
  commandTask();
  wifiControlTask();
  imuTask();
  robotControlTask();
  motorTask();
  servoTask();
}