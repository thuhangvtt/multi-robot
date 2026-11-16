#include "command.h"
#include "config.h"
#include "imu.h"
#include "motor.h"
#include "robot_control.h"
#include "servo_control.h"
#include "wifi_control.h"


void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  Serial.println("\n=================================");
  Serial.printf("MULTI-ROBOT (ROBOT %u)\n", ROBOT_ID);
  Serial.println("=================================");

  motorInit();
  servoInit();
  imuInit();
  robotControlInit();
  commandInit();
  wifiControlInit();

  Serial.println("[System] KHỞI ĐỘNG HOÀN TẤT. SẴN SÀNG NHẬN LỆNH.\n");
}

void loop() {
  commandTask();
  wifiControlTask();
  imuTask();
  robotControlTask();
  motorTask();
  servoTask();
}
