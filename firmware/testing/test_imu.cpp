#include "../config.h"
#include "../motor.h"
#include "../imu.h"
#include "serial_line.h"

SerialLine console;
bool stream=true;
uint32_t lastPrint=0;

void command(const char* line) {
  if (!strcmp(line,"ZERO")) { imuResetYaw(); Serial.println("OK ZERO (yaw origin only; not bias calibration)"); }
  else if (!strcmp(line,"STATUS")) imuPrintStatus();
  else if (!strcmp(line,"PAUSE")) stream=false;
  else if (!strcmp(line,"STREAM")) stream=true;
  else Serial.println("Use ZERO | STATUS | PAUSE | STREAM. Rotate robot by hand; motor commands unavailable.");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  motorInit(); motorStop();
  Serial.printf("TEST 3 IMU ROBOT=%u: keep still during startup bias calibration\n",ROBOT_ID);
  imuInit();
  Serial.println("ms,gyro_z_deg_s,yaw_deg,ready");
}

void loop() {
  motorStop(); // This test never drives the wheels.
  imuTask(); console.poll(command);
  if (stream && millis()-lastPrint>=200) {
    lastPrint=millis();
    Serial.printf("%lu,%.4f,%.4f,%d\n",(unsigned long)millis(),imuGyroZDegreesPerSecond(),imuYawDegrees(),imuIsReady());
  }
}
