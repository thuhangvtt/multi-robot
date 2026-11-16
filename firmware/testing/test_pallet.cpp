#include "../config.h"
#include "../motor.h"
#include "../servo_control.h"
#include "serial_line.h"

SerialLine console;
bool armed=false;

void command(const char* line) {
  int a,b;
  if (!strcmp(line,"ARM")) {
    if (!armed) { servoInit(); armed=true; }
    Serial.println("ARMED; lower angles applied. LIFT | LOWER | SERVO a b | STATUS");
    return;
  }
  if (!strcmp(line,"STATUS")) {
    Serial.printf("ROBOT=%u armed=%d a=%d b=%d commanded_lifted=%d\n",ROBOT_ID,armed,servoGetAngleA(),servoGetAngleB(),servoIsLifted());
    return;
  }
  if (!armed) { Serial.println("Use ARM first: this attaches servos and moves to configured LOWER angles"); return; }
  if (!strcmp(line,"LIFT")) { servoLiftPallet(); Serial.println("OK LIFT sent (verify pallet physically)"); }
  else if (!strcmp(line,"LOWER")) { servoLowerPallet(); Serial.println("OK LOWER sent"); }
  else if (sscanf(line,"SERVO %d %d",&a,&b)==2 && a>=0 && a<=180 && b>=0 && b<=180) {
    servoSetAngles(a,b); Serial.println("OK SERVO");
  } else Serial.println("Use ARM | LIFT | LOWER | SERVO <a 0..180> <b 0..180> | STATUS");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  motorInit(); motorStop();
  Serial.printf("TEST 4 PALLET ROBOT=%u: wheels stopped. Servos NOT attached until ARM.\n",ROBOT_ID);
}

void loop() { motorStop(); console.poll(command); if (armed) servoTask(); }
