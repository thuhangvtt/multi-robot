#include "motor_pulse.h"

MotorPulse pulse;
SerialLine console;

void command(const char* line) {
  int left, right, ms;
  if (!strcmp(line,"STOP")) pulse.stop("STOP");
  else if (sscanf(line,"PWM %d %d %d", &left,&right,&ms)==3) pulse.begin(left,right,ms,true);
  else if (!strcmp(line,"STATUS"))
    Serial.printf("ROBOT=%u running=%d fault=%d max_duty=%d\n",ROBOT_ID,pulse.running,drv8833IsFault(),DC_MOTOR_MAX_PWM);
  else Serial.println("Use PWM <left signed duty> <right signed duty> <100..5000 ms> | STOP | STATUS");
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pulse.init();
  Serial.println("TEST 1 PWM: e.g. PWM 20 0 1000. Start with wheels lifted.");
}

void loop() { pulse.tick(); console.poll(command); }
