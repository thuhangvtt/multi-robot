#pragma once
#include "../config.h"
#include "../motor.h"
#include "serial_line.h"

class MotorPulse {
public:
  bool running = false;
  bool measured = false;
  bool raw = true;
  int left = 0, right = 0;
  uint32_t started = 0, requested = 0, actual = 0;
  void init() {
    motorInit();
    motorSetStabilizer(false);
    Serial.printf("ROBOT=%u RAW_LIMIT=%d No Wi-Fi; motors initially stopped\n",
                  ROBOT_ID, DC_MOTOR_MAX_PWM);
  }
  bool begin(int l, int r, int ms, bool direct) {
    if (running) {
      Serial.println("ERROR busy; STOP first");
      return false;
    }
    if (ms < 100 || ms > 5000 ||
        (direct && (l < -DC_MOTOR_MAX_PWM || l > DC_MOTOR_MAX_PWM ||
                    r < -DC_MOTOR_MAX_PWM || r > DC_MOTOR_MAX_PWM)) ||
        (!direct && (l < 50 || l > 130 || r < 50 || r > 130)) ||
        drv8833IsFault()) {
      Serial.println("ERROR range/fault: pulse 100..5000ms, duty within "
                     "RAW_LIMIT or protocol 50..130");
      return false;
    }
    left = l;
    right = r;
    requested = (uint32_t)ms;
    raw = direct;
    measured = false;
    started = millis();
    running = true;
    if (raw)
      motorSetRawPwm(l, r);
    else
      motorSetSpeeds(l, r);
    Serial.printf("BEGIN,robot=%u,mode=%s,left=%d,right=%d,requested_ms=%lu\n",
                  ROBOT_ID, raw ? "RAW" : "PROTOCOL", l, r,
                  (unsigned long)requested);
    return true;
  }
  void stop(const char *reason) {
    motorStop();
    if (running) {
      actual = millis() - started;
      measured = !strcmp(reason, "DONE");
      Serial.printf("END,actual_ms=%lu,reason=%s\n", (unsigned long)actual,
                    reason);
    }
    running = false;
  }
  void tick() {
    if (running && drv8833IsFault())
      stop("FAULT");
    if (running && millis() - started >= requested)
      stop("DONE");
    motorTask();
  }
};
