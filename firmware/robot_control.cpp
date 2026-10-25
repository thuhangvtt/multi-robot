/**
 * @file robot_control.cpp
 * @brief Triển khai điều phối lệnh, watchdog timeout và tín hiệu LED.
 */

#include "robot_control.h"
#include "config.h"
#include "motor.h"
#include "servo_control.h"
#include "imu.h"

namespace {
uint32_t lastCommandTime = 0;
bool isTimedOut = false;
uint32_t lastBluePulse = 0;
bool bluePulseActive = false;
}

void robotControlInit() {
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  robotSetLedRed(false);
  robotSetLedBlue(false);

  lastCommandTime = millis();
  robotStop();
}

void robotApplyCommand(int leftSpeed, int rightSpeed, int palletAction) {
  motorSetSpeeds(leftSpeed, rightSpeed);
  servoSetPalletAction(palletAction);

  lastCommandTime = millis();
  isTimedOut = false;
  robotSetLedRed(false);
  robotPulseLedBlue();
}

void robotStop() {
  motorStop();
}

void robotSetLedRed(bool on) {
  digitalWrite(LED_RED_PIN, on ? HIGH : LOW);
}

void robotSetLedBlue(bool on) {
  digitalWrite(LED_BLUE_PIN, on ? HIGH : LOW);
}

void robotPulseLedBlue() {
  robotSetLedBlue(true);
  lastBluePulse = millis();
  bluePulseActive = true;
}

bool robotIsTimedOut() {
  return isTimedOut;
}

void robotControlTask() {
  const uint32_t now = millis();

  if (!isTimedOut && (now - lastCommandTime > COMMAND_TIMEOUT_MS)) {
    isTimedOut = true;
    motorStop();
    robotSetLedRed(true);
    Serial.println("[Watchdog] Command timeout! Stopped motors.");
  }

  if (bluePulseActive && (now - lastBluePulse > 30)) {
    robotSetLedBlue(false);
    bluePulseActive = false;
  }

  if (drv8833IsFault()) {
    robotSetLedRed(true);
  }
}