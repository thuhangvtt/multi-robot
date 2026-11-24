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
bool hasCommand = false;
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
  if (drv8833IsFault()) { motorStop(); return; }
  motorSetSpeeds(leftSpeed, rightSpeed);
  servoSetPalletAction(palletAction);

  lastCommandTime = millis();
  hasCommand = true;
  isTimedOut = false;
  robotSetLedRed(false);
  robotPulseLedBlue();
}

void robotApplyVelocity(float linearMmPerSec, float angularDegPerSec, int palletAction) {
  if (drv8833IsFault()) { motorStop(); return; }
  motorSetVelocity(linearMmPerSec, angularDegPerSec);
  servoSetPalletAction(palletAction);

  lastCommandTime = millis();
  hasCommand = true;
  isTimedOut = false;
  robotSetLedRed(false);
  robotPulseLedBlue();
}

void robotStop() {
  motorStop();
}

void robotEmergencyStop() {
  motorStop();
  robotSetLedRed(true);
}

void robotResetEmergencyStop() {
  motorStop();
  hasCommand = false;
  isTimedOut = false;
  lastCommandTime = millis();
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

  if (hasCommand && !isTimedOut && (now - lastCommandTime > COMMAND_TIMEOUT_MS)) {
    isTimedOut = true;
    const bool moving = motorIsMoving();
    if (moving) robotEmergencyStop(); else motorStop();
    robotSetLedRed(true);
    Serial.println("[Watchdog] Command timeout! Stopped motors.");
  }

  if (bluePulseActive && (now - lastBluePulse > 30)) {
    robotSetLedBlue(false);
    bluePulseActive = false;
  }

  if (drv8833IsFault()) {
    robotEmergencyStop();
  }
}
