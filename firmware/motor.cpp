/**
 * @file motor.cpp
 * @brief Triển khai điều khiển động cơ DC DRV8833.
 */

#include "motor.h"
#include "config.h"

namespace {
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
bool faultDetected = false;

int convertToDcPwm(int val) {
  if (val == 0) return 0;

  if (val >= 40 && val <= 140) {
    int delta = val - 90;
    if (abs(delta) <= 1) return 0;

    if (delta > 0) {
      return map(delta, 2, 40, DC_MOTOR_MIN_PWM, DC_MOTOR_MAX_PWM);
    } else {
      return -map(-delta, 2, 40, DC_MOTOR_MIN_PWM, DC_MOTOR_MAX_PWM);
    }
  }

  return constrain(val, -255, 255);
}

void driveHBridge(uint8_t pin1, uint8_t pin2, int pwm, bool inverted) {
  if (inverted) pwm = -pwm;

  if (pwm > 0) {
    analogWrite(pin1, constrain(pwm, 0, 255));
    analogWrite(pin2, 0);
  } else if (pwm < 0) {
    analogWrite(pin1, 0);
    analogWrite(pin2, constrain(-pwm, 0, 255));
  } else {
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
  }
}
}

void motorInit() {
  pinMode(DRV8833_EEP_PIN, OUTPUT);
  digitalWrite(DRV8833_EEP_PIN, HIGH);

  pinMode(DRV8833_FAULT_PIN, INPUT_PULLUP);

  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  motorStop();
}

void motorSetSpeeds(int leftSpeed, int rightSpeed) {
  digitalWrite(DRV8833_EEP_PIN, HIGH);

  int leftPwm = convertToDcPwm(leftSpeed);
  int rightPwm = convertToDcPwm(rightSpeed);

  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;

  driveHBridge(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, leftPwm, MOTOR_LEFT_INVERTED);
  driveHBridge(MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, rightPwm, MOTOR_RIGHT_INVERTED);
}

void motorStop() {
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  analogWrite(MOTOR_LEFT_IN1, 0);
  analogWrite(MOTOR_LEFT_IN2, 0);
  analogWrite(MOTOR_RIGHT_IN1, 0);
  analogWrite(MOTOR_RIGHT_IN2, 0);
}

int motorGetLeftSpeed() {
  return currentLeftSpeed;
}

int motorGetRightSpeed() {
  return currentRightSpeed;
}

bool drv8833IsFault() {
  return digitalRead(DRV8833_FAULT_PIN) == LOW;
}

void motorTask() {
  faultDetected = drv8833IsFault();
  if (faultDetected) {
    motorStop();
  }
}