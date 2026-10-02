#include "motor.h"
#include "config.h"
namespace {
int16_t leftTarget = 0, rightTarget = 0, leftOutput = 0, rightOutput = 0;
uint32_t lastUpdate = 0;
int16_t approach(int16_t current, int16_t target) {
  if (current < target) return min<int16_t>(current + PWM_RAMP_STEP, target);
  if (current > target) return max<int16_t>(current - PWM_RAMP_STEP, target);
  return current;
}
void writeMotor(uint8_t pinA, uint8_t pinB, int16_t value) {
  value = constrain(value, -PWM_LIMIT, PWM_LIMIT);
  analogWrite(pinA, value >= 0 ? value : 0);
  analogWrite(pinB, value < 0 ? -value : 0);
}
int16_t applyMotorDirection(int16_t value, bool inverted) {
  return inverted ? -value : value;
}
}
void motorInit() {
  pinMode(MOTOR_ENABLE, OUTPUT);
  digitalWrite(MOTOR_ENABLE, HIGH);
  pinMode(MOTOR_LEFT_IN1, OUTPUT); pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT); pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  motorStop();
}
void motorSetTarget(int16_t left, int16_t right) {
  leftTarget = constrain(left, -PWM_LIMIT, PWM_LIMIT);
  rightTarget = constrain(right, -PWM_LIMIT, PWM_LIMIT);
}
void motorStop() { motorSetTarget(0, 0); }
void motorTask() {
  const uint32_t now = millis();
  if (now - lastUpdate < CONTROL_PERIOD_MS) return;
  lastUpdate = now;
  leftOutput = approach(leftOutput, leftTarget);
  rightOutput = approach(rightOutput, rightTarget);
  writeMotor(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, applyMotorDirection(leftOutput, MOTOR_LEFT_INVERTED));
  writeMotor(MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, applyMotorDirection(rightOutput, MOTOR_RIGHT_INVERTED));
}
int16_t motorLeftOutput() { return leftOutput; }
int16_t motorRightOutput() { return rightOutput; }