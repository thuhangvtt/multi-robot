#include "robot_control.h"
#include "config.h"
#include "imu.h"
#include "motor.h"
namespace {
int16_t requestedLeft = 0, requestedRight = 0;
bool straightEnabled = false;
bool turnActive = false;
int8_t turnDirection = 0;
float turnStartYaw = 0.0f;
float targetYaw = 0.0f;
uint32_t lastCommand = 0, lastStatus = 0;
float angleDifference(float current, float start) {
  float difference = current - start;
  while (difference > 180.0f) difference -= 360.0f;
  while (difference < -180.0f) difference += 360.0f;
  return difference;
}
}
void robotControlInit() { lastCommand = millis(); robotStop(); }
void robotSetMotion(int16_t left, int16_t right) {
  turnActive = false;
  requestedLeft = constrain(left, -PWM_LIMIT, PWM_LIMIT); requestedRight = constrain(right, -PWM_LIMIT, PWM_LIMIT); lastCommand = millis();
}
void robotSetStraight(bool enabled) { straightEnabled = enabled; if (enabled) targetYaw = imuYawDegrees(); }
void robotTurn(int8_t direction) {
  if (!imuIsReady() || direction == 0) return;
  turnActive = true;
  turnDirection = direction > 0 ? 1 : -1;
  turnStartYaw = imuYawDegrees();
  straightEnabled = false;
  lastCommand = millis();
}
void robotStop() { requestedLeft = 0; requestedRight = 0; turnActive = false; motorStop(); }
void robotControlTask() {
  if (!turnActive && millis() - lastCommand > COMMAND_TIMEOUT_MS) { robotStop(); straightEnabled = false; return; }
  int16_t left = requestedLeft, right = requestedRight;
  if (turnActive) {
    if (fabs(angleDifference(imuYawDegrees(), turnStartYaw)) >= TURN_ANGLE_DEGREES) {
      robotStop();
      return;
    }
    left = -turnDirection * TURN_PWM;
    right = turnDirection * TURN_PWM;
    lastCommand = millis();
  } else if (straightEnabled && imuIsReady() &&
             ((requestedLeft > 0 && requestedRight > 0) ||
              (requestedLeft < 0 && requestedRight < 0))) {
    float error = targetYaw - imuYawDegrees();
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    float correction = YAW_KP * error - YAW_KD * imuGyroZDegreesPerSecond();
    correction = constrain(correction, -MAX_YAW_CORRECTION, MAX_YAW_CORRECTION);
    left = constrain(static_cast<int16_t>(requestedLeft - correction), -PWM_LIMIT, PWM_LIMIT);
    right = constrain(static_cast<int16_t>(requestedRight + correction), -PWM_LIMIT, PWM_LIMIT);
  }
  motorSetTarget(left, right);
  if (millis() - lastStatus >= STATUS_PERIOD_MS) {
    lastStatus = millis(); Serial.print("STATE target="); Serial.print(requestedLeft); Serial.print(','); Serial.print(requestedRight);
    Serial.print(" output="); Serial.print(motorLeftOutput()); Serial.print(','); Serial.print(motorRightOutput());
    Serial.print(" straight="); Serial.println(straightEnabled ? 1 : 0);
  }
}
bool robotStraightEnabled() { return straightEnabled; }