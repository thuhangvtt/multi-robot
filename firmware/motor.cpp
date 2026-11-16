#include "motor.h"
#include "config.h"
#include "imu.h"

namespace {
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
int targetLeftPwm = 0;
int targetRightPwm = 0;
int activeLeftPwm = 0;
int activeRightPwm = 0;
bool faultDetected = false;
bool rawCalibration = false;
bool stabilizerEnabled = USE_IMU_STABILIZER;
uint32_t lastMotorUpdate = 0;
constexpr int MAX_PWM_SLEW_STEP = 6;

int convertToDcPwm(int val, int startPwm, int referencePwm) {
  if (val == 0)
    return 0;

  if (val >= 40 && val <= 140) {
    int delta = val - 90;
    if (abs(delta) <= 1)
      return 0;

    int absDelta = constrain(abs(delta), 2, MOTOR_COMMAND_REFERENCE_DELTA);
    int dcPwm =
        map(absDelta, 2, MOTOR_COMMAND_REFERENCE_DELTA, startPwm, referencePwm);

    return (delta > 0) ? dcPwm : -dcPwm;
  }

  return constrain(val, -255, 255);
}

void driveHBridge(uint8_t pin1, uint8_t pin2, int pwm, bool inverted) {
  if (inverted)
    pwm = -pwm;

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

void motorSetStabilizer(bool enabled) { stabilizerEnabled = enabled; }

void motorSetSpeeds(int leftSpeed, int rightSpeed) {
  rawCalibration = false;
  digitalWrite(DRV8833_EEP_PIN, HIGH);

  int trimmedLeft = (leftSpeed != 0 && leftSpeed != 90)
                        ? (leftSpeed + MOTOR_LEFT_TRIM)
                        : leftSpeed;
  int trimmedRight = (rightSpeed != 0 && rightSpeed != 90)
                         ? (rightSpeed + MOTOR_RIGHT_TRIM)
                         : rightSpeed;

  targetLeftPwm = convertToDcPwm(trimmedLeft, MOTOR_LEFT_START_PWM,
                                 MOTOR_LEFT_REFERENCE_PWM);
  targetRightPwm = convertToDcPwm(trimmedRight, MOTOR_RIGHT_START_PWM,
                                  MOTOR_RIGHT_REFERENCE_PWM);

  currentLeftSpeed = leftSpeed;
  currentRightSpeed = rightSpeed;

  if (targetLeftPwm == 0 && targetRightPwm == 0) {
    activeLeftPwm = 0;
    activeRightPwm = 0;
    driveHBridge(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, 0, MOTOR_LEFT_INVERTED);
    driveHBridge(MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, 0, MOTOR_RIGHT_INVERTED);
  }
}

void motorStop() {
  rawCalibration = false;
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  targetLeftPwm = 0;
  targetRightPwm = 0;
  activeLeftPwm = 0;
  activeRightPwm = 0;
  analogWrite(MOTOR_LEFT_IN1, 0);
  analogWrite(MOTOR_LEFT_IN2, 0);
  analogWrite(MOTOR_RIGHT_IN1, 0);
  analogWrite(MOTOR_RIGHT_IN2, 0);
}

void motorSetRawPwm(int leftPwm, int rightPwm) {
  rawCalibration = true;
  currentLeftSpeed = leftPwm;
  currentRightSpeed = rightPwm;
  if (drv8833IsFault()) {
    motorStop();
    return;
  }
  digitalWrite(DRV8833_EEP_PIN, HIGH);
  driveHBridge(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2,
               constrain(leftPwm, -DC_MOTOR_MAX_PWM, DC_MOTOR_MAX_PWM),
               MOTOR_LEFT_INVERTED);
  driveHBridge(MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2,
               constrain(rightPwm, -DC_MOTOR_MAX_PWM, DC_MOTOR_MAX_PWM),
               MOTOR_RIGHT_INVERTED);
}

int motorGetLeftSpeed() { return currentLeftSpeed; }

int motorGetRightSpeed() { return currentRightSpeed; }

bool drv8833IsFault() {

  if (!DRV8833_USE_FAULT_PIN)
    return false;
  return digitalRead(DRV8833_FAULT_PIN) == LOW;
}

void motorTask() {
  const bool newFault = drv8833IsFault();
  if (newFault != faultDetected) {
    Serial.println(newFault ? "[Motor] nFAULT LOW: stopping motors."
                            : "[Motor] nFAULT cleared.");
  }
  faultDetected = newFault;
  if (faultDetected) {
    motorStop();
    return;
  }

  if (rawCalibration) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastMotorUpdate < CONTROL_PERIOD_MS) {
    return;
  }
  lastMotorUpdate = now;

  auto stepSlew = [](int current, int target) -> int {
    if (target == 0) return 0;
    int diff = target - current;
    if (abs(diff) <= MAX_PWM_SLEW_STEP) return target;
    return current + (diff > 0 ? MAX_PWM_SLEW_STEP : -MAX_PWM_SLEW_STEP);
  };

  activeLeftPwm = stepSlew(activeLeftPwm, targetLeftPwm);
  activeRightPwm = stepSlew(activeRightPwm, targetRightPwm);

  int driveLeft = activeLeftPwm;
  int driveRight = activeRightPwm;

  if (stabilizerEnabled && imuIsReady() && (driveLeft != 0 || driveRight != 0)) {
    int speedDiff = abs(currentLeftSpeed - currentRightSpeed);
    if (speedDiff <= 2) {
      float gyroZ = imuGyroZDegreesPerSecond();
      float correction = gyroZ * IMU_STABILIZER_KP;
      int corrPwm = constrain((int)round(correction), -IMU_MAX_CORRECTION_PWM,
                              IMU_MAX_CORRECTION_PWM);
      driveLeft = constrain(driveLeft + corrPwm, -DC_MOTOR_MAX_PWM, DC_MOTOR_MAX_PWM);
      driveRight = constrain(driveRight - corrPwm, -DC_MOTOR_MAX_PWM, DC_MOTOR_MAX_PWM);
    }
  }

  driveHBridge(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, driveLeft, MOTOR_LEFT_INVERTED);
  driveHBridge(MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, driveRight, MOTOR_RIGHT_INVERTED);
}
