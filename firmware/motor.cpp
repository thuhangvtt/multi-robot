#include "motor.h"
#include "config.h"
#include "imu.h"
#include <math.h>

namespace {
enum class MotorMode { Stopped, LegacyPwm, Velocity, RawPwm };

MotorMode motorMode = MotorMode::Stopped;
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
int targetLeftPwm = 0;
int targetRightPwm = 0;
int activeLeftPwm = 0;
int activeRightPwm = 0;
float targetLinearMmS = 0.0f;
float targetAngularDps = 0.0f;
float yawRateIntegral = 0.0f;
bool straightHoldActive = false;
float straightRefYaw = 0.0f;
int lastAppliedLeftPwm = 0;
int lastAppliedRightPwm = 0;
uint32_t leftKickUntil = 0;
uint32_t rightKickUntil = 0;
bool faultDetected = false;
bool stabilizerEnabled = USE_IMU_STABILIZER;
uint32_t lastMotorUpdate = 0;
constexpr int MAX_PWM_SLEW_STEP = 6;

int wheelSpeedToPwm(float vMmS, int minRunPwm, int refPwm) {
  const float absV = fabsf(vMmS);
  if (absV < 2.0f) {
    return 0;
  }
  int mag = 0;
  if (absV < MOTOR_MIN_LINEAR_SPEED_MM_S) {
    mag = (int)roundf((absV / MOTOR_MIN_LINEAR_SPEED_MM_S) * (float)minRunPwm);
  } else {
    const float span = fmaxf(MOTOR_REF_SPEED_MM_S - MOTOR_MIN_LINEAR_SPEED_MM_S, 1.0f);
    const float frac = (absV - MOTOR_MIN_LINEAR_SPEED_MM_S) / span;
    mag = (int)roundf((float)minRunPwm + frac * (float)(refPwm - minRunPwm));
  }
  mag = constrain(mag, 0, DC_MOTOR_MAX_PWM);
  return (vMmS > 0.0f) ? mag : -mag;
}

int applyKickStart(int desiredPwm, int lastPwm, int minRunPwm, uint32_t now, uint32_t& kickUntil) {
  if (desiredPwm == 0) {
    kickUntil = 0;
    return 0;
  }
  if (lastPwm == 0 && abs(desiredPwm) >= minRunPwm) {
    kickUntil = now + MOTOR_KICK_DURATION_MS;
  }
  if ((int32_t)(kickUntil - now) > 0) {
    const int boost = (desiredPwm > 0) ? MOTOR_KICK_PWM_BOOST : -MOTOR_KICK_PWM_BOOST;
    return constrain(desiredPwm + boost, -DC_MOTOR_MAX_PWM, DC_MOTOR_MAX_PWM);
  }
  return desiredPwm;
}

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
  targetLinearMmS = 0.0f;
  targetAngularDps = 0.0f;
  yawRateIntegral = 0.0f;
  straightHoldActive = false;

  if (targetLeftPwm == 0 && targetRightPwm == 0) {
    motorStop();
  } else {
    motorMode = MotorMode::LegacyPwm;
  }
}

void motorSetVelocity(float linearMmPerSec, float angularDegPerSec) {
  if (drv8833IsFault()) {
    motorStop();
    return;
  }

  linearMmPerSec = constrain(linearMmPerSec, -300.0f, 300.0f);
  angularDegPerSec = constrain(angularDegPerSec, -240.0f, 240.0f);
  if (fabsf(linearMmPerSec) < 2.0f && fabsf(angularDegPerSec) < 1.0f) {
    motorStop();
    return;
  }

  digitalWrite(DRV8833_EEP_PIN, HIGH);
  if (motorMode != MotorMode::Velocity || (targetAngularDps * angularDegPerSec) < -1.0f) {
    yawRateIntegral = 0.0f;
  }

  const bool wantStraightHold = (fabsf(angularDegPerSec) < 1.0f && fabsf(linearMmPerSec) >= 5.0f);
  if (wantStraightHold) {
    if (!straightHoldActive || motorMode != MotorMode::Velocity) {
      straightRefYaw = imuYawDegrees();
      straightHoldActive = true;
    }
  } else {
    straightHoldActive = false;
  }

  targetLinearMmS = linearMmPerSec;
  targetAngularDps = angularDegPerSec;
  motorMode = MotorMode::Velocity;
}

void motorStop() {
  motorMode = MotorMode::Stopped;
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
  targetLeftPwm = 0;
  targetRightPwm = 0;
  activeLeftPwm = 0;
  activeRightPwm = 0;
  targetLinearMmS = 0.0f;
  targetAngularDps = 0.0f;
  yawRateIntegral = 0.0f;
  straightHoldActive = false;
  lastAppliedLeftPwm = 0;
  lastAppliedRightPwm = 0;
  leftKickUntil = 0;
  rightKickUntil = 0;
  analogWrite(MOTOR_LEFT_IN1, 0);
  analogWrite(MOTOR_LEFT_IN2, 0);
  analogWrite(MOTOR_RIGHT_IN1, 0);
  analogWrite(MOTOR_RIGHT_IN2, 0);
}

void motorSetRawPwm(int leftPwm, int rightPwm) {
  motorMode = MotorMode::RawPwm;
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

bool motorIsMoving() {
  if (motorMode == MotorMode::Velocity) {
    return fabsf(targetLinearMmS) >= 2.0f || fabsf(targetAngularDps) >= 1.0f;
  }
  if (motorMode == MotorMode::LegacyPwm || motorMode == MotorMode::RawPwm) {
    return (currentLeftSpeed != 0 && currentLeftSpeed != 90) ||
           (currentRightSpeed != 0 && currentRightSpeed != 90);
  }
  return false;
}

bool motorIsVelocityMode() { return motorMode == MotorMode::Velocity; }

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

  if (motorMode == MotorMode::RawPwm || motorMode == MotorMode::Stopped) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastMotorUpdate < CONTROL_PERIOD_MS) {
    return;
  }
  const float dt = constrain((now - lastMotorUpdate) / 1000.0f, 0.002f, 0.05f);
  lastMotorUpdate = now;

  if (motorMode == MotorMode::Velocity) {
    float desiredW = targetAngularDps;
    if (stabilizerEnabled && imuIsReady() && straightHoldActive) {
      const float yawErr = straightRefYaw - imuYawDegrees();
      desiredW = constrain(yawErr * IMU_STRAIGHT_HEADING_KP,
                           -IMU_STRAIGHT_HEADING_MAX_DPS, IMU_STRAIGHT_HEADING_MAX_DPS);
    }

    const float halfBase = WHEEL_BASE_MM * 0.5f;
    const float wRad = desiredW * (PI / 180.0f);
    float vLeft = targetLinearMmS - halfBase * wRad;
    float vRight = targetLinearMmS + halfBase * wRad;

    if (stabilizerEnabled && imuIsReady()) {
      const float gyroZ = imuGyroZDegreesPerSecond();
      const float wErr = desiredW - gyroZ;
      const float pTerm = IMU_YAW_RATE_KP * wErr;
      const float maxI = IMU_YAW_RATE_MAX_DIFF_MM_S / IMU_YAW_RATE_KI;
      const float nextI = constrain(yawRateIntegral + wErr * dt, -maxI, maxI);
      const float rawDiff = pTerm + IMU_YAW_RATE_KI * nextI;
      if (!((rawDiff > IMU_YAW_RATE_MAX_DIFF_MM_S && wErr > 0.0f) ||
            (rawDiff < -IMU_YAW_RATE_MAX_DIFF_MM_S && wErr < 0.0f))) {
        yawRateIntegral = nextI;
      }
      const float diffCorr = constrain(pTerm + IMU_YAW_RATE_KI * yawRateIntegral,
                                       -IMU_YAW_RATE_MAX_DIFF_MM_S,
                                       IMU_YAW_RATE_MAX_DIFF_MM_S);
      vLeft -= diffCorr;
      vRight += diffCorr;
    }

    int driveLeft = wheelSpeedToPwm(vLeft, MOTOR_LEFT_MIN_RUN_PWM, MOTOR_LEFT_REFERENCE_PWM);
    int driveRight = wheelSpeedToPwm(vRight, MOTOR_RIGHT_MIN_RUN_PWM, MOTOR_RIGHT_REFERENCE_PWM);
    driveLeft = applyKickStart(driveLeft, lastAppliedLeftPwm, MOTOR_LEFT_MIN_RUN_PWM, now, leftKickUntil);
    driveRight = applyKickStart(driveRight, lastAppliedRightPwm, MOTOR_RIGHT_MIN_RUN_PWM, now, rightKickUntil);

    driveHBridge(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, driveLeft, MOTOR_LEFT_INVERTED);
    driveHBridge(MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, driveRight, MOTOR_RIGHT_INVERTED);
    lastAppliedLeftPwm = driveLeft;
    lastAppliedRightPwm = driveRight;
    currentLeftSpeed = driveLeft;
    currentRightSpeed = driveRight;
    return;
  }

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
