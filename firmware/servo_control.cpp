#include "servo_control.h"
#include "config.h"
#include <ESP32Servo.h>
#include <math.h>

namespace {
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;

int angleToUs(int angle) {
  int clamped = constrain(angle, 0, 180);
  return SERVO_MIN_US + (int)lroundf(clamped * ((float)(SERVO_MAX_US - SERVO_MIN_US) / 180.0f));
}

int usToAngle(int us) {
  int clamped = constrain(us, SERVO_MIN_US, SERVO_MAX_US);
  return (int)lroundf((clamped - SERVO_MIN_US) * (180.0f / (float)(SERVO_MAX_US - SERVO_MIN_US)));
}

Servo servoA;
Servo servoB;
int currentUsA = angleToUs(SERVO_A_LOWER_ANGLE);
int currentUsB = angleToUs(SERVO_B_LOWER_ANGLE);
int startUsA = angleToUs(SERVO_A_LOWER_ANGLE);
int startUsB = angleToUs(SERVO_B_LOWER_ANGLE);
int targetUsA = angleToUs(SERVO_A_LOWER_ANGLE);
int targetUsB = angleToUs(SERVO_B_LOWER_ANGLE);
uint32_t moveStartMs = 0;
uint32_t moveDurationMs = SERVO_MOVE_DURATION_MS;
uint32_t lastServoUpdateMs = 0;
bool moving = false;
bool isLifted = false;
}

void servoInit() {
  servoA.setPeriodHertz(50);
  servoB.setPeriodHertz(50);
  servoA.attach(SERVO_A_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoB.attach(SERVO_B_PIN, SERVO_MIN_US, SERVO_MAX_US);

  currentUsA = startUsA = targetUsA = angleToUs(SERVO_A_LOWER_ANGLE);
  currentUsB = startUsB = targetUsB = angleToUs(SERVO_B_LOWER_ANGLE);
  moving = false;
  isLifted = false;
  servoA.writeMicroseconds(currentUsA);
  servoB.writeMicroseconds(currentUsB);
}

void servoSetAngles(int angleA, int angleB) {
  int newTargetUsA = angleToUs(angleA);
  int newTargetUsB = angleToUs(angleB);

  if (newTargetUsA == targetUsA && newTargetUsB == targetUsB) {
    return;
  }

  startUsA = currentUsA;
  startUsB = currentUsB;
  targetUsA = newTargetUsA;
  targetUsB = newTargetUsB;

  int maxDeltaUs = max(abs(targetUsA - startUsA), abs(targetUsB - startUsB));
  if (maxDeltaUs == 0) {
    moving = false;
    return;
  }

  int refStrokeUs = max(
      abs(angleToUs(SERVO_A_LIFT_ANGLE) - angleToUs(SERVO_A_LOWER_ANGLE)),
      abs(angleToUs(SERVO_B_LIFT_ANGLE) - angleToUs(SERVO_B_LOWER_ANGLE)));
  if (refStrokeUs <= 0) {
    refStrokeUs = 500;
  }

  moveDurationMs = (uint32_t)max<long>(
      SERVO_UPDATE_INTERVAL_MS,
      ((long)maxDeltaUs * (long)SERVO_MOVE_DURATION_MS) / refStrokeUs);
  moveStartMs = millis();
  lastServoUpdateMs = moveStartMs - SERVO_UPDATE_INTERVAL_MS;
  moving = true;
}

void servoLiftPallet() {
  servoSetAngles(SERVO_A_LIFT_ANGLE, SERVO_B_LIFT_ANGLE);
  isLifted = true;
}

void servoLowerPallet() {
  servoSetAngles(SERVO_A_LOWER_ANGLE, SERVO_B_LOWER_ANGLE);
  isLifted = false;
}

void servoSetPalletAction(int action) {
  if (action == 1) {
    servoLiftPallet();
  } else if (action == 2) {
    servoLowerPallet();
  }
}

bool servoIsLifted() {
  return isLifted;
}

int servoGetAngleA() {
  return usToAngle(currentUsA);
}

int servoGetAngleB() {
  return usToAngle(currentUsB);
}

void servoTask() {
  if (!moving) {
    return;
  }

  uint32_t now = millis();
  if (now - lastServoUpdateMs < SERVO_UPDATE_INTERVAL_MS) {
    return;
  }
  lastServoUpdateMs = now;

  uint32_t elapsed = now - moveStartMs;
  if (elapsed >= moveDurationMs) {
    currentUsA = targetUsA;
    currentUsB = targetUsB;
    moving = false;
  } else {
    float u = (float)elapsed / (float)moveDurationMs;
    float eased = 0.5f - 0.5f * cosf(u * 3.14159265f);
    currentUsA = startUsA + (int)lroundf((targetUsA - startUsA) * eased);
    currentUsB = startUsB + (int)lroundf((targetUsB - startUsB) * eased);
  }

  servoA.writeMicroseconds(currentUsA);
  servoB.writeMicroseconds(currentUsB);
}
