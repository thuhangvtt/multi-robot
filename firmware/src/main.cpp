#include <Arduino.h>

namespace {
constexpr uint8_t MOTOR_LEFT_IN1 = 26;
constexpr uint8_t MOTOR_LEFT_IN2 = 25;
constexpr uint8_t MOTOR_RIGHT_IN1 = 33;
constexpr uint8_t MOTOR_RIGHT_IN2 = 32;
constexpr uint8_t DRV8833_SLEEP_PIN = 15;
constexpr uint8_t DRV8833_FAULT_PIN = 5;
constexpr bool MOTOR_LEFT_INVERTED = true;
constexpr bool MOTOR_RIGHT_INVERTED = true;
constexpr int MOTOR_RIGHT_FORWARD_BOOST = 0;

constexpr int SERVO_NEUTRAL = 90;
constexpr int SIGNAL_MIN = 40;
constexpr int SIGNAL_MAX = 140;
constexpr int MOTOR_MIN_PWM = 40;
constexpr int MOTOR_MAX_PWM = 120;
constexpr uint32_t DEFAULT_TEST_MS = 1000;
constexpr uint32_t MAX_TEST_MS = 30000;
constexpr uint32_t COMMAND_TIMEOUT_MS = 2000;
constexpr bool AUTO_START_TEST = true;
constexpr int AUTO_START_LEFT_SIGNAL = 90;
constexpr int AUTO_START_RIGHT_SIGNAL = 90;
constexpr uint32_t AUTO_START_DELAY_MS = 5000;
constexpr uint32_t AUTO_START_DURATION_MS = 20000;

char inputLine[80];
uint8_t inputLength = 0;
int activeLeft = 0;
int activeRight = 0;
uint32_t stopAt = 0;
uint32_t autoStartAt = 0;
bool autoStartPending = false;

int signalToPwm(int signal) {
  if (signal == 0) return 0;
  if (signal < SIGNAL_MIN || signal > SIGNAL_MAX) {
    return constrain(signal, -255, 255);
  }

  const int delta = signal - SERVO_NEUTRAL;
  if (abs(delta) <= 1) return 0;

  const int magnitude = map(abs(delta), 2, 40, MOTOR_MIN_PWM, MOTOR_MAX_PWM);
  return delta > 0 ? magnitude : -magnitude;
}

int compensateRightSignal(int signal) {
  if (signal > SERVO_NEUTRAL) {
    return min(signal + MOTOR_RIGHT_FORWARD_BOOST, SIGNAL_MAX);
  }
  return signal;
}

void writeMotor(uint8_t pin1, uint8_t pin2, int pwm, bool inverted) {
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

void stopMotors() {
  writeMotor(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, 0, MOTOR_LEFT_INVERTED);
  writeMotor(MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, 0, MOTOR_RIGHT_INVERTED);
  activeLeft = 0;
  activeRight = 0;
  stopAt = 0;
}

void runMotors(int left, int right, uint32_t durationMs) {
  const int compensatedRight = compensateRightSignal(right);
  const int leftPwm = signalToPwm(left);
  const int rightPwm = signalToPwm(compensatedRight);

  digitalWrite(DRV8833_SLEEP_PIN, HIGH);
  writeMotor(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, leftPwm, MOTOR_LEFT_INVERTED);
  writeMotor(MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, rightPwm, MOTOR_RIGHT_INVERTED);
  activeLeft = left;
  activeRight = right;
  stopAt = millis() + min(durationMs, MAX_TEST_MS);

  Serial.print("RUN left_signal=");
  Serial.print(left);
  Serial.print(" right_signal=");
  Serial.print(compensatedRight);
  Serial.print(" left_pwm=");
  Serial.print(leftPwm);
  Serial.print(" right_pwm=");
  Serial.print(rightPwm);
  Serial.print(" duration_ms=");
  Serial.println(min(durationMs, MAX_TEST_MS));
}

void printHelp() {
  Serial.println();
  Serial.println("=== ROBOT CALIBRATION FIRMWARE ===");
  Serial.println("M <left> <right> [ms]  Test tin hieu 40..140, mac dinh 1000 ms");
  Serial.println("RUN <left> <right> <ms> Chay theo thoi gian, tu dung sau khi het");
  Serial.println("AUTO <signal> <ms>      Chay thang tu dong, khong can giu USB");
  Serial.println("STOP                    Dung dong co ngay");
  Serial.println("STATUS                  Xem tin hieu, PWM va loi DRV8833");
  Serial.println("HELP                    Hien thi huong dan");
  Serial.println();
  Serial.println("Vi du: M 92 92 1000 | M 100 100 5000 | M 88 88 1000");
  Serial.println("Trung tinh=90; tien >90; lui <90. Khong dat tay vao banh.");
}

void handleCommand(char *line) {
  int left = 0;
  int right = 0;
  unsigned long duration = DEFAULT_TEST_MS;

  if (sscanf(line, "M %d %d %lu", &left, &right, &duration) >= 2) {
    runMotors(left, right, duration);
  } else if (sscanf(line, "RUN %d %d %lu", &left, &right, &duration) == 3) {
    runMotors(left, right, duration);
  } else if (sscanf(line, "AUTO %d %lu", &left, &duration) == 2) {
    runMotors(left, left, duration);
  } else if (strcmp(line, "STOP") == 0) {
    autoStartPending = false;
    stopMotors();
    Serial.println("OK STOP");
  } else if (strcmp(line, "STATUS") == 0) {
    Serial.print("STATUS signalL=");
    Serial.print(activeLeft);
    Serial.print(" signalR=");
    Serial.print(activeRight);
    Serial.print(" pwmL=");
    Serial.print(signalToPwm(activeLeft));
    Serial.print(" pwmR=");
    Serial.print(signalToPwm(activeRight));
    Serial.print(" fault=");
    Serial.println(digitalRead(DRV8833_FAULT_PIN) == LOW ? "YES" : "NO");
  } else if (strcmp(line, "HELP") == 0) {
    printHelp();
  } else {
    Serial.println("ERROR lenh khong hop le; go HELP");
  }
}
}

void setup() {
  Serial.begin(115200);
  pinMode(DRV8833_SLEEP_PIN, OUTPUT);
  pinMode(DRV8833_FAULT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  digitalWrite(DRV8833_SLEEP_PIN, HIGH);
  stopMotors();
  autoStartAt = millis() + AUTO_START_DELAY_MS;
  autoStartPending = AUTO_START_TEST;
  Serial.println("Calibration firmware ready");
  printHelp();
  if (AUTO_START_TEST) {
    Serial.println("AUTO START: 5 giay nua se chay trai=100 phai=105 trong 20 giay");
    Serial.println("Dat robot tren san, san sang nut dung khan cap.");
  }
}

void loop() {
  if (autoStartPending && static_cast<int32_t>(millis() - autoStartAt) >= 0) {
    autoStartPending = false;
    runMotors(AUTO_START_LEFT_SIGNAL, AUTO_START_RIGHT_SIGNAL,
              AUTO_START_DURATION_MS);
  }

  if (stopAt != 0 && static_cast<int32_t>(millis() - stopAt) >= 0) {
    stopMotors();
    Serial.println("DONE STOP");
  }

  if (digitalRead(DRV8833_FAULT_PIN) == LOW) {
    stopMotors();
    Serial.println("FAULT STOP");
  }

  while (Serial.available() > 0) {
    const char character = static_cast<char>(Serial.read());
    if (character == '\n' || character == '\r') {
      if (inputLength == 0) continue;
      inputLine[inputLength] = '\0';
      handleCommand(inputLine);
      inputLength = 0;
    } else if (inputLength < sizeof(inputLine) - 1) {
      inputLine[inputLength++] = character;
    } else {
      inputLength = 0;
      Serial.println("ERROR lenh qua dai");
    }
  }
}
