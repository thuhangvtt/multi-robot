
#include "../config.h"
#include "../motor.h"
#include "serial_line.h"
#include <ESP32Servo.h>

namespace {
Servo servoA;
Servo servoB;
SerialLine console;

bool armed = false;
float curA = 45, curB = 45;
float fromA = 45, fromB = 45;
int tgtA = 45, tgtB = 45;
uint32_t moveStart = 0, moveDur = 0;
bool moving = false;

int restA = 45, restB = 45;
int gripA = 90, gripB = 90;

int cyclesLeft = 0;
bool cycleToGrip = true;
uint32_t holdUntil = 0;
const uint32_t CYCLE_HOLD_MS = 800;
const uint32_t DEFAULT_MOVE_MS = 600;

int clampAngle(int v) { return v < 0 ? 0 : (v > 180 ? 180 : v); }

void writeNow() {
  servoA.write((int)(curA + 0.5f));
  servoB.write((int)(curB + 0.5f));
}

void startMove(int a, int b, uint32_t ms) {
  tgtA = clampAngle(a);
  tgtB = clampAngle(b);
  if (ms == 0) {
    curA = fromA = tgtA;
    curB = fromB = tgtB;
    moving = false;
    writeNow();
    Serial.printf("OK a=%d b=%d (instant)\n", tgtA, tgtB);
    return;
  }
  fromA = curA;
  fromB = curB;
  moveStart = millis();
  moveDur = ms;
  moving = true;
}

void updateMove() {
  if (!moving)
    return;
  uint32_t t = millis() - moveStart;
  float f = t >= moveDur ? 1.0f : (float)t / (float)moveDur;
  curA = fromA + (tgtA - fromA) * f;
  curB = fromB + (tgtB - fromB) * f;
  writeNow();
  if (f >= 1.0f) {
    moving = false;
    Serial.printf("DONE a=%d b=%d (%lu ms)\n", tgtA, tgtB,
                  (unsigned long)moveDur);
    if (cyclesLeft > 0)
      holdUntil = millis() + CYCLE_HOLD_MS;
  }
}

void updateCycle() {
  if (cyclesLeft <= 0 || moving || millis() < holdUntil)
    return;
  if (cycleToGrip) {
    startMove(gripA, gripB, DEFAULT_MOVE_MS);
    cycleToGrip = false;
  } else {
    startMove(restA, restB, DEFAULT_MOVE_MS);
    cycleToGrip = true;
    if (--cyclesLeft == 0)
      Serial.println("CYCLE finished");
  }
}

void printHelp() {
  Serial.println("ARM [a b]        gan servo va dua ve a,b (mac dinh 45 45). "
                 "CANG PHAI TU DO, CHUA CO PALLET");
  Serial.println("OFF              nha servo (buong luc de xoay tay)");
  Serial.println("A <deg> [ms]     chi servo A    | B <deg> [ms]  chi servo B");
  Serial.println(
      "BOTH <a> <b> [ms] ca hai cung luc (ms=0 la nhay ngay, mac dinh 600)");
  Serial.println("NUDGE A|B <+-deg> nhich tung do de tim goc cang vuong goc / "
                 "cham pallet");
  Serial.println("SET REST <a> <b> | SET GRIP <a> <b>   chot goc nghi / goc "
                 "gap tung servo");
  Serial.println("REST [ms] | GRIP [ms]   di toi goc da chot");
  Serial.println("CYCLE <n>        lap REST->GRIP->REST n lan | STOP dung moi "
                 "chuyen dong");
  Serial.println(
      "STATUS | CFG     xem goc / in dong dan vao firmware/config.h | HELP");
}

void printStatus() {
  Serial.printf("ROBOT=%u armed=%d A(pin%u)=%d B(pin%u)=%d moving=%d\n",
                ROBOT_ID, armed, SERVO_A_PIN, (int)(curA + 0.5f), SERVO_B_PIN,
                (int)(curB + 0.5f), moving);
  Serial.printf("REST A=%d B=%d | GRIP A=%d B=%d\n", restA, restB, gripA,
                gripB);
}

void printConfig() {
  Serial.println(
      "--- dan vao firmware/config.h (LOWER = nghi, LIFT = gap) ---");
  Serial.printf("constexpr int SERVO_A_LOWER_ANGLE = %d;\n", restA);
  Serial.printf("constexpr int SERVO_A_LIFT_ANGLE = %d;\n", gripA);
  Serial.printf("constexpr int SERVO_B_LOWER_ANGLE = %d;\n", restB);
  Serial.printf("constexpr int SERVO_B_LIFT_ANGLE = %d;\n", gripB);
  Serial.println("--------------------------------------------------------");
}

bool needArm() {
  if (armed)
    return false;
  Serial.println("Chua ARM. Go: ARM  (se gan servo va dua ve goc nghi)");
  return true;
}

void command(const char *raw) {
  char line[96];
  size_t n = strlen(raw);
  if (n >= sizeof(line))
    n = sizeof(line) - 1;
  for (size_t i = 0; i < n; ++i)
    line[i] = toupper((unsigned char)raw[i]);
  line[n] = 0;

  int a, b, ms, c;
  char which;

  if (!strcmp(line, "HELP")) {
    printHelp();
    return;
  }
  if (!strcmp(line, "STATUS")) {
    printStatus();
    return;
  }
  if (!strcmp(line, "CFG")) {
    printConfig();
    return;
  }

  if (!strncmp(line, "ARM", 3)) {
    int sa = 45, sb = 45;
    int got = sscanf(line, "ARM %d %d", &sa, &sb);
    if (got != 2) {
      sa = 45;
      sb = 45;
    }
    sa = clampAngle(sa);
    sb = clampAngle(sb);
    if (!armed) {
      servoA.setPeriodHertz(50);
      servoB.setPeriodHertz(50);
      servoA.attach(SERVO_A_PIN, 500, 2500);
      servoB.attach(SERVO_B_PIN, 500, 2500);
      armed = true;
    }
    curA = fromA = tgtA = sa;
    curB = fromB = tgtB = sb;
    moving = false;
    cyclesLeft = 0;
    writeNow();
    Serial.printf("ARMED a=%d b=%d. Danh dau servo nao la A (chan %u), servo "
                  "nao la B (chan %u).\n",
                  sa, sb, SERVO_A_PIN, SERVO_B_PIN);
    return;
  }

  if (!strcmp(line, "OFF")) {
    if (armed) {
      servoA.detach();
      servoB.detach();
      armed = false;
    }
    moving = false;
    cyclesLeft = 0;
    Serial.println("OK servo released (co the xoay tay). Dung ARM de gan lai.");
    return;
  }

  if (!strcmp(line, "STOP")) {
    moving = false;
    cyclesLeft = 0;
    tgtA = (int)(curA + 0.5f);
    tgtB = (int)(curB + 0.5f);
    Serial.printf("STOPPED at a=%d b=%d\n", tgtA, tgtB);
    return;
  }

  if (!strncmp(line, "SET ", 4)) {
    if (sscanf(line, "SET REST %d %d", &a, &b) == 2) {
      restA = clampAngle(a);
      restB = clampAngle(b);
      Serial.printf("OK REST A=%d B=%d\n", restA, restB);
    } else if (sscanf(line, "SET GRIP %d %d", &a, &b) == 2) {
      gripA = clampAngle(a);
      gripB = clampAngle(b);
      Serial.printf("OK GRIP A=%d B=%d\n", gripA, gripB);
    } else {
      Serial.println("Dung: SET REST <a> <b> | SET GRIP <a> <b>");
    }
    return;
  }

  if (needArm())
    return;

  if (sscanf(line, "NUDGE %c %d", &which, &a) == 2 &&
      (which == 'A' || which == 'B')) {
    cyclesLeft = 0;
    if (which == 'A')
      startMove((int)(curA + 0.5f) + a, (int)(curB + 0.5f), 150);
    else
      startMove((int)(curA + 0.5f), (int)(curB + 0.5f) + a, 150);
    return;
  }
  if (!strncmp(line, "NUDGE", 5)) {
    Serial.println("Dung: NUDGE A|B <+-deg>");
    return;
  }

  if (!strncmp(line, "BOTH", 4)) {
    c = sscanf(line, "BOTH %d %d %d", &a, &b, &ms);
    if (c < 2) {
      Serial.println("Dung: BOTH <a> <b> [ms]");
      return;
    }
    cyclesLeft = 0;
    startMove(a, b, c == 3 ? (uint32_t)max(ms, 0) : DEFAULT_MOVE_MS);
    return;
  }

  if (sscanf(line, "A %d %d", &a, &ms) >= 1 && line[0] == 'A' &&
      line[1] == ' ') {
    c = sscanf(line, "A %d %d", &a, &ms);
    cyclesLeft = 0;
    startMove(a, (int)(curB + 0.5f),
              c == 2 ? (uint32_t)max(ms, 0) : DEFAULT_MOVE_MS);
    return;
  }
  if (sscanf(line, "B %d %d", &b, &ms) >= 1 && line[0] == 'B' &&
      line[1] == ' ') {
    c = sscanf(line, "B %d %d", &b, &ms);
    cyclesLeft = 0;
    startMove((int)(curA + 0.5f), b,
              c == 2 ? (uint32_t)max(ms, 0) : DEFAULT_MOVE_MS);
    return;
  }

  if (!strncmp(line, "REST", 4) || !strncmp(line, "GRIP", 4)) {
    bool grip = line[0] == 'G';
    c = sscanf(line + 4, "%d", &ms);
    cyclesLeft = 0;
    if (grip)
      startMove(gripA, gripB, c == 1 ? (uint32_t)max(ms, 0) : DEFAULT_MOVE_MS);
    else
      startMove(restA, restB, c == 1 ? (uint32_t)max(ms, 0) : DEFAULT_MOVE_MS);
    return;
  }

  if (sscanf(line, "CYCLE %d", &a) == 1 && a > 0 && a <= 50) {
    cyclesLeft = a;
    cycleToGrip = true;
    holdUntil = 0;
    Serial.printf("CYCLE %d lan (dung bang STOP)\n", a);
    return;
  }

  Serial.println("Lenh khong hop le. Go HELP.");
}
} // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  motorInit();
  motorStop();
  Serial.printf(
      "TEST 5 SERVO ROBOT=%u: banh dung, servo CHUA gan. Go HELP, roi ARM.\n",
      ROBOT_ID);
}

void loop() {
  motorStop();
  console.poll(command);
  if (armed) {
    updateMove();
    updateCycle();
  }
}
