#include "motor_pulse.h"

constexpr bool TEST_RAW_PWM = true;      
constexpr int TEST_LEFT = 61;             
constexpr int TEST_RIGHT = 63;
constexpr int TEST_RUN_MS = 5000;         
constexpr uint32_t TEST_START_DELAY_MS = 5000; 

static_assert(TEST_RUN_MS >= 100 && TEST_RUN_MS <= 5000, "Run time must be 100..5000 ms");
static_assert(TEST_RAW_PWM
    ? (TEST_LEFT >= -DC_MOTOR_MAX_PWM && TEST_LEFT <= DC_MOTOR_MAX_PWM &&
       TEST_RIGHT >= -DC_MOTOR_MAX_PWM && TEST_RIGHT <= DC_MOTOR_MAX_PWM)
    : (TEST_LEFT >= 50 && TEST_LEFT <= 130 && TEST_RIGHT >= 50 && TEST_RIGHT <= 130),
    "Invalid motor command for the selected mode");

MotorPulse pulse;
SerialLine console;
uint32_t readyAt = 0;
bool attempted = false;
bool announced = false;

void command(const char* line) {
  if (!strcmp(line, "STOP")) {
    attempted = true;
    pulse.stop("STOP");
    Serial.println("Cancelled; reset/power cycle to run again.");
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  pulse.init();
  readyAt = millis();
  Serial.printf("AUTO SPEED TEST: wait %lu ms, then %s L=%d R=%d for %d ms ONCE.\n",
                (unsigned long)TEST_START_DELAY_MS, TEST_RAW_PWM?"RAW":"PROTOCOL",
                TEST_LEFT, TEST_RIGHT, TEST_RUN_MS);
  Serial.println("Runs after every reset/power-up, including after upload. No button or Serial command needed.");
}

void loop() {
  pulse.tick();
  console.poll(command);
  if (!attempted && millis()-readyAt >= TEST_START_DELAY_MS) {
    attempted = true; // Never retry automatically, including after a driver fault.
    if (!pulse.begin(TEST_LEFT, TEST_RIGHT, TEST_RUN_MS, TEST_RAW_PWM))
      Serial.println("Run rejected; remains stopped until reset.");
  }
  if (attempted && !pulse.running && !announced) {
    announced = true;
    motorStop();
    Serial.println("TEST FINISHED. Measure energized travel S; average v = S_mm * 1000 / actual_ms.");
    Serial.println("Without USB, nominal time is TEST_RUN_MS; use video for the actual moving interval.");
  }
}
