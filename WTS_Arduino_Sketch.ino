#include <Wire.h>
#include <Adafruit_BMP085.h>

// Arduino UNO - FAN control + telemetry + STEPPER control (A4988 style STEP/DIR/EN)
//
// Commands:
//   FAN ALL <0-100>
//   FAN <1-4> <0-100>
//   FAN STOP
//
//   STEPPER MOVE <steps> <sps>     // relative move, steps can be negative, sps = steps/sec (e.g. 800)
//   STEPPER GOTO <pos> <sps>       // absolute position in steps
//   STEPPER STOP
//   STEPPER EN <0|1>               // 1=enable (EN low), 0=disable (EN high)
//
// Telemetry (5Hz):
//   T fanAll=.. fan1=.. fan2=.. fan3=.. fan4=.. rpm1=.. rpm2=.. rpm3=.. rpm4=.. mpx_adc=.. bmp_p=.. bmp_t=.. as5600=.. step_pos=.. step_target=.. step_en=.. step_moving=.. step_sps=..

#define BAUD 115200

// ---------- BMP180 ----------
Adafruit_BMP085 bmp;
bool bmpOk = false;

// ---------- FAN SETUP ----------
const int FAN_PWM_PIN[4] = {3, 5, 6, 9};  // keep your existing fan pins
int fanPct[4] = {0, 0, 0, 0};
int fanAll = 0;

int clampPct(int v) { if (v < 0) return 0; if (v > 100) return 100; return v; }

void setFan(int idx, int pct) {
  pct = clampPct(pct);
  fanPct[idx] = pct;
  int duty = map(pct, 0, 100, 0, 255);
  analogWrite(FAN_PWM_PIN[idx], duty);
}

void setAllFans(int pct) {
  pct = clampPct(pct);
  fanAll = pct;
  for (int i = 0; i < 4; i++) setFan(i, pct);
}

// ---------- STEPPER SETUP (A4988 STEP/DIR/EN) ----------
#define STEP_DIR_PIN   2
#define STEP_STEP_PIN  4
#define STEP_EN_PIN    7    // optional; A4988 EN is ACTIVE LOW

volatile long step_pos = 0;       // current position in steps
volatile long step_target = 0;    // target position in steps

bool step_enabled = true;
bool step_moving = false;

int step_sps = 800;              // steps/sec (commanded)
unsigned long step_interval_us = 0;
unsigned long last_step_us = 0;

// set enable (A4988 EN is active LOW)
void stepSetEnable(bool en) {
  step_enabled = en;
  digitalWrite(STEP_EN_PIN, en ? LOW : HIGH);
  if (!en) step_moving = false;
}

int clampSps(int v) {
  if (v < 10) return 10;
  if (v > 4000) return 4000; 
  return v;
}

void stepSetSpeedSps(int sps) {
  step_sps = clampSps(sps);
  step_interval_us = (unsigned long)(1000000UL / (unsigned long)step_sps);
  if (step_interval_us < 200) step_interval_us = 200; // keep pulses sane
}

void stepStop() {
  step_target = step_pos;
  step_moving = false;
}

void stepMoveRelative(long steps, int sps) {
  if (!step_enabled) return;
  stepSetSpeedSps(sps);
  step_target = step_pos + steps;
  step_moving = (step_target != step_pos);
}

void stepGotoAbsolute(long pos, int sps) {
  if (!step_enabled) return;
  stepSetSpeedSps(sps);
  step_target = pos;
  step_moving = (step_target != step_pos);
}

// Non-blocking stepper update (call often)
void stepperUpdate() {
  if (!step_enabled) return;
  if (!step_moving) return;

  long cur = step_pos;
  long tgt = step_target;
  if (cur == tgt) {
    step_moving = false;
    return;
  }

  // direction
  bool dir = (tgt > cur);
  digitalWrite(STEP_DIR_PIN, dir ? HIGH : LOW);

  unsigned long now = micros();
  if ((unsigned long)(now - last_step_us) < step_interval_us) return;
  last_step_us = now;

  // STEP pulse (A4988 needs >= 1us high; we give ~3us)
  digitalWrite(STEP_STEP_PIN, HIGH);
  delayMicroseconds(3);
  digitalWrite(STEP_STEP_PIN, LOW);

  // update position
  step_pos += dir ? 1 : -1;

  // stop condition
  if (step_pos == step_target) step_moving = false;
}

// ---------- COMMAND PARSING ----------
void handleLine(char* line) {
  while (*line == ' ' || *line == '\t') line++;
  if (!*line) return;

  // FAN STOP
  if (strcmp(line, "FAN STOP") == 0) {
    setAllFans(0);
    Serial.println("OK FAN STOP");
    return;
  }

  // FAN ...
  {
    char which[8] = {0};
    int pct = 0;
    if (sscanf(line, "FAN %7s %d", which, &pct) == 2) {
      if (strcmp(which, "ALL") == 0) {
        setAllFans(pct);
        Serial.print("OK FAN ALL ");
        Serial.println(fanAll);
        return;
      }
      int n = atoi(which);
      if (n >= 1 && n <= 4) {
        setFan(n - 1, pct);
        Serial.print("OK FAN ");
        Serial.print(n);
        Serial.print(" ");
        Serial.println(clampPct(pct));
        return;
      }
    }
  }

  // STEPPER STOP
  if (strcmp(line, "STEPPER STOP") == 0) {
    stepStop();
    Serial.println("OK STEPPER STOP");
    return;
  }

  // STEPPER EN <0|1>
  {
    int enVal = -1;
    if (sscanf(line, "STEPPER EN %d", &enVal) == 1) {
      stepSetEnable(enVal != 0);
      Serial.print("OK STEPPER EN ");
      Serial.println(step_enabled ? 1 : 0);
      return;
    }
  }

  // STEPPER MOVE <steps> <sps>
  {
    long steps = 0;
    int sps = 0;
    if (sscanf(line, "STEPPER MOVE %ld %d", &steps, &sps) == 2) {
      stepMoveRelative(steps, sps);
      Serial.print("OK STEPPER MOVE target=");
      Serial.print(step_target);
      Serial.print(" sps=");
      Serial.println(step_sps);
      return;
    }
  }

  // STEPPER GOTO <pos> <sps>
  {
    long pos = 0;
    int sps = 0;
    if (sscanf(line, "STEPPER GOTO %ld %d", &pos, &sps) == 2) {
      stepGotoAbsolute(pos, sps);
      Serial.print("OK STEPPER GOTO target=");
      Serial.print(step_target);
      Serial.print(" sps=");
      Serial.println(step_sps);
      return;
    }
  }

  Serial.print("ERR ");
  Serial.println(line);
}

void setup() {
  Serial.begin(BAUD);
  Wire.begin();

  // Fans
  for (int i = 0; i < 4; i++) {
    pinMode(FAN_PWM_PIN[i], OUTPUT);
    analogWrite(FAN_PWM_PIN[i], 0);
  }

  // Stepper pins
  pinMode(STEP_DIR_PIN, OUTPUT);
  pinMode(STEP_STEP_PIN, OUTPUT);
  pinMode(STEP_EN_PIN, OUTPUT);

  digitalWrite(STEP_STEP_PIN, LOW);
  digitalWrite(STEP_DIR_PIN, LOW);
  stepSetEnable(true);
  stepSetSpeedSps(step_sps);

  // BMP180
  bmpOk = bmp.begin();
  if (bmpOk) {
    Serial.println("OK BMP180 FOUND");
  } else {
    Serial.println("WARN BMP180 NOT FOUND");
  }

  Serial.println("READY");
}

void loop() {
  // Update stepper frequently (non-blocking)
  stepperUpdate();

  // Serial input
  static char buf[96];
  static byte idx = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[idx] = 0;
      handleLine(buf);
      idx = 0;
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }

  // Telemetry at 5Hz
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last >= 200) {
    last = now;

    // Simulated RPM (until real tach)
    int rpm1 = fanPct[0] * 40;
    int rpm2 = fanPct[1] * 40;
    int rpm3 = fanPct[2] * 40;
    int rpm4 = fanPct[3] * 40;

    // MPX still fake for now
    int mpx_adc = (now / 10) % 1024;

    // BMP180 readings
    float bmp_p = 0.0f;
    float bmp_t = 0.0f;

    if (bmpOk) {
      bmp_t = bmp.readTemperature();
      bmp_p = bmp.readPressure() / 100.0f; // hPa
    }

    // AS5600 still fake for now
    float as5600 = (now / 100) % 360;

    Serial.print("T ");
    Serial.print("fanAll="); Serial.print(fanAll);
    Serial.print(" fan1=");  Serial.print(fanPct[0]);
    Serial.print(" fan2=");  Serial.print(fanPct[1]);
    Serial.print(" fan3=");  Serial.print(fanPct[2]);
    Serial.print(" fan4=");  Serial.print(fanPct[3]);

    Serial.print(" rpm1=");  Serial.print(rpm1);
    Serial.print(" rpm2=");  Serial.print(rpm2);
    Serial.print(" rpm3=");  Serial.print(rpm3);
    Serial.print(" rpm4=");  Serial.print(rpm4);

    Serial.print(" mpx_adc="); Serial.print(mpx_adc);
    Serial.print(" bmp_p=");   Serial.print(bmp_p, 1);
    Serial.print(" bmp_t=");   Serial.print(bmp_t, 1);
    Serial.print(" as5600=");  Serial.print(as5600, 1);

    Serial.print(" step_pos=");    Serial.print(step_pos);
    Serial.print(" step_target="); Serial.print(step_target);
    Serial.print(" step_en=");     Serial.print(step_enabled ? 1 : 0);
    Serial.print(" step_moving="); Serial.print(step_moving ? 1 : 0);
    Serial.print(" step_sps=");    Serial.println(step_sps);
  }
}
