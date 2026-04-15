#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <EEPROM.h>

#define BAUD 115200

// =====================================================
// BMP180
// =====================================================
Adafruit_BMP085 bmp;
bool bmpOk = false;

// =====================================================
// AS5600
// =====================================================
#define AS5600_ADDR 0x36
bool as5600Ok = false;

// EEPROM storage for encoder zero
const int EEPROM_MAGIC_ADDR = 0;
const int EEPROM_ZERO_ADDR  = 4;
const uint32_t EEPROM_MAGIC = 0xA560BEEF;

float as5600ZeroDeg = 0.0f;
bool as5600ZeroSet = false;

bool as5600Present()
{
  Wire.beginTransmission(AS5600_ADDR);
  return (Wire.endTransmission() == 0);
}

int readAS5600Raw()
{
  // RAW ANGLE register = 0x0C (high) + 0x0D (low)
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);
  if (Wire.endTransmission(false) != 0) return -1;

  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() < 2) return -1;

  int highByte = Wire.read();
  int lowByte  = Wire.read();

  return ((highByte << 8) | lowByte) & 0x0FFF;
}

float readAS5600Deg()
{
  int raw = readAS5600Raw();
  if (raw < 0) return -999.0f;
  return (raw * 360.0f) / 4096.0f;
}

float normalizeAngle180(float deg)
{
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

float readAS5600SignedDeg()
{
  float rawDeg = readAS5600Deg();
  if (rawDeg < -100.0f) return -999.0f;

  if (!as5600ZeroSet)
    return normalizeAngle180(rawDeg);

  return normalizeAngle180(rawDeg - as5600ZeroDeg);
}

void saveEncoderZeroToEEPROM()
{
  EEPROM.put(EEPROM_MAGIC_ADDR, EEPROM_MAGIC);
  EEPROM.put(EEPROM_ZERO_ADDR, as5600ZeroDeg);
}

void loadEncoderZeroFromEEPROM()
{
  uint32_t magic = 0;
  EEPROM.get(EEPROM_MAGIC_ADDR, magic);

  if (magic == EEPROM_MAGIC)
  {
    EEPROM.get(EEPROM_ZERO_ADDR, as5600ZeroDeg);
    as5600ZeroSet = true;
  }
  else
  {
    as5600ZeroDeg = 0.0f;
    as5600ZeroSet = false;
  }
}

void clearEncoderZeroInEEPROM()
{
  uint32_t zeroMagic = 0;
  EEPROM.put(EEPROM_MAGIC_ADDR, zeroMagic);
  as5600ZeroDeg = 0.0f;
  as5600ZeroSet = false;
}

// =====================================================
// MPXV7002DP
// =====================================================
#define MPX_PIN A0

float readMPXV7002_mbar()
{
  // Average several samples for stability
  const int samples = 16;
  long sum = 0;
  for (int i = 0; i < samples; i++)
  {
    sum += analogRead(MPX_PIN);
  }

  float adc = sum / (float)samples;

  // Convert ADC to voltage (assuming 5.0V Arduino analog reference)
  float voltage = adc * (5.0f / 1023.0f);

  // MPXV7002DP transfer function:
  // Vout = Vs * (0.2 * P + 0.5), with P in kPa and Vs = 5V
  // => P(kPa) = (Vout / Vs - 0.5) / 0.2
  float pressure_kPa = ((voltage / 5.0f) - 0.5f) / 0.2f;

  // Convert kPa to mbar
  float pressure_mbar = pressure_kPa * 10.0f;

  return pressure_mbar;
}

// =====================================================
// FAN SETUP
// =====================================================
const int FAN_PWM_PIN[4] = {3, 5, 6, 9};
int fanPct[4] = {0, 0, 0, 0};
int fanAll = 0;

int clampPct(int v)
{
  if (v < 0) return 0;
  if (v > 100) return 100;
  return v;
}

void setFan(int idx, int pct)
{
  pct = clampPct(pct);
  fanPct[idx] = pct;
  int duty = map(pct, 0, 100, 0, 255);
  analogWrite(FAN_PWM_PIN[idx], duty);
}

void setAllFans(int pct)
{
  pct = clampPct(pct);
  fanAll = pct;
  for (int i = 0; i < 4; i++) setFan(i, pct);
}

// =====================================================
// STEPPER SETUP (A4988 STEP / DIR / EN)
// =====================================================
#define STEP_DIR_PIN   2
#define STEP_STEP_PIN  4
#define STEP_EN_PIN    7

volatile long step_pos = 0;
volatile long step_target = 0;

bool step_enabled = true;
bool step_moving = false;

int step_sps = 800;
unsigned long step_interval_us = 0;
unsigned long last_step_us = 0;

void stepSetEnable(bool en)
{
  step_enabled = en;
  digitalWrite(STEP_EN_PIN, en ? LOW : HIGH); // A4988 EN is active LOW
  if (!en) step_moving = false;
}

int clampSps(int v)
{
  if (v < 10) return 10;
  if (v > 4000) return 4000;
  return v;
}

void stepSetSpeedSps(int sps)
{
  step_sps = clampSps(sps);
  step_interval_us = (unsigned long)(1000000UL / (unsigned long)step_sps);
  if (step_interval_us < 200) step_interval_us = 200;
}

void stepStop()
{
  step_target = step_pos;
  step_moving = false;
}

void stepMoveRelative(long steps, int sps)
{
  if (!step_enabled) return;
  stepSetSpeedSps(sps);
  step_target = step_pos + steps;
  step_moving = (step_target != step_pos);
}

void stepGotoAbsolute(long pos, int sps)
{
  if (!step_enabled) return;
  stepSetSpeedSps(sps);
  step_target = pos;
  step_moving = (step_target != step_pos);
}

void stepperUpdate()
{
  if (!step_enabled) return;
  if (!step_moving) return;

  long cur = step_pos;
  long tgt = step_target;

  if (cur == tgt)
  {
    step_moving = false;
    return;
  }

  bool dir = (tgt > cur);
  digitalWrite(STEP_DIR_PIN, dir ? HIGH : LOW);

  unsigned long now = micros();
  if ((unsigned long)(now - last_step_us) < step_interval_us) return;
  last_step_us = now;

  digitalWrite(STEP_STEP_PIN, HIGH);
  delayMicroseconds(3);
  digitalWrite(STEP_STEP_PIN, LOW);

  step_pos += dir ? 1 : -1;

  if (step_pos == step_target) step_moving = false;
}

// =====================================================
// COMMAND PARSING
// =====================================================
void handleLine(char* line)
{
  while (*line == ' ' || *line == '\t') line++;
  if (!*line) return;

  // ---------------- FAN STOP ----------------
  if (strcmp(line, "FAN STOP") == 0)
  {
    setAllFans(0);
    Serial.println("OK FAN STOP");
    return;
  }

  // ---------------- FAN ----------------
  {
    char which[8] = {0};
    int pct = 0;

    if (sscanf(line, "FAN %7s %d", which, &pct) == 2)
    {
      if (strcmp(which, "ALL") == 0)
      {
        setAllFans(pct);
        Serial.print("OK FAN ALL ");
        Serial.println(fanAll);
        return;
      }

      int n = atoi(which);
      if (n >= 1 && n <= 4)
      {
        setFan(n - 1, pct);
        Serial.print("OK FAN ");
        Serial.print(n);
        Serial.print(" ");
        Serial.println(clampPct(pct));
        return;
      }
    }
  }

  // ---------------- STEPPER STOP ----------------
  if (strcmp(line, "STEPPER STOP") == 0)
  {
    stepStop();
    Serial.println("OK STEPPER STOP");
    return;
  }

  // ---------------- STEPPER EN ----------------
  {
    int enVal = -1;
    if (sscanf(line, "STEPPER EN %d", &enVal) == 1)
    {
      stepSetEnable(enVal != 0);
      Serial.print("OK STEPPER EN ");
      Serial.println(step_enabled ? 1 : 0);
      return;
    }
  }

  // ---------------- STEPPER MOVE ----------------
  {
    long steps = 0;
    int sps = 0;
    if (sscanf(line, "STEPPER MOVE %ld %d", &steps, &sps) == 2)
    {
      stepMoveRelative(steps, sps);
      Serial.print("OK STEPPER MOVE target=");
      Serial.print(step_target);
      Serial.print(" sps=");
      Serial.println(step_sps);
      return;
    }
  }

  // ---------------- STEPPER GOTO ----------------
  {
    long pos = 0;
    int sps = 0;
    if (sscanf(line, "STEPPER GOTO %ld %d", &pos, &sps) == 2)
    {
      stepGotoAbsolute(pos, sps);
      Serial.print("OK STEPPER GOTO target=");
      Serial.print(step_target);
      Serial.print(" sps=");
      Serial.println(step_sps);
      return;
    }
  }

  // ---------------- ENCODER ZERO ----------------
  if (strcmp(line, "ENCODER ZERO") == 0)
  {
    float rawDeg = readAS5600Deg();
    if (rawDeg < -100.0f)
    {
      Serial.println("ERR ENCODER ZERO");
      return;
    }

    as5600ZeroDeg = rawDeg;
    as5600ZeroSet = true;
    saveEncoderZeroToEEPROM();

    // Reset logical step position too
    step_pos = 0;
    step_target = 0;
    step_moving = false;

    Serial.println("OK ENCODER ZERO");
    return;
  }

  // ---------------- ENCODER CLEAR ----------------
  if (strcmp(line, "ENCODER CLEAR") == 0)
  {
    clearEncoderZeroInEEPROM();
    Serial.println("OK ENCODER CLEAR");
    return;
  }

  Serial.print("ERR ");
  Serial.println(line);
}

// =====================================================
// SETUP
// =====================================================
void setup()
{
  Serial.begin(BAUD);
  Wire.begin();

  // Fans
  for (int i = 0; i < 4; i++)
  {
    pinMode(FAN_PWM_PIN[i], OUTPUT);
    analogWrite(FAN_PWM_PIN[i], 0);
  }

  // MPX
  pinMode(MPX_PIN, INPUT);

  // Stepper
  pinMode(STEP_DIR_PIN, OUTPUT);
  pinMode(STEP_STEP_PIN, OUTPUT);
  pinMode(STEP_EN_PIN, OUTPUT);

  digitalWrite(STEP_STEP_PIN, LOW);
  digitalWrite(STEP_DIR_PIN, LOW);
  stepSetEnable(true);
  stepSetSpeedSps(step_sps);

  // BMP180
  bmpOk = bmp.begin();
  if (bmpOk) Serial.println("OK BMP180 FOUND");
  else       Serial.println("WARN BMP180 NOT FOUND");

  // AS5600
  as5600Ok = as5600Present();
  if (as5600Ok) Serial.println("OK AS5600 FOUND");
  else          Serial.println("WARN AS5600 NOT FOUND");

  // Load saved encoder zero
  loadEncoderZeroFromEEPROM();
  if (as5600ZeroSet) Serial.println("OK ENCODER ZERO LOADED");
  else               Serial.println("WARN ENCODER ZERO NOT SET");

  Serial.println("READY");
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  // Stepper update
  stepperUpdate();

  // ---------- Serial input ----------
  static char buf[96];
  static byte idx = 0;

  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\r') continue;

    if (c == '\n')
    {
      buf[idx] = 0;
      handleLine(buf);
      idx = 0;
    }
    else if (idx < sizeof(buf) - 1)
    {
      buf[idx++] = c;
    }
  }

  // ---------- Telemetry ----------
  static unsigned long last = 0;
  unsigned long now = millis();

  if (now - last >= 200)
  {
    last = now;

    // Fan RPM still simulated for now
    int rpm1 = fanPct[0] * 40;
    int rpm2 = fanPct[1] * 40;
    int rpm3 = fanPct[2] * 40;
    int rpm4 = fanPct[3] * 40;

    // MPX real in mbar
    float mpx_mbar = readMPXV7002_mbar();

    // BMP180 real
    float bmp_p = 0.0f;
    float bmp_t = 0.0f;
    if (bmpOk)
    {
      bmp_t = bmp.readTemperature();
      bmp_p = bmp.readPressure() / 100.0f; // hPa = mbar
    }

    // AS5600 real, signed relative to saved home
    float as5600 = -999.0f;
    if (as5600Ok)
    {
      as5600 = readAS5600SignedDeg();
    }

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

    Serial.print(" mpx_adc="); Serial.print(mpx_mbar, 1);
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
