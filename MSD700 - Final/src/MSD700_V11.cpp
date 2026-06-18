/******************************************************************************
 * File        : MSD700_V11.cpp
 * Project     : MSD700 main Controller
 *
 * Description :
 *   Main firmware for the MSD700 system controller
 *
 * Communication Interfaces:
 *     - Serial1 : SBUS Receiver
 *     - Serial2 : Left Motor Driver (RS485)
 *     - Serial3 : Right Motor Driver (RS485)
 *     - Serial  : Mini PC Communication
 ******************************************************************************/
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ModbusRTU.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "Odometry.h"
#include "DiffDrive.h"

#define SBUS_FRAME_SIZE 25
LiquidCrystal_I2C lcd(0x27, 20, 4);

// ============================================================
// OLD CN4 I/O pins - silenced (avoid floating)
// ============================================================
#define OLD_mLeftFWD 30
#define OLD_mLeftREV 31
#define OLD_stopModeL 32
#define OLD_m0L 33
#define OLD_mbFreeL 34
#define OLD_mmLPin 2
#define OLD_mRightFWD 35
#define OLD_mRightREV 36
#define OLD_stopModeR 37
#define OLD_m0R 38
#define OLD_mbFreeR 39
#define OLD_mmRPin 3

// ============================================================
// Modbus pins / slave ids
// ============================================================
#define MODBUS_LEFT_DE_RE   7
#define MODBUS_LEFT_SLAVE   1
#define MODBUS_RIGHT_DE_RE  6
#define MODBUS_RIGHT_SLAVE  2

// ============================================================
// SPEED / TUNING PARAMETERS 
// current speed tunning is less priority than the internal potensiometer in the motor driver
// ============================================================
#define PARAM_LINEAR_SPEED_MAX     0.462861317f // speed limit in m/s (calculated from 3000 rpm, 0.1105m wheel radius, 100:1 reduction)
#define PARAM_ANGULAR_SPEED_MAX    0.74f // speed limit in rad/s 
#define PARAM_GAIN_LINEAR          2.5f
#define PARAM_GAIN_ANGULAR         2.5f

#define PARAM_LEFT_SPEED_TRIM      1.f
#define PARAM_RIGHT_SPEED_TRIM     1.0f

#define PARAM_SBUS_FILTER_ALPHA    0.35f
#define PARAM_RPM_MIN              80
#define PARAM_RPM_MAX              3000
#define PARAM_RPM_CHANGE_THRESHOLD 5
#define PARAM_CMD_RESEND_MS        200      // re-assert FWD/REV keep-alive
#define PARAM_MODBUS_BROADCAST_WRITES true  // one driver per bus: no response wait
#define PARAM_WHEEL_RADIUS         0.1105f
#define PARAM_WHEEL_BASE           0.6f
#define PARAM_REDUCTION_RATIO      100.0f
#define PARAM_MINIPC_TIMEOUT_MS    600      // stop if no MiniPC cmd in this window
#define PARAM_MINIPC_SERIAL_BYTES_PER_LOOP 32

#define PARAM_ODO_CAL_DISTANCE_M   1.000f

// ============================================================
// Actuator / relay pins
// ============================================================
#define liftFWD 42
#define liftREV 43
#define dumpFWD 44
#define dumpREV 45
#define powerRelay 24
#define LightRed 25
#define BatteryPin A6

// ============================================================
// Motor stack
// ============================================================
ModbusRTU mbLeft;
ModbusRTU mbRight;
DiffDrive drive(mbLeft, MODBUS_LEFT_SLAVE, mbRight, MODBUS_RIGHT_SLAVE);

// ============================================================
// SBUS globals
// ============================================================
uint8_t  sbusFrame[SBUS_FRAME_SIZE];
uint16_t sbusChannel[16];
int      sbusIndex = 0;
bool     sbusFrameReady = false;
unsigned long lastSBUS = 0;

float filteredVolt = 0.0f;

float minipc_linear  = 0.0f;
float minipc_angular = 0.0f;
unsigned long minipc_last_cmd = 0;          // for the new MiniPC failsafe

float sbus_leftY = 0.0f, sbus_leftX = 0.0f;
float sbus_rightY = 0.0f, sbus_rightX = 0.0f;
bool  sbus_power = false, sbus_power_last = false;
int   sbus_mode = 0;

int           sbus_mode_debounced = 0;
unsigned long mode_last_change = 0;
const unsigned long MODE_DEBOUNCE_TIME = 200;

enum SystemState { STATE_DISARMED, STATE_ARMED_RC, STATE_ARMED_MINIPC, STATE_ARMED_NEUTRAL };
SystemState   currentState = STATE_DISARMED;
unsigned long stateChangeTime = 0;

byte bigChars[8][8] = {
    {0x1F,0x11,0x11,0x11,0x11,0x11,0x11,0x1F},{0x00,0x04,0x0C,0x04,0x04,0x04,0x04,0x0E},
    {0x1F,0x01,0x01,0x1F,0x10,0x10,0x10,0x1F},{0x1F,0x01,0x01,0x1F,0x01,0x01,0x01,0x1F},
    {0x11,0x11,0x11,0x1F,0x01,0x01,0x01,0x01},{0x1F,0x10,0x10,0x1F,0x01,0x01,0x01,0x1F},
    {0x1F,0x10,0x10,0x1F,0x11,0x11,0x11,0x1F},{0x1F,0x01,0x01,0x01,0x01,0x01,0x01,0x01}
};

// ============================================================
// SBUS
// ============================================================
void readSBUS() {
    while (Serial1.available()) {
        uint8_t b = Serial1.read();
        if (sbusIndex == 0) { if (b == 0x0F) sbusFrame[sbusIndex++] = b; }
        else {
            sbusFrame[sbusIndex++] = b;
            if (sbusIndex == SBUS_FRAME_SIZE) {
                sbusFrameReady = true; lastSBUS = millis(); sbusIndex = 0;
            }
        }
    }
}

void decodeSBUS() {
    sbusChannel[0]  = (sbusFrame[1]       | sbusFrame[2]  << 8) & 0x07FF;
    sbusChannel[1]  = (sbusFrame[2]  >> 3 | sbusFrame[3]  << 5) & 0x07FF;
    sbusChannel[2]  = (sbusFrame[3]  >> 6 | sbusFrame[4]  << 2 | sbusFrame[5]  << 10) & 0x07FF;
    sbusChannel[3]  = (sbusFrame[5]  >> 1 | sbusFrame[6]  << 7) & 0x07FF;
    sbusChannel[4]  = (sbusFrame[6]  >> 4 | sbusFrame[7]  << 4) & 0x07FF;
    sbusChannel[5]  = (sbusFrame[7]  >> 7 | sbusFrame[8]  << 1 | sbusFrame[9]  << 9) & 0x07FF;
    sbusChannel[6]  = (sbusFrame[9]  >> 2 | sbusFrame[10] << 6) & 0x07FF;
    sbusChannel[7]  = (sbusFrame[10] >> 5 | sbusFrame[11] << 3) & 0x07FF;
    sbusChannel[8]  = (sbusFrame[12]      | sbusFrame[13] << 8) & 0x07FF;
    sbusChannel[9]  = (sbusFrame[13] >> 3 | sbusFrame[14] << 5) & 0x07FF;
    sbusChannel[10] = (sbusFrame[14] >> 6 | sbusFrame[15] << 2 | sbusFrame[16] << 10) & 0x07FF;
    sbusChannel[11] = (sbusFrame[16] >> 1 | sbusFrame[17] << 7) & 0x07FF;
    sbusChannel[12] = (sbusFrame[17] >> 4 | sbusFrame[18] << 4) & 0x07FF;
    sbusChannel[13] = (sbusFrame[18] >> 7 | sbusFrame[19] << 1 | sbusFrame[20] << 9) & 0x07FF;
    sbusChannel[14] = (sbusFrame[20] >> 2 | sbusFrame[21] << 6) & 0x07FF;
    sbusChannel[15] = (sbusFrame[21] >> 5 | sbusFrame[22] << 3) & 0x07FF;
}

float normalizeSBUS(uint16_t value) {
    float r = ((float)value - 1024.0f) / 800.0f;
    if (r >  1.0f) r =  1.0f;
    if (r < -1.0f) r = -1.0f;
    return r;
}
float applyDeadzone(float v, float dz) { return (fabs(v) < dz) ? 0.0f : v; }

int getDebouncedMode(uint16_t ch6) {
    int nm = (ch6 < 700) ? -1 : (ch6 > 1400) ? 1 : 0;
    if (nm != sbus_mode_debounced && millis() - mode_last_change > MODE_DEBOUNCE_TIME) {
        sbus_mode_debounced = nm; mode_last_change = millis();
    }
    return sbus_mode_debounced;
}

void updateSBUSControl() {
    if (!sbusFrameReady) return;
    sbusFrameReady = false;
    decodeSBUS();
    lastSBUS = millis();

    const float DEADZONE = 0.10f;
    
    sbus_leftY  =  applyDeadzone(normalizeSBUS(sbusChannel[3]), DEADZONE); // FWD/BACK  -> linear
    sbus_leftX  = -applyDeadzone(normalizeSBUS(sbusChannel[1]), DEADZONE); // LEFT/RIGHT -> angular
    sbus_rightY =  applyDeadzone(normalizeSBUS(sbusChannel[2]), DEADZONE);
    sbus_rightX =  applyDeadzone(normalizeSBUS(sbusChannel[0]), DEADZONE);

    sbus_power_last = sbus_power;
    sbus_power = (sbusChannel[4] > 1200);
    sbus_mode  = getDebouncedMode(sbusChannel[5]);
}

void updateSystemState() {
    bool sbusAlive = (millis() - lastSBUS < 500);
    SystemState newState;
    if (!sbusAlive || !sbus_power) newState = STATE_DISARMED;
    else switch (sbus_mode) {
        case  1: newState = STATE_ARMED_RC;     break;
        case -1: newState = STATE_ARMED_MINIPC; break;
        default: newState = STATE_ARMED_NEUTRAL;break;
    }
    if (newState != currentState) {
        if (millis() - stateChangeTime > 50) { currentState = newState; stateChangeTime = millis(); }
    } else stateChangeTime = millis();
}

void gradualPowerUp() {
    static bool powerWasOn = false;
    if (currentState != STATE_DISARMED && !powerWasOn) {
        digitalWrite(powerRelay, HIGH);
        powerWasOn = true;                  // removed blocking delay(50)
    } else if (currentState == STATE_DISARMED) {
        digitalWrite(powerRelay, LOW);
        powerWasOn = false;
    }
}

// ============================================================
// Actuators
// ============================================================
void liftSBUS(float v) {
    if      (v >  0.3f) { digitalWrite(liftFWD, HIGH); digitalWrite(liftREV, LOW);  }
    else if (v < -0.3f) { digitalWrite(liftFWD, LOW);  digitalWrite(liftREV, HIGH); }
    else                { digitalWrite(liftFWD, LOW);  digitalWrite(liftREV, LOW);  }
}
void dumpSBUS(float v) {
    if      (v >  0.3f) { digitalWrite(dumpFWD, HIGH); digitalWrite(dumpREV, LOW);  }
    else if (v < -0.3f) { digitalWrite(dumpFWD, LOW);  digitalWrite(dumpREV, HIGH); }
    else                { digitalWrite(dumpFWD, LOW);  digitalWrite(dumpREV, LOW);  }
}

void sbusToVelocity(float &linear, float &angular) {
    static float fLX = 0.0f, fLY = 0.0f;
    fLX = PARAM_SBUS_FILTER_ALPHA * sbus_leftX + (1.0f - PARAM_SBUS_FILTER_ALPHA) * fLX;
    fLY = PARAM_SBUS_FILTER_ALPHA * sbus_leftY + (1.0f - PARAM_SBUS_FILTER_ALPHA) * fLY;
    linear  = fLY * PARAM_LINEAR_SPEED_MAX  * PARAM_GAIN_LINEAR;
    angular = fLX * PARAM_ANGULAR_SPEED_MAX * PARAM_GAIN_ANGULAR;
}

// ============================================================
// Battery (non-blocking, no delay)
// ============================================================
float readBatteryVoltageOnce() {
    float adc      = (float)analogRead(BatteryPin);
    float vA6      = adc * (4.1f / 1023.0f);
    float divRatio = (51.0f + 9.1f) / 9.1f;
    return vA6 * divRatio * 1.031f;
}

void updateBattery() {
    static unsigned long last = 0;
    if (millis() - last > 50) {              // Low-pass filtered battery voltage measurement
        float v = readBatteryVoltageOnce();
        filteredVolt = 0.97f * filteredVolt + 0.03f * v;
        last = millis();
    }
}

int calculateSOC(float v) {
    if (v >= 29.4f) return 100;
    if (v <= 18.0f) return 0;
    if (v >= 27.5f) return 80 + (int)((v - 27.5f) * (20.0f / (29.4f - 27.5f)));
    if (v >= 24.5f) return 40 + (int)((v - 24.5f) * (40.0f / (27.5f - 24.5f)));
    return (int)((v - 18.0f) * (40.0f / (24.5f - 18.0f)));
}

void displaySOC(int soc, float voltage) {
    lcd.setCursor(0, 0); lcd.print(" BATTERY STATUS   ");
    lcd.setCursor(0, 1); lcd.print("Volt: "); lcd.print(voltage, 1); lcd.print("V     ");
    lcd.setCursor(0, 2); lcd.print("State: ");
    switch (currentState) {
        case STATE_DISARMED:      lcd.print("DISARMED"); break;
        case STATE_ARMED_RC:      lcd.print("RC MODE "); break;
        case STATE_ARMED_MINIPC:  lcd.print("MINI PC "); break;
        case STATE_ARMED_NEUTRAL: lcd.print("NEUTRAL "); break;
    }
    lcd.print("    ");
    lcd.setCursor(0, 3); lcd.print("SOC "); lcd.print(soc); lcd.print("% [");
    int filled = map(soc, 0, 100, 0, 12);
    for (int i = 0; i < 12; i++) { if (i < filled) lcd.write(255); else lcd.print(" "); }
    lcd.print("]");
}

void updateLCD() {
    static unsigned long last = 0;
    if (millis() - last > 500) { displaySOC(calculateSOC(filteredVolt), filteredVolt); last = millis(); }
}

bool isVelocityLine(const char *line) {
    char c = line[0];
    return (c == '-') || (c == '+') || (c == '.') || (c >= '0' && c <= '9');
}

void processSerialLine(char *line) {
    if (strcmp(line, "ODO_RESET") == 0) {
        odometry.reset();
        Serial.println(F("{\"type\":\"odom_cal\",\"status\":\"reset\"}"));
        return;
    }

    if (strncmp(line, "ODO_CAL", 7) == 0) {
        float distance = PARAM_ODO_CAL_DISTANCE_M;
        char *comma = strchr(line, ',');
        if (comma && atof(comma + 1) > 0.0f) {
            distance = atof(comma + 1);
        }
        odometry.sendCalibration(Serial, distance);
        return;
    }

    if (!isVelocityLine(line)) return;

    char *comma = strchr(line, ',');
    if (comma) {
        *comma = '\0';
        minipc_linear   = atof(line);
        minipc_angular  = atof(comma + 1);
        minipc_last_cmd = millis();     
        // Serial.print("{\"type\":\"debug\",\"linear\":");
        // Serial.print(minipc_linear,4);
        // Serial.print(",\"angular\":");
        // Serial.print(minipc_angular,4);
        // Serial.println("}");
    }
}

void readMiniPC() {
    static char buf[128];
    static uint8_t idx = 0;

    uint8_t bytesRead = 0;
    while (Serial.available() && bytesRead < PARAM_MINIPC_SERIAL_BYTES_PER_LOOP) {
        char c = (char)Serial.read();
        bytesRead++;
        if (c == '\r') continue;

        if (c == '\n') {
            buf[idx] = '\0';
            processSerialLine(buf);
            idx = 0;
        } else if (idx < sizeof(buf) - 1) {
            buf[idx++] = c;
        } else {
            idx = 0;  
        }
    }
}

// ============================================================
// setup()
// ============================================================
void setup() {
    Serial.begin(115200);

    Serial.println("===== MEGA BOOT =====");

    Serial.setTimeout(5);
    Serial1.begin(100000, SERIAL_8E2);       // SBUS

    Serial2.begin(115200, SERIAL_8E1);       // Left bus
    mbLeft.begin(&Serial2, MODBUS_LEFT_DE_RE);
    mbLeft.master();

    Serial3.begin(115200, SERIAL_8E1);       // Right bus
    mbRight.begin(&Serial3, MODBUS_RIGHT_DE_RE);
    mbRight.master();

    // Optional Modbus response timeout configuration.
    // mbLeft.setTimeout(50);
    // mbRight.setTimeout(50);

    drive.begin(PARAM_WHEEL_RADIUS, PARAM_WHEEL_BASE, PARAM_REDUCTION_RATIO,
                PARAM_LINEAR_SPEED_MAX, PARAM_ANGULAR_SPEED_MAX,
                PARAM_RPM_MIN, PARAM_RPM_MAX,
                PARAM_RPM_CHANGE_THRESHOLD, PARAM_CMD_RESEND_MS);
     

/******************************************************************************
 * Idle pre-spin disabled.
 *
 * BLV drivers require RPM commands above the configured minimum threshold
 * before motor motion occurs.
 ******************************************************************************/
    drive.setIdlePreSpin(false);
    drive.setBroadcastWrites(PARAM_MODBUS_BROADCAST_WRITES);
    drive.setSideTrim(PARAM_LEFT_SPEED_TRIM, PARAM_RIGHT_SPEED_TRIM);

    // ---- DIRECTION CALIBRATION (flip these after the bench test below) ----
    drive.setInvertLeft(false);
    drive.setInvertRight(false);
    drive.setSwapSides(false);
    drive.setInvertAngular(false);

    // Silence old CN4 pins
    pinMode(OLD_mLeftFWD,OUTPUT);  digitalWrite(OLD_mLeftFWD,LOW);
    pinMode(OLD_mLeftREV,OUTPUT);  digitalWrite(OLD_mLeftREV,LOW);
    pinMode(OLD_stopModeL,OUTPUT); digitalWrite(OLD_stopModeL,LOW);
    pinMode(OLD_m0L,OUTPUT);       digitalWrite(OLD_m0L,LOW);
    pinMode(OLD_mbFreeL,OUTPUT);   digitalWrite(OLD_mbFreeL,LOW);
    pinMode(OLD_mmLPin,OUTPUT);    analogWrite(OLD_mmLPin,0);
    pinMode(OLD_mRightFWD,OUTPUT); digitalWrite(OLD_mRightFWD,LOW);
    pinMode(OLD_mRightREV,OUTPUT); digitalWrite(OLD_mRightREV,LOW);
    pinMode(OLD_stopModeR,OUTPUT); digitalWrite(OLD_stopModeR,LOW);
    pinMode(OLD_m0R,OUTPUT);       digitalWrite(OLD_m0R,LOW);
    pinMode(OLD_mbFreeR,OUTPUT);   digitalWrite(OLD_mbFreeR,LOW);
    pinMode(OLD_mmRPin,OUTPUT);    analogWrite(OLD_mmRPin,0);

    pinMode(MODBUS_LEFT_DE_RE,OUTPUT);  digitalWrite(MODBUS_LEFT_DE_RE,LOW);
    pinMode(MODBUS_RIGHT_DE_RE,OUTPUT); digitalWrite(MODBUS_RIGHT_DE_RE,LOW);
    pinMode(powerRelay,OUTPUT);         digitalWrite(powerRelay,LOW);
    pinMode(liftFWD,OUTPUT); digitalWrite(liftFWD,LOW);
    pinMode(liftREV,OUTPUT); digitalWrite(liftREV,LOW);
    pinMode(dumpFWD,OUTPUT); digitalWrite(dumpFWD,LOW);
    pinMode(dumpREV,OUTPUT); digitalWrite(dumpREV,LOW);
    pinMode(LightRed,OUTPUT);digitalWrite(LightRed,LOW);
    pinMode(BatteryPin,INPUT);

    lcd.init(); lcd.backlight(); lcd.clear();
    for (int i = 0; i < 8; i++) lcd.createChar(i, bigChars[i]);

    filteredVolt = readBatteryVoltageOnce();
    odometry.begin();

    currentState = STATE_DISARMED;
    stateChangeTime = millis();

    drive.stop();
}

// ============================================================
// loop()
// ============================================================
void loop() {
    drive.task();

    readSBUS();
    updateSBUSControl();

    // Only apply RC link failsafe when actually in RC mode
    if (currentState == STATE_ARMED_RC && millis() - lastSBUS > 100)
        drive.stop();

    updateSystemState();
    gradualPowerUp();

    switch (currentState) {
        case STATE_DISARMED:
            drive.stop();
            break;

        case STATE_ARMED_RC: {
            if (millis() - lastSBUS > 100) { drive.stop(); break; }
            float linear, angular;
            sbusToVelocity(linear, angular);
            drive.drive(linear, angular);
            break;
        }

        case STATE_ARMED_MINIPC:
            if (millis() - minipc_last_cmd < PARAM_MINIPC_TIMEOUT_MS)
                drive.drive(minipc_angular, -minipc_linear);
            else
                drive.stop();
            break;

        case STATE_ARMED_NEUTRAL:
            drive.stop();
            break;
    }

    drive.task();
    readMiniPC();
    updateBattery();
    updateLCD();
    odometry.update();
    odometry.sendJson(Serial);
}
