// MSD700.cpp
// edited by Habibie on 15 march 2026
// update: adding the new function for mini pc mode

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

#define SBUS_FRAME_SIZE 25

LiquidCrystal_I2C lcd(0x27, 20, 4);

// Left motor driver pins
#define mLeftFWD 30
#define mLeftREV 31
#define stopModeL 32
#define m0L 33
#define mbFreeL 34
#define mmLPin 2  // left motor speed PWM

// Right motor driver pins
#define mRightFWD 35
#define mRightREV 36
#define stopModeR 37
#define m0R 38
#define mbFreeR 39
#define mmRPin 3  // Right motor speed PWM

// Cylinder / Actuator driver pins
#define liftFWD 42
#define liftREV 43
#define dumpFWD 44
#define dumpREV 45

// Power relay pin | switch output
#define powerRelay 24 

// Stack light relay pin
#define LightRed 25

// Battery voltage input pin
#define BatteryPin A6

// Watchdog timer to detect resets
#define RESET_CAUSE_REGISTER MCUSR

// global variables for SBUS control
float sbus_leftY = 0.0;
float sbus_leftX = 0.0;
float sbus_rightY = 0.0;
float sbus_rightX = 0.0;

bool sbus_power = false;
bool sbus_power_last = false;
int sbus_mode = 0;  // -1: MINI PC, 0: Neutral, 1: RC

// global varibles for Mini PC mode
float minipc_linear = 0.0;
float minipc_angular = 0.0;
bool minipc_cmd_received = false;

// debounced mode to prevent rapid switching
int sbus_mode_debounced = 0;
unsigned long mode_last_change = 0;
const unsigned long MODE_DEBOUNCE_TIME = 200; // ms

unsigned long lastSBUS = 0;

// for speed tunning
float GAIN1  = 2.5;   // reduce turning, from the testing gain value 1 = 207 pwm
float GAIN2 = 2.5;  // increase forward, from the testing gain value 1 = 100 pwm

// calculation velocity to rpm
#define M_PI 3.14159265358979323846
float LINEAR_SPEED_MAX = 0.462861317;
float ANGULAR_SPEED_MAX = 0.74;

// control variable
float g_linear_x = 0.0;
float g_angular_z = 0.0;

// calculate variable
float g_left_rpm;
float g_right_rpm;
int g_motor_brake = 1;       // none : 0:neutral 1:brake

bool allowOperation = false;

// Global variables for LCD display
unsigned long previousMillis = 0;
bool lightState = false;
float volt = 0;
int value = 0;
float filteredVolt = 0;

// System state machine
enum SystemState {
  STATE_DISARMED,
  STATE_ARMED_RC,
  STATE_ARMED_MINIPC,
  STATE_ARMED_NEUTRAL
};
SystemState currentState = STATE_DISARMED;

// FIXED: Moved stateChangeTime to global scope to eliminate warning
unsigned long stateChangeTime = 0;

byte bigChars[8][8] = {
  {0x1F,0x11,0x11,0x11,0x11,0x11,0x11,0x1F}, // 0 full block border
  {0x00,0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}, // 1
  {0x1F,0x01,0x01,0x1F,0x10,0x10,0x10,0x1F}, // 2
  {0x1F,0x01,0x01,0x1F,0x01,0x01,0x01,0x1F}, // 3
  {0x11,0x11,0x11,0x1F,0x01,0x01,0x01,0x01}, // 4
  {0x1F,0x10,0x10,0x1F,0x01,0x01,0x01,0x1F}, // 5
  {0x1F,0x10,0x10,0x1F,0x11,0x11,0x11,0x1F}, // 6
  {0x1F,0x01,0x01,0x01,0x01,0x01,0x01,0x01}  // 7
};

// SBUS frame buffer and channel values
uint8_t sbusFrame[SBUS_FRAME_SIZE];
uint16_t sbusChannel[16];

int sbusIndex = 0;
bool sbusFrameReady = false;

/*======================================================================
 * Function declarations
 *=======================================================================*/

void readSBUS()
{
    while (Serial1.available())
    {
        uint8_t b = Serial1.read();

        // Sync start byte
        if (sbusIndex == 0)
        {
            if (b == 0x0F)
            {
                sbusFrame[sbusIndex++] = b;
            }
        }
        else
        {
            sbusFrame[sbusIndex++] = b;

            if (sbusIndex == SBUS_FRAME_SIZE)
            {
                sbusFrameReady = true;
                lastSBUS = millis();
                sbusIndex = 0;
            }
        }
    }
}

void decodeSBUS() {
  sbusChannel[0]  = (sbusFrame[1]       | sbusFrame[2]  << 8) & 0x07FF;
  sbusChannel[1]  = (sbusFrame[2] >> 3  | sbusFrame[3]  << 5) & 0x07FF;
  sbusChannel[2]  = (sbusFrame[3] >> 6  | sbusFrame[4]  << 2 | sbusFrame[5] << 10) & 0x07FF;
  sbusChannel[3]  = (sbusFrame[5] >> 1  | sbusFrame[6]  << 7) & 0x07FF;
  sbusChannel[4]  = (sbusFrame[6] >> 4  | sbusFrame[7]  << 4) & 0x07FF;
  sbusChannel[5]  = (sbusFrame[7] >> 7  | sbusFrame[8]  << 1 | sbusFrame[9] << 9) & 0x07FF;
  sbusChannel[6]  = (sbusFrame[9] >> 2  | sbusFrame[10] << 6) & 0x07FF;
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

float normalizeSBUS(uint16_t value)
{
    const float center = 1024.0;
    const float halfRange = 800.0;

    float result = ((float)value - center) / halfRange;

    if (result > 1.0) result = 1.0;
    if (result < -1.0) result = -1.0;

    return result;
}

float applyDeadzone(float value, float deadzone)
{
    if (abs(value) < deadzone)
        return 0.0;
    else
        return value;
}

// Debounced mode reading
int getDebouncedMode(uint16_t ch6) {
    int newMode;
    
    if (ch6 < 700)
        newMode = -1;     // MINI PC
    else if (ch6 > 1400)
        newMode = 1;      // RC
    else
        newMode = 0;      // Neutral
        
    // Debounce logic
    if (newMode != sbus_mode_debounced) {
        if (millis() - mode_last_change > MODE_DEBOUNCE_TIME) {
            sbus_mode_debounced = newMode;
            mode_last_change = millis();
        }
    }
    
    return sbus_mode_debounced;
}

void updateSBUSControl()
{
    if (!sbusFrameReady) return;

    sbusFrameReady = false;
    decodeSBUS();

    lastSBUS = millis();

    float raw_leftY  = normalizeSBUS(sbusChannel[1]);
    float raw_leftX  = normalizeSBUS(sbusChannel[3]);
    float raw_rightY = normalizeSBUS(sbusChannel[2]);
    float raw_rightX = normalizeSBUS(sbusChannel[0]);

    const float DEADZONE = 0.10;

    sbus_leftY  = applyDeadzone(raw_leftY, DEADZONE);
    sbus_leftX  = applyDeadzone(raw_leftX, DEADZONE);
    sbus_rightY = applyDeadzone(raw_rightY, DEADZONE);
    sbus_rightX = applyDeadzone(raw_rightX, DEADZONE);

    // POWER SWITCH (CH5) - Store previous state
    sbus_power_last = sbus_power;
    sbus_power = (sbusChannel[4] > 1200);

    // MODE SWITCH (CH6) - Get debounced mode
    sbus_mode = getDebouncedMode(sbusChannel[5]);
}

void updateSystemState() {
    SystemState newState;
    bool sbusAlive = (millis() - lastSBUS < 200);
    
    // Failsafe: if SBUS lost, force disarmed
    if (!sbusAlive) {
        newState = STATE_DISARMED;
    }
    // Power switch takes precedence
    else if (!sbus_power) {
        newState = STATE_DISARMED;
    }
    // Armed based on mode
    else {
        switch(sbus_mode) {
            case 1:
                newState = STATE_ARMED_RC;
                break;
            case -1:
                newState = STATE_ARMED_MINIPC;
                break;
            case 0:
            default:
                newState = STATE_ARMED_NEUTRAL;
                break;
        }
    }
    
    // Apply state change with hysteresis to prevent rapid toggling
    if (newState != currentState) {
        // Only change if stable for a short time
        if (millis() - stateChangeTime > 50) {
            currentState = newState;
            stateChangeTime = millis();  
        }
    } else {
        // Reset timer if state is stable
        stateChangeTime = millis();  
    }
}

// Gradual power-up to prevent brown-out
void gradualPowerUp() {
    static bool powerWasOn = false;
    
    if (currentState != STATE_DISARMED && !powerWasOn) {
        // Just armed - gradually enable power
        digitalWrite(powerRelay, HIGH);
        delay(50);  // Let power stabilize
        
        // Re-initialize peripherals if needed
        lcd.init();
        lcd.backlight();
        
        powerWasOn = true;
    } else if (currentState == STATE_DISARMED) {
        digitalWrite(powerRelay, LOW);
        powerWasOn = false;
    }
}

float velToRpm(float v) {
  const float REDUCTION_RATIO = 100.0;
  const float WHEEL_RADIUS = 0.1105;
  float n = 30 / M_PI * REDUCTION_RATIO / WHEEL_RADIUS * v;
  return n;
}

void ik(float v, float w)
{
    const float WHEEL_DISTANCE = 0.6;
    float v_l = v - (WHEEL_DISTANCE / 2.0) * w;
    float v_r = v + (WHEEL_DISTANCE / 2.0) * w;

    g_left_rpm  = velToRpm(v_l);
    g_right_rpm = velToRpm(v_r);
}

void writeMotorPwm(float left_rpm, float right_rpm)
{
    // ----- LEFT MOTOR (INVERTED LOGIC) -----
    if (left_rpm > 0)
    {
        digitalWrite(mLeftFWD, HIGH);
        digitalWrite(mLeftREV, LOW);
    }
    else if (left_rpm < 0)
    {
        digitalWrite(mLeftFWD, LOW);
        digitalWrite(mLeftREV, HIGH);
    }
    else
    {
        digitalWrite(mLeftFWD, LOW);
        digitalWrite(mLeftREV, LOW);
    }

    int left_pwm = constrain(map(abs(left_rpm), 0, 2000, 0, 255), 0, 255); // Meaning: 0 RPM   → PWM 0, 4000 RPM → PWM 255
    analogWrite(mmLPin, left_pwm);
    

    // ----- RIGHT MOTOR (INVERTED LOGIC) -----
    if (right_rpm > 0)
    {
        digitalWrite(mRightFWD, LOW);
        digitalWrite(mRightREV, HIGH);
    }
    else if (right_rpm < 0)
    {
        digitalWrite(mRightFWD, HIGH);
        digitalWrite(mRightREV, LOW);
    }
    else
    {
        digitalWrite(mRightFWD, LOW);
        digitalWrite(mRightREV, LOW);
    }

    int right_pwm = constrain(map(abs(right_rpm), 0, 2000, 0, 255), 0, 255);
    analogWrite(mmRPin, right_pwm);
}

void driveRobot(float linear, float angular)
{
    linear  = constrain(linear, -LINEAR_SPEED_MAX, LINEAR_SPEED_MAX);
    angular = constrain(angular, -ANGULAR_SPEED_MAX, ANGULAR_SPEED_MAX);

    // Convert v,w to wheel RPM
    ik(linear, angular);

    writeMotorPwm(g_left_rpm, g_right_rpm);
}

void stopMotor(bool motor_brake_state) {
  if (motor_brake_state == 0) {
    digitalWrite(mbFreeL, HIGH);
    digitalWrite(mbFreeR, HIGH);
  } else if (motor_brake_state == 1) {
    digitalWrite(mbFreeL, LOW);
    digitalWrite(mbFreeR, LOW);
  }
  analogWrite(mmLPin, 0);
  analogWrite(mmRPin, 0);
}

void liftSBUS(float value)
{
    if (value > 0.3)
    {
        digitalWrite(liftFWD, HIGH);
        digitalWrite(liftREV, LOW);
    }
    else if (value < -0.3)
    {
        digitalWrite(liftFWD, LOW);
        digitalWrite(liftREV, HIGH);
    }
    else
    {
        digitalWrite(liftFWD, LOW);
        digitalWrite(liftREV, LOW);
    }
}

void dumpSBUS(float value)
{
    if (value > 0.3)
    {
        digitalWrite(dumpFWD, HIGH);
        digitalWrite(dumpREV, LOW);
    }
    else if (value < -0.3)
    {
        digitalWrite(dumpFWD, LOW);
        digitalWrite(dumpREV, HIGH);
    }
    else
    {
        digitalWrite(dumpFWD, LOW);
        digitalWrite(dumpREV, LOW);
    }
}

void sbusToVelocity(float &linear, float &angular)
{
    const float alpha = 0.35;

    static float filteredLX = 0;
    static float filteredLY = 0;

    filteredLX = alpha * sbus_leftX + (1 - alpha) * filteredLX;
    filteredLY = alpha * sbus_leftY + (1 - alpha) * filteredLY;

    linear  = -filteredLY * LINEAR_SPEED_MAX * GAIN2;
    angular = -filteredLX    * ANGULAR_SPEED_MAX * GAIN1;
}

void failsafeCheck()
{
    if (millis() - lastSBUS > 100)
    {
        sbus_leftX = 0;
        sbus_leftY = 0;
        sbus_rightX = 0;
        sbus_rightY = 0;

        stopMotor(1);
    }
}

// function to read battery voltage section ----------------------------------------

float readBatteryVoltage() {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(BatteryPin);
    delay(1);
  }
  float adcValue = sum / 10.0;
  float Vref = 4.1;
  float vA6 = adcValue * (Vref / 1023.0);
  float dividerRatio = (51.0 + 9.1) / 9.1;
  float batteryVoltage = vA6 * dividerRatio * 1.031;
  return batteryVoltage;
}

void checkBatteryStatus() {
  float batteryVoltage = readBatteryVoltage();
  
  if (batteryVoltage < 20.00 && batteryVoltage >= 18.0) {
    Serial.println("Voltage below 20V. Only driving motors are active.");
    digitalWrite(LightRed, HIGH);
    digitalWrite(liftFWD, LOW);
    digitalWrite(liftREV, LOW);
    digitalWrite(dumpFWD, LOW);
    digitalWrite(dumpREV, LOW);
  } else if (batteryVoltage < 18.0) {
    Serial.println("Voltage below 18V. Shutting down all operations.");
    digitalWrite(LightRed, HIGH);
    stopMotor(g_motor_brake);
    digitalWrite(liftFWD, LOW);
    digitalWrite(liftREV, LOW);
    digitalWrite(dumpFWD, LOW);
    digitalWrite(dumpREV, LOW);
  } else {
    Serial.println("Voltage normal. All operations are allowed.");
    digitalWrite(LightRed, LOW);
  }
}

void updateBattery()
{
    static unsigned long lastBatteryRead = 0;
    if (millis() - lastBatteryRead > 500) {
        float volt = readBatteryVoltage();
        float alpha = 0.85;
        filteredVolt = alpha * filteredVolt + (1.0 - alpha) * volt;
        lastBatteryRead = millis();
    }
}

int calculateSOC(float voltage) {
  if (voltage >= 29.4) return 100;
  if (voltage <= 18.0) return 0;
  if (voltage >= 27.5) {
    return 80 + (voltage - 27.5) * (20.0 / (29.4 - 27.5));
  }
  else if (voltage >= 24.5) {
    return 40 + (voltage - 24.5) * (40.0 / (27.5 - 24.5));
  }
  else {
    return (voltage - 18.0) * (40.0 / (24.5 - 18.0));
  }
}

void printBattery()
{
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
        // Serial.print("SOC: ");
        // Serial.println(calculateSOC(filteredVolt));
        lastPrint = millis();
    }
}

void displaySOC(int soc, float voltage) {
  lcd.setCursor(0,0);
  lcd.print("BATTERY STATUS   ");

  lcd.setCursor(0,1);
  lcd.print("Volt: ");
  lcd.print(voltage,1);
  lcd.print("V     ");

  lcd.setCursor(0,2);
  
  // Display current state
  lcd.print("State: ");
  switch(currentState) {
    case STATE_DISARMED:
      lcd.print("DISARMED");
      break;
    case STATE_ARMED_RC:
      lcd.print("RC MODE ");
      break;
    case STATE_ARMED_MINIPC:
      lcd.print("MINI PC ");
      break;
    case STATE_ARMED_NEUTRAL:
      lcd.print("NEUTRAL ");
      break;
  }
  lcd.print("    ");

  lcd.setCursor(0,3);
  lcd.print("SOC ");
  lcd.print(soc);
  lcd.print("% [");
  
  int totalBars = 12;   
  int filledBars = map(soc, 0, 100, 0, totalBars);

  for(int i=0; i<totalBars; i++){
    if(i < filledBars)
      lcd.write(255);
    else
      lcd.print(" ");
  }
  lcd.print("]");
}

void updateLCD()
{
    static unsigned long lastLCD = 0;
    if (millis() - lastLCD > 500) {
        int soc = calculateSOC(filteredVolt);
        displaySOC(soc, filteredVolt);
        lastLCD = millis();
    }
}

// // Check for reset cause
// void checkResetCause() {
//   uint8_t resetCause = RESET_CAUSE_REGISTER;
//   if (resetCause & (1 << PORF)) {
//     Serial.println("Power-on Reset detected");
//   }
//   if (resetCause & (1 << EXTRF)) {
//     Serial.println("External Reset detected - possible brown-out!");
//   }
//   if (resetCause & (1 << BORF)) {
//     Serial.println("Brown-out Reset detected - voltage dropped!");
//   }
//   RESET_CAUSE_REGISTER = 0;  // Clear flags
// }

void readMiniPC()
{
    if (Serial.available())
    {
        String data = Serial.readStringUntil('\n');

        int comma = data.indexOf(',');

        if (comma > 0)
        {
            minipc_linear = data.substring(0, comma).toFloat();
            minipc_angular = data.substring(comma + 1).toFloat();

            minipc_cmd_received = true;
        }
    }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(100000, SERIAL_8E2);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  for(int i=0;i<8;i++){
    lcd.createChar(i, bigChars[i]);
  }

  // Motor driver speed setting pins
  pinMode(mmLPin, OUTPUT);
  pinMode(mmRPin, OUTPUT);

  // Left motor driver output
  pinMode(mLeftFWD, OUTPUT);
  pinMode(mLeftREV, OUTPUT);
  pinMode(stopModeL, OUTPUT);
  pinMode(m0L, OUTPUT);
  pinMode(mbFreeL, OUTPUT);

  // Right motor driver output
  pinMode(mRightFWD, OUTPUT);
  pinMode(mRightREV, OUTPUT);
  pinMode(stopModeR, OUTPUT);
  pinMode(m0R, OUTPUT);
  pinMode(mbFreeR, OUTPUT);

  // Power relay control - start disarmed
  pinMode(powerRelay, OUTPUT);
  digitalWrite(powerRelay, LOW);

  digitalWrite(stopModeR, HIGH);
  digitalWrite(stopModeL, HIGH);
  digitalWrite(m0R, HIGH);
  digitalWrite(m0L, HIGH);
  digitalWrite(mbFreeR, HIGH);
  digitalWrite(mbFreeL, HIGH);

  pinMode(liftFWD, OUTPUT);
  pinMode(liftREV, OUTPUT);
  pinMode(dumpFWD, OUTPUT);
  pinMode(dumpREV, OUTPUT);
  pinMode(LightRed, OUTPUT);
  pinMode(BatteryPin, INPUT);
  
  filteredVolt = readBatteryVoltage();
  
  // Initialize state
  currentState = STATE_DISARMED;
  stateChangeTime = millis();  // FIXED: Initialize the state change timer
  
//   Serial.println("SBUS Control ready - REV 2.1");
}

void loop() {
  // Always read SBUS
  readSBUS();
  updateSBUSControl();

  readMiniPC();
  
  // Update battery monitoring (low priority)
  updateBattery();
  updateLCD();
  printBattery();
  
  // Update system state based on latest inputs
  updateSystemState();
  
  // Apply gradual power-up to prevent brown-out
  gradualPowerUp();
  
  // Main state machine execution
  switch(currentState) {
    case STATE_DISARMED:
      // Motors stopped, power relay may be off
      stopMotor(1);
      // All actuators disabled
      digitalWrite(liftFWD, LOW);
      digitalWrite(liftREV, LOW);
      digitalWrite(dumpFWD, LOW);
      digitalWrite(dumpREV, LOW);
      break;
      
    case STATE_ARMED_RC:
      // RC Mode - full control
      {
        if (millis() - lastSBUS > 100) {
            stopMotor(1);
            break;
        }

        float linear;
        float angular;

        sbusToVelocity(linear, angular);

        driveRobot(linear, angular);
      }
      break;
      
    case STATE_ARMED_MINIPC:
      
      {
        if (minipc_cmd_received) {
            driveRobot(minipc_linear, minipc_angular);
            }
        else {
            stopMotor(1);
        }
    }
      break;
      
    case STATE_ARMED_NEUTRAL:
      // Neutral - motors stopped, but system armed
      stopMotor(1);
      break;    
  }
  
  // Failsafe check always running
  failsafeCheck();
  
  // Small delay to prevent overwhelming the system
  delay(1);
}