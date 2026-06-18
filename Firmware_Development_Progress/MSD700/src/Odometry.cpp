// Odometry.cpp
#include "Odometry.h"
#include <math.h>

Odometry odometry;

// ----------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------
Odometry::Odometry()
    : _leftTicks(0), _rightTicks(0),
      _lastAStates(0),
      _x(0), _y(0), _theta(0), _v(0), _omega(0),
      _lastUpdateUs(0), _lastLeftTicks(0), _lastRightTicks(0),
      _lastDeltaLeft(0), _lastDeltaRight(0),
      _lastJsonLeftTicks(0), _lastJsonRightTicks(0), _jsonWasMoving(false),
      _metersPerTick(0)
{}

// ----------------------------------------------------------------------
// begin() – configure pins and enable interrupt
// ----------------------------------------------------------------------
void Odometry::begin() {
    // Compute meters per encoder tick (x2 quadrature)
    float ticksPerWheelRev = (float)ODO_PPR * ODO_REDUCTION_RATIO * 2.0f;
    _metersPerTick = (2.0f * M_PI * ODO_WHEEL_RADIUS) / ticksPerWheelRev;

    // Set encoder pins as inputs
    pinMode(ENC_LEFT_A, INPUT);
    pinMode(ENC_LEFT_B, INPUT);
    pinMode(ENC_RIGHT_A, INPUT);
    pinMode(ENC_RIGHT_B, INPUT);

    // Capture initial states of channel A for both encoders
    _lastAStates = 0;
    if (digitalRead(ENC_LEFT_A))  _lastAStates |= 0b01;
    if (digitalRead(ENC_RIGHT_A)) _lastAStates |= 0b10;

    // Enable Pin Change Interrupt bank 2
    PCICR  |= (1 << PCIE2);        // enable PCINT[23:16]
    // Unmask left A (PCINT16) and right A (PCINT18)
    PCMSK2 |= (1 << PCINT16) | (1 << PCINT18);

    _lastUpdateUs   = micros();
    _lastLeftTicks  = 0;
    _lastRightTicks = 0;
    _lastDeltaLeft  = 0;
    _lastDeltaRight = 0;
    _lastJsonLeftTicks  = 0;
    _lastJsonRightTicks = 0;
    _jsonWasMoving = false;
}

// ----------------------------------------------------------------------
// handleEncoderInterrupt() – called from ISR, updates tick counters
// ----------------------------------------------------------------------
void Odometry::handleEncoderInterrupt() {
    // Read current states of both A and B signals
    uint8_t leftA  = digitalRead(ENC_LEFT_A);
    uint8_t leftB  = digitalRead(ENC_LEFT_B);
    uint8_t rightA = digitalRead(ENC_RIGHT_A);
    uint8_t rightB = digitalRead(ENC_RIGHT_B);

    uint8_t currentAStates = (leftA ? 0b01 : 0) | (rightA ? 0b10 : 0);
    uint8_t changed = _lastAStates ^ currentAStates;

    // Left encoder changed?
    if (changed & 0b01) {
        // Quadrature decode:
        // rising edge (A went HIGH)  -> forward if B LOW
        // falling edge (A went LOW)  -> forward if B HIGH
        if (leftA) {
            _leftTicks += (leftB == LOW) ? 1 : -1;
        } else {
            _leftTicks += (leftB == HIGH) ? 1 : -1;
        }
    }

    // Right encoder changed?
    if (changed & 0b10) {
        if (rightA) {
            _rightTicks += (rightB == LOW) ? 1 : -1;
        } else {
            _rightTicks += (rightB == HIGH) ? 1 : -1;
        }
    }

    // Update stored states
    _lastAStates = currentAStates;
}

// ----------------------------------------------------------------------
// update() – compute velocities and integrate pose
// ----------------------------------------------------------------------
void Odometry::update() {
    unsigned long nowUs = micros();
    unsigned long dtUs  = nowUs - _lastUpdateUs;
    if (dtUs < 5000UL) return;   // at least 5 ms between updates

    // Snapshot ticks atomically
    noInterrupts();
    long leftTicks  = _leftTicks;
    long rightTicks = _rightTicks;
    interrupts();

    long deltaLeft  = leftTicks  - _lastLeftTicks;
    long deltaRight = rightTicks - _lastRightTicks;
    _lastDeltaLeft  = deltaLeft;
    _lastDeltaRight = deltaRight;
    float dt = (float)dtUs * 1e-6f;

    // Distance traveled by each wheel
    float distLeft  = (float)deltaLeft  * _metersPerTick;
    float distRight = (float)deltaRight * _metersPerTick;

    // Differential drive kinematics
    float dist   = (distLeft + distRight) / 2.0f;
    float dTheta = (distRight - distLeft) / ODO_WHEEL_BASE;

    // Mid‑point integration
    float heading = _theta + dTheta / 2.0f;
    _x     += dist * cos(heading);
    _y     += dist * sin(heading);
    _theta += dTheta;

    // Normalize theta to [-PI, PI]
    while (_theta >  M_PI) _theta -= 2.0f * M_PI;
    while (_theta < -M_PI) _theta += 2.0f * M_PI;

    // Velocities
    _v     = dist / dt;
    _omega = dTheta / dt;

    // Save for next iteration
    _lastLeftTicks  = leftTicks;
    _lastRightTicks = rightTicks;
    _lastUpdateUs   = nowUs;
}

// ----------------------------------------------------------------------
// sendJson() – JSON output (unchanged, but now uses real data)
// ----------------------------------------------------------------------
void Odometry::sendJson(Stream &serial) {
    const unsigned long SEND_INTERVAL_MS = 50;
    static unsigned long lastSend = 0;
    if (millis() - lastSend < SEND_INTERVAL_MS) return;

    long leftTicks;
    long rightTicks;
    getTicks(leftTicks, rightTicks);

    bool tickChanged = (leftTicks != _lastJsonLeftTicks) ||
                       (rightTicks != _lastJsonRightTicks);
    bool movingNow = (_lastDeltaLeft != 0) || (_lastDeltaRight != 0);

    // Send only when encoder pulses arrive, plus one final zero-speed frame
    // after motion stops so the Mini PC sees a clean stopped state.
    if (!tickChanged && !(_jsonWasMoving && !movingNow)) return;

    lastSend = millis();
    _lastJsonLeftTicks = leftTicks;
    _lastJsonRightTicks = rightTicks;
    _jsonWasMoving = movingNow;

    serial.print(F("{\"type\":\"odom\",\"x\":"));
    serial.print(_x, 4);
    serial.print(F(",\"y\":"));
    serial.print(_y, 4);
    serial.print(F(",\"theta\":"));
    serial.print(_theta, 4);
    serial.print(F(",\"v\":"));
    serial.print(_v, 4);
    serial.print(F(",\"omega\":"));
    serial.print(_omega, 4);
    serial.print(F(",\"left_ticks\":"));
    serial.print(leftTicks);
    serial.print(F(",\"right_ticks\":"));
    serial.print(rightTicks);
    serial.print(F(",\"avg_ticks_abs\":"));
    serial.print(((float)labs(leftTicks) + (float)labs(rightTicks)) * 0.5f, 1);
    serial.println(F("}"));
}

void Odometry::sendCalibration(Stream &serial, float distanceMeters) {
    long leftTicks;
    long rightTicks;
    getTicks(leftTicks, rightTicks);

    long leftAbs = labs(leftTicks);
    long rightAbs = labs(rightTicks);
    float avgTicks = ((float)leftAbs + (float)rightAbs) * 0.5f;
    float wheelRevs = distanceMeters / (2.0f * M_PI * ODO_WHEEL_RADIUS);
    float ticksPerWheelRev = (wheelRevs > 0.0f) ? (avgTicks / wheelRevs) : 0.0f;
    float suggestedPpr = ticksPerWheelRev / (ODO_REDUCTION_RATIO * 2.0f);

    serial.print(F("{\"type\":\"odom_cal\",\"distance_m\":"));
    serial.print(distanceMeters, 3);
    serial.print(F(",\"left_ticks\":"));
    serial.print(leftTicks);
    serial.print(F(",\"right_ticks\":"));
    serial.print(rightTicks);
    serial.print(F(",\"avg_ticks\":"));
    serial.print(avgTicks, 1);
    serial.print(F(",\"ticks_per_wheel_rev\":"));
    serial.print(ticksPerWheelRev, 1);
    serial.print(F(",\"suggested_odo_ppr\":"));
    serial.print(suggestedPpr, 2);
    serial.println(F("}"));
}

// // CSV version (optional)
// void Odometry::sendData(Stream &serial) {
//     const unsigned long SEND_INTERVAL_MS = 50;
//     static unsigned long lastSend = 0;
//     if (millis() - lastSend < SEND_INTERVAL_MS) return;
//     lastSend = millis();

//     serial.print("ODO,");
//     serial.print(_x, 4);      serial.print(",");
//     serial.print(_y, 4);      serial.print(",");
//     serial.print(_theta, 4);  serial.print(",");
//     serial.print(_v, 4);      serial.print(",");
//     serial.println(_omega, 4);
// }

// ----------------------------------------------------------------------
// reset() – zero all state
// ----------------------------------------------------------------------
void Odometry::reset() {
    noInterrupts();
    _leftTicks  = 0;
    _rightTicks = 0;
    interrupts();

    _lastLeftTicks  = 0;
    _lastRightTicks = 0;
    _lastDeltaLeft  = 0;
    _lastDeltaRight = 0;
    _lastJsonLeftTicks  = 0;
    _lastJsonRightTicks = 0;
    _jsonWasMoving = false;
    _x = _y = _theta = 0.0f;
    _v = _omega = 0.0f;
    _lastUpdateUs = micros();
}

// ----------------------------------------------------------------------
// getData() – return current values
// ----------------------------------------------------------------------
OdometryData Odometry::getData() const {
    OdometryData d;
    d.x = _x; d.y = _y; d.theta = _theta;
    d.v = _v; d.omega = _omega;
    return d;
}

void Odometry::getTicks(long &left, long &right) const {
    noInterrupts();
    left = _leftTicks;
    right = _rightTicks;
    interrupts();
}

// ----------------------------------------------------------------------
// Global ISR – connect PCINT2 vector to the Odometry handler
// ----------------------------------------------------------------------
ISR(PCINT2_vect) {
    odometry.handleEncoderInterrupt();
}
