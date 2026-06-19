#include <Arduino.h>

// Odometry.h
#ifndef ODOMETRY_H
#define ODOMETRY_H

// MSD700 physical constants
#define ODO_WHEEL_RADIUS      0.1105f
#define ODO_WHEEL_BASE        0.6f
#define ODO_REDUCTION_RATIO   100.0f
#define ODO_PPR               1024

// Encoder pins – left (A8/A9), right (A10/A11)
#define ENC_LEFT_A    A8   // PCINT16
#define ENC_LEFT_B    A9
#define ENC_RIGHT_A   A10  // PCINT18
#define ENC_RIGHT_B   A11

static constexpr float ODO_TICKS_PER_METER = 2700.0f;

// OdometryData struct 
struct OdometryData {
    float x, y, theta;
    float v, omega;
};

class Odometry {
public:
    Odometry();
    void begin();
    void update();
    void sendJson(Stream &serial);
    void sendCalibration(Stream &serial, float distanceMeters);
    // void sendData(Stream &serial);   // CSV
    void reset();
    OdometryData getData() const;
    void getTicks(long &left, long &right) const;

    // ISR handler 
    void handleEncoderInterrupt();

private:
    // Encoder tick counters (shared with ISR)
    volatile long _leftTicks;
    volatile long _rightTicks;

    // Last states of channel A 
    volatile uint8_t _lastAStates;   // bit0 = left A, bit1 = right A

    // Pose & velocity
    float _x, _y, _theta, _v, _omega;

    // Timing
    unsigned long _lastUpdateUs;
    long _lastLeftTicks;
    long _lastRightTicks;
    long _lastDeltaLeft;
    long _lastDeltaRight;
    long _lastJsonLeftTicks;
    long _lastJsonRightTicks;
    bool _jsonWasMoving;

    // Meters per tick
    float _metersPerTick;
};

extern Odometry odometry;

#endif
