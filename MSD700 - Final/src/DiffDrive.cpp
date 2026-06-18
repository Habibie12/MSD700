/******************************************************************************
 * File        : DiffDrive.cpp
 * Project     : MSD700 Locomotion Controller
 *
 * Description :
 *   Differential-drive motor control implementation for the MSD700
 *
 *   This module provides:
 *     - BLV motor driver communication via Modbus RTU
 *     - Differential-drive inverse kinematics
 *     - Velocity-to-RPM conversion
 *     - Independent left/right motor control
 *     - Command scheduling and transmission
 *
 * Notes:
 *     - Each motor operates on an independent RS485 bus.
 *     - Motor commands are transmitted using a non-blocking state machine.
 *     - Wheel velocity commands are generated from differential-drive
 *       kinematics using linear and angular velocity inputs.
 *
 ******************************************************************************/

#include "DiffDrive.h"
#include <math.h>

static inline int32_t i32abs(int32_t v) { return v < 0 ? -v : v; }

BLVMotor::BLVMotor(ModbusRTU &bus, uint8_t slaveId, bool invert)
    : _bus(bus), _slave(slaveId), _invert(invert),
      _broadcastWrites(false),
      _rpmMin(80), _rpmMax(3000), _threshold(5), _cmdResendMs(200),
      _state(IDLE),
      _pendingRpm(0), _lastRpm(0),
      _pendingCmd(BLV_BIT_STOP), _lastCmd(0xFFFF),
      _lastCmdWriteMs(0),
      _wasStopped(true)
{
    _speedBuf[0] = _speedBuf[1] = 0;
    _cmdBuf[0]   = _cmdBuf[1]   = 0;
}

void BLVMotor::begin(int32_t rpmMin, int32_t rpmMax,
                     int32_t changeThreshold, uint32_t cmdResendMs,
                     bool broadcastWrites) {
    _rpmMin      = rpmMin;
    _rpmMax      = rpmMax;
    _threshold   = changeThreshold;
    _cmdResendMs = cmdResendMs;
    _broadcastWrites = broadcastWrites;
}

int32_t BLVMotor::clampMag(int32_t rpm) const {
    int32_t m = i32abs(rpm);
    if (m > 0 && m < _rpmMin) m = 0;        
    if (m > _rpmMax)          m = _rpmMax;
    return m;
}

// Determine command
void BLVMotor::setRpm(int32_t rpm) {
    if (_invert) rpm = -rpm;                
    int32_t mag = clampMag(rpm);
    
    // Determine command
    uint16_t newCmd;
    if      (mag == 0)  newCmd = BLV_BIT_STOP;
    else if (rpm > 0)   newCmd = BLV_BIT_FWD;
    else                newCmd = BLV_BIT_REV;
    
    // Track stop state for combined write optimization
    if (newCmd == BLV_BIT_STOP) {
        _wasStopped = true;
    } else if (mag > 0) {
        _wasStopped = false;
    }
    
    _pendingRpm = mag;
    _pendingCmd = newCmd;
}

// Each motor operates on an independent RS485 bus and manages its own
// transaction scheduling
void BLVMotor::task() {
    _bus.task();                 
    if (_bus.slave()) return;    // a transaction is in flight on this bus

    const uint32_t now = millis();
    
    /******************************************************************************
     * Command keep-alive.
     *
     * Periodically re-transmits the current command, including STOP, to
     * maintain bus activity and ensure consistent motor response after
     * Speed and command registers are transmitted as separate Modbus
     ******************************************************************************/
    if (_state == IDLE) {
        bool speedChanged = (i32abs(_pendingRpm - _lastRpm) > _threshold);
        bool cmdChanged   = (_pendingCmd != _lastCmd);
    
        bool cmdStale     = (now - _lastCmdWriteMs > _cmdResendMs);
        
        
        if (speedChanged)                _state = WRITE_SPEED;
        else if (cmdChanged || cmdStale) _state = WRITE_CMD;
        else return;                 // nothing to send
    }

    if (_state == WRITE_SPEED) {
        _speedBuf[0] = (uint16_t)((_pendingRpm >> 16) & 0xFFFF);
        _speedBuf[1] = (uint16_t)( _pendingRpm        & 0xFFFF);
        uint8_t target = _broadcastWrites ? 0 : _slave;
        if (_bus.writeHreg(target, BLV_REG_SPEED, _speedBuf, 2)) {
            _lastRpm = _pendingRpm;
            _state = (_pendingCmd != _lastCmd) ? WRITE_CMD : IDLE;
        }
        return;
    }

    if (_state == WRITE_CMD) {
        _cmdBuf[0] = 0x0000;
        _cmdBuf[1] = _pendingCmd;
        uint8_t target = _broadcastWrites ? 0 : _slave;
        if (_bus.writeHreg(target, BLV_REG_CMD, _cmdBuf, 2)) {
            _lastCmd        = _pendingCmd;
            _lastCmdWriteMs = now;
            _state          = IDLE;
        }
        return;
    }
}

// =====================================================================
// DiffDrive
// =====================================================================
DiffDrive::DiffDrive(ModbusRTU &leftBus,  uint8_t leftSlave,
                     ModbusRTU &rightBus, uint8_t rightSlave)
    : _left(leftBus,  leftSlave,  false),
      _right(rightBus, rightSlave, false),
      _wheelRadius(0.1f), _wheelBase(0.5f), _reduction(100.0f),
      _leftTrim(1.0f), _rightTrim(1.0f),
      _linMax(0.5f), _angMax(0.7f),
      _swap(false), _invAng(false),
      _leftRpm(0), _rightRpm(0),
      _idlePwmEnabled(false), _idlePwmRpm(0)
{}

void DiffDrive::begin(float wheelRadius, float wheelBase, float reductionRatio,
                      float linMax, float angMax,
                      int32_t rpmMin, int32_t rpmMax,
                      int32_t changeThreshold, uint32_t cmdResendMs) {
    _wheelRadius = wheelRadius;
    _wheelBase   = wheelBase;
    _reduction   = reductionRatio;
    _linMax      = linMax;
    _angMax      = angMax;
    _left.begin(rpmMin, rpmMax, changeThreshold, cmdResendMs);
    _right.begin(rpmMin, rpmMax, changeThreshold, cmdResendMs);
    
    // Set idle pre-spin RPM just below movement threshold
    _idlePwmRpm = rpmMin - 5;  // Just below minimum, won't actually move
}

float DiffDrive::velToRpm(float v) const {
    // wheel_rpm = v / (2*pi*r) * 60 * gear = v * 30 * gear / (pi * r)
    return (30.0f / (float)M_PI) * _reduction / _wheelRadius * v;
}

void DiffDrive::setIdlePreSpin(bool enable) {
    _idlePwmEnabled = enable;
    if (!enable) {
        stop();  // Clear any idle commands
    }
}

void DiffDrive::drive(float linear, float angular) {
    linear  = constrain(linear,  -_linMax, _linMax);
    angular = constrain(angular, -_angMax, _angMax);
    if (_invAng) angular = -angular;

    // Clean, standard differential-drive IK. NO hidden sign flips here.
    const float halfBase = _wheelBase * 0.5f;
    float vL = linear - halfBase * angular;
    float vR = linear + halfBase * angular;

    int32_t rpmL = (int32_t)velToRpm(vL);
    int32_t rpmR = (int32_t)velToRpm(vR);
    rpmL = (int32_t)((float)rpmL * _leftTrim);
    rpmR = (int32_t)((float)rpmR * _rightTrim);
    _leftRpm  = (float)rpmL;
    _rightRpm = (float)rpmR;

    if (_swap) { _left.setRpm(rpmR); _right.setRpm(rpmL); }
    else       { _left.setRpm(rpmL); _right.setRpm(rpmR); }
    
    // Apply idle pre-spin if enabled
    if (_idlePwmEnabled) {
        // Apply sub-threshold RPM command to keep the driver active
        if (rpmL == 0) _left.setRpm(_idlePwmRpm);
        if (rpmR == 0) _right.setRpm(_idlePwmRpm);
    }
}

void DiffDrive::stop() {
    if (_idlePwmEnabled) {
        // Keep motors "energized" but not moving
        _left.setRpm(_idlePwmRpm);
        _right.setRpm(_idlePwmRpm);
    } else {
        _left.stop();
        _right.stop();
    }
    _leftRpm = 0;
    _rightRpm = 0;
}

void DiffDrive::task() {
    _left.task();   // both buses are independent UARTs -> effectively parallel
    _right.task();
}

void DiffDrive::setBroadcastWrites(bool v) {
    _left.setBroadcastWrites(v);
    _right.setBroadcastWrites(v);
}

void DiffDrive::setSideTrim(float leftTrim, float rightTrim) {
    _leftTrim = leftTrim;
    _rightTrim = rightTrim;
}
