// DiffDrive.cpp
#include "DiffDrive.h"
#include <math.h>

static inline int32_t i32abs(int32_t v) { return v < 0 ? -v : v; }

// =====================================================================
// BLVMotor
// =====================================================================
BLVMotor::BLVMotor(ModbusRTU &bus, uint8_t slaveId, bool invert)
    : _bus(bus), _slave(slaveId), _invert(invert),
      _broadcastWrites(false),
      _operationDataNo(2),
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

void BLVMotor::setOperationDataNo(uint8_t no) {
    if (no > 7) no = 7;
    _operationDataNo = no;
    _lastCmd = 0xFFFF;  // force a command refresh with the new M-bit pattern
}

uint16_t BLVMotor::operationDataBits() const {
    uint16_t bits = 0;
    if (_operationDataNo & 0x01) bits |= BLV_BIT_M0;
    if (_operationDataNo & 0x02) bits |= BLV_BIT_M1;
    if (_operationDataNo & 0x04) bits |= BLV_BIT_M2;
    return bits;
}

int32_t BLVMotor::clampMag(int32_t rpm) const {
    int32_t m = i32abs(rpm);
    if (m > 0 && m < _rpmMin) m = 0;        // sub-minimum -> treat as stop
    if (m > _rpmMax)          m = _rpmMax;
    return m;
}

void BLVMotor::setRpm(int32_t rpm) {
    if (_invert) rpm = -rpm;                // single point of motor inversion
    int32_t mag = clampMag(rpm);
    
    // Determine command
    uint16_t newCmd;
    if      (mag == 0)  newCmd = BLV_BIT_STOP;
    else if (rpm > 0)   newCmd = BLV_BIT_FWD;
    else                newCmd = BLV_BIT_REV;
    newCmd |= operationDataBits();
    
    // Track stop state for combined write optimization
    if (newCmd == BLV_BIT_STOP) {
        _wasStopped = true;
    } else if (mag > 0) {
        _wasStopped = false;
    }
    
    _pendingRpm = mag;
    _pendingCmd = newCmd;
}

// Non-blocking. Uses _bus.slave() (0 == bus free) to gate dispatch, so the
// LEFT motor NEVER waits on the RIGHT bus and vice-versa.
void BLVMotor::task() {
    _bus.task();                 // pump the Modbus engine
    if (_bus.slave()) return;    // a transaction is in flight on this bus

    const uint32_t now = millis();

    if (_state == IDLE) {
        bool speedChanged = (i32abs(_pendingRpm - _lastRpm) > _threshold);
        bool cmdChanged   = (_pendingCmd != _lastCmd);
        // Keep-alive: re-assert the command every _cmdResendMs, INCLUDING stop.
        // Re-sending STOP keeps this RS-485 bus continuously talking while the
        // robot is idle, so the first move after sitting still does not hit a
        // cold bus / first-frame timeout. With both motors doing this, the two
        // sides stay in lockstep and start together. This is the correct,
        // non-moving version of "always send a command to both wheels".
        bool cmdStale     = (now - _lastCmdWriteMs > _cmdResendMs);
        
        // ONE Modbus frame per pass - NEVER two back-to-back. Starting from a
        // stop simply means "speed changed", which routes through WRITE_SPEED.
        // Because the command also changed (STOP -> FWD/REV), WRITE_SPEED then
        // chains to WRITE_CMD on a LATER pass, only after the speed frame has
        // fully completed on the wire. This removes the racy double-write that
        // was corrupting frames (the red C-ERR storm) and starving the bus.
        if (speedChanged)                _state = WRITE_SPEED;
        else if (cmdChanged || cmdStale) _state = WRITE_CMD;
        else return;                 // nothing to send
    }

    // (The old WRITE_SPEED_AND_CMD combined-write state was removed: issuing a
    // second frame after a single _bus.task() did not let the first transaction
    // finish, so the two frames collided and the driver flagged C-ERR. Speed and
    // command are now always sent as two clean, separate transactions below.)

    if (_state == WRITE_SPEED) {
        _speedBuf[0] = (uint16_t)((_pendingRpm >> 16) & 0xFFFF);
        _speedBuf[1] = (uint16_t)( _pendingRpm        & 0xFFFF);
        uint16_t speedReg = BLV_REG_SPEED + ((uint16_t)_operationDataNo * 2);
        uint8_t target = _broadcastWrites ? 0 : _slave;
        if (_bus.writeHreg(target, speedReg, _speedBuf, 2)) {
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
        // If a motor should be stopped but pre-spin is active, give it a tiny RPM
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

void DiffDrive::setOperationDataNo(uint8_t no) {
    _left.setOperationDataNo(no);
    _right.setOperationDataNo(no);
}
