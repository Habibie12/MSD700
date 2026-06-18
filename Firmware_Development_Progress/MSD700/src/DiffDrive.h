// DiffDrive.h
// Object-oriented motor control for the MSD700 differential-drive base.
//
//   BLVMotor  - one Oriental Motor BLV driver on one RS-485 / Modbus bus.
//               Non-blocking write state machine, no callbacks needed.
//   DiffDrive - owns two BLVMotors + the kinematics. Single place where
//               ALL direction/sign handling lives (no hidden negations).
//
// Target: Arduino Mega 2560 + emelianov/modbus-esp8266 (ModbusRTU).
#include <Arduino.h>

#ifndef DIFFDRIVE_H
#define DIFFDRIVE_H

#include <ModbusRTU.h>

// ---- BLV driver register map ----
#define BLV_REG_SPEED   0x0480   // operation-data speed No.0 (32-bit, 2 regs)
#define BLV_REG_CMD     0x007C   // driver input command   (2 regs)
#define BLV_BIT_M0      0x0001   // NET-IN0 = operation-data select bit 0
#define BLV_BIT_M1      0x0002   // NET-IN1 = operation-data select bit 1
#define BLV_BIT_M2      0x0004   // NET-IN2 = operation-data select bit 2
#define BLV_BIT_FWD     0x0008
#define BLV_BIT_REV     0x0010
#define BLV_BIT_STOP    0x0000

// =====================================================================
// BLVMotor : one motor on one bus
// =====================================================================
class BLVMotor {
public:
    BLVMotor(ModbusRTU &bus, uint8_t slaveId, bool invert = false);

    void begin(int32_t rpmMin, int32_t rpmMax,
               int32_t changeThreshold, uint32_t cmdResendMs,
               bool broadcastWrites = false);

    // Signed RPM. Positive = forward (BEFORE the per-motor invert flag).
    void setRpm(int32_t rpm);
    void stop() { setRpm(0); }

    // Non-blocking. Call every loop pass. Drives the write state machine.
    void task();

    void setInvert(bool v) { _invert = v; }
    void setBroadcastWrites(bool v) { _broadcastWrites = v; }
    void setOperationDataNo(uint8_t no);
    bool invert() const    { return _invert; }
    int32_t commandedRpm() const { return _pendingRpm; }

private:
    enum State : uint8_t { IDLE, WRITE_SPEED, WRITE_CMD, WRITE_SPEED_AND_CMD };
    int32_t clampMag(int32_t rpm) const;
    uint16_t operationDataBits() const;

    ModbusRTU &_bus;
    uint8_t   _slave;
    bool      _invert;
    bool      _broadcastWrites;
    uint8_t   _operationDataNo;

    int32_t  _rpmMin, _rpmMax, _threshold;
    uint32_t _cmdResendMs;

    State    _state;
    int32_t  _pendingRpm, _lastRpm;
    uint16_t _pendingCmd, _lastCmd;
    uint16_t _speedBuf[2], _cmdBuf[2];
    uint32_t _lastCmdWriteMs;
    bool     _wasStopped;  // Track if motor was previously stopped
};

// =====================================================================
// DiffDrive : the robot. Kinematics + two motors.
//   linear  : m/s,   + = forward
//   angular : rad/s, + = counter-clockwise (turn left)   [ROS REP-103]
// =====================================================================
class DiffDrive {
public:
    DiffDrive(ModbusRTU &leftBus,  uint8_t leftSlave,
              ModbusRTU &rightBus, uint8_t rightSlave);

    void begin(float wheelRadius, float wheelBase, float reductionRatio,
               float linMax, float angMax,
               int32_t rpmMin, int32_t rpmMax,
               int32_t changeThreshold, uint32_t cmdResendMs);

    void drive(float linear, float angular);
    void stop();
    void task();   // call every loop pass

    // -------- direction calibration (see DriveCalibration in .cpp) --------
    void setInvertLeft(bool v)   { _left.setInvert(v);  }
    void setInvertRight(bool v)  { _right.setInvert(v); }
    void setSwapSides(bool v)    { _swap   = v; }
    void setInvertAngular(bool v){ _invAng = v; }
    void setBroadcastWrites(bool v);
    void setSideTrim(float leftTrim, float rightTrim);
    void setOperationDataNo(uint8_t no);
    
    // Idle pre-spin feature for instant response
    void setIdlePreSpin(bool enable);

    float leftRpm()  const { return _leftRpm; }
    float rightRpm() const { return _rightRpm; }

private:
    float velToRpm(float v) const;

    BLVMotor _left;
    BLVMotor _right;

    float _wheelRadius, _wheelBase, _reduction;
    float _leftTrim, _rightTrim;
    float _linMax, _angMax;
    bool  _swap, _invAng;
    float _leftRpm, _rightRpm;
    
    // Idle pre-spin feature
    bool _idlePwmEnabled;
    int32_t _idlePwmRpm;
};

#endif // DIFFDRIVE_H
