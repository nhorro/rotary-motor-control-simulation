#pragma once

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cmath>

enum class MotorCommand {
    STOP,
    ALTA_CW,
    MEDIA_CW,
    BAJA_CW,
    BAJA_CCW,
    MEDIA_CCW,
    ALTA_CCW
};

class IMotorActuator {
public:
    virtual ~IMotorActuator() = default;
    virtual void send(MotorCommand cmd) = 0;
};

enum class RotationDirection {
    SHORTEST,
    CW,
    CCW
};

class MotorControl {
public:
    enum class Mode { STOPPED, MANUAL, SCANNING, ROTATING };

    explicit MotorControl(IMotorActuator& actuator);
    ~MotorControl();

    void start();
    void stop();

    // Thread-safe calls from outside
    void setMode(Mode mode);
    void setManualTarget(double angleDeg);
    void setScanRange(double startDeg, double endDeg);
    void setFixedSpeed(double speedDegPerSec);
    void updateEncoder(double angleDeg);

    Mode getMode() const { return mode_; }
    double getScanRangeStartAngle() const { return scanStart_; }
    double getScanRangeEndAngle() const { return scanEnd_; }
    double getManualTarget() const { return manualTarget_; }

    enum class ScanState { GOTO_START, SCANNING_FORWARD, SCANNING_BACKWARD };
    ScanState getScanState() const { return scanState_; }
    
    

private:
    void controlLoop();
    void handleStopped();
    void handleManual();
    void handleRotating();
    void handleScanning();

    

    std::mutex mtx_;
    std::condition_variable cv_;
    Mode mode_ = Mode::STOPPED;

    double currentAngle_ = 0.0;
    double previousAngle_ = 0.0;
    double manualTarget_ = 0.0;
    double scanStart_ = 0.0;
    double scanEnd_ = 0.0;
    double speedDegPerSec_ = 0.0;

    std::atomic<bool> running_;
    std::thread worker_;
    ScanState scanState_ = ScanState::GOTO_START;
    static constexpr double kAngleTolerance = 0.5; // deg

    IMotorActuator& actuator_;

    void rotateTo(double targetDeg, RotationDirection direction = RotationDirection::SHORTEST);
    void rotateAtFixedSpeed(double speedDegPerSec);
    void stopMotor();
};
