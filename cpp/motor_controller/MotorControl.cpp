#include "MotorControl.h"
#include <chrono>
#include <iostream>

using namespace std::chrono_literals;

static double angular_error(double from, double to) {
    double diff = std::fmod(to - from + 540.0, 360.0) - 180.0;
    return diff;
}

MotorControl::MotorControl(IMotorActuator& actuator)
    : actuator_(actuator), running_(false) {}

MotorControl::~MotorControl() {
    stop();
}

void MotorControl::start() {
    running_ = true;
    worker_ = std::thread(&MotorControl::controlLoop, this);
}

void MotorControl::stop() {
    running_ = false;
    cv_.notify_all();
    if (worker_.joinable())
        worker_.join();
}

void MotorControl::setMode(Mode mode) {
    std::lock_guard<std::mutex> lock(mtx_);
    mode_ = mode;
    if (mode == Mode::SCANNING)
        scanState_ = ScanState::GOTO_START;
    cv_.notify_all();
}

void MotorControl::setManualTarget(double angleDeg) {
    std::lock_guard<std::mutex> lock(mtx_);
    manualTarget_ = angleDeg;
    cv_.notify_all();
}

void MotorControl::setScanRange(double startDeg, double endDeg) {
    std::lock_guard<std::mutex> lock(mtx_);
    scanStart_ = startDeg;
    scanEnd_ = endDeg;
    cv_.notify_all();
}

void MotorControl::setFixedSpeed(double speedDegPerSec) {
    std::lock_guard<std::mutex> lock(mtx_);
    speedDegPerSec_ = speedDegPerSec;
    cv_.notify_all();
}

void MotorControl::updateEncoder(double angleDeg) {
    std::lock_guard<std::mutex> lock(mtx_);
    previousAngle_ = currentAngle_;
    currentAngle_ = angleDeg;
}


void MotorControl::controlLoop() {
    while (running_) {
        Mode currentMode;
        {
            std::unique_lock<std::mutex> lock(mtx_);
            cv_.wait_for(lock, 10ms);
            currentMode = mode_;
        }

        switch (currentMode) {
            case Mode::STOPPED: handleStopped(); break;
            case Mode::MANUAL:  handleManual(); break;
            case Mode::SCANNING: handleScanning(); break;
            case Mode::ROTATING: handleRotating(); break;
        }
    }
}

void MotorControl::handleStopped() {
    stopMotor();
}

void MotorControl::handleManual() {
    double target, current, velocity;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        target = manualTarget_;
        current = currentAngle_;
    }

    double error = angular_error(current, target);
    if (std::abs(error) < kAngleTolerance) {
        setMode(Mode::STOPPED);
        actuator_.send(MotorCommand::STOP);
        return;
    }

    rotateTo(target);
}


void MotorControl::handleRotating() {
    double speed;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        speed = speedDegPerSec_;
    }
    rotateAtFixedSpeed(speed);
}

void MotorControl::handleScanning() {
    double start, end, current;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        start = scanStart_;
        end = scanEnd_;
        current = currentAngle_;
    }

    auto angleDiff = [](double a, double b) {
        double diff = fmod(b - a + 360.0, 360.0);
        return diff;
    };

    switch (scanState_) {
        case ScanState::GOTO_START:
            if (std::abs(angleDiff(current, start)) < kAngleTolerance)
                scanState_ = ScanState::SCANNING_FORWARD;
            else {                
                rotateTo(start);
            }
                
            break;

        case ScanState::SCANNING_FORWARD:
            if (angleDiff(current, end) < kAngleTolerance)
                scanState_ = ScanState::SCANNING_BACKWARD;
            else {                
                rotateTo(end);
            }
            break;

        case ScanState::SCANNING_BACKWARD:
            if (angleDiff(current, start) < kAngleTolerance)
                scanState_ = ScanState::SCANNING_FORWARD;
            else {                
                rotateTo(start);
            }                
            break;
    }
}

void MotorControl::rotateTo(double targetDeg, RotationDirection dir) {
    double current;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        current = currentAngle_;
    }

    // Error angular según dirección deseada
    double error;
    switch (dir) {
        case RotationDirection::CW:
            error = fmod(targetDeg - current + 360.0, 360.0);
            if (error == 0) error = 0.0;
            break;
        case RotationDirection::CCW:
            error = -fmod(current - targetDeg + 360.0, 360.0);
            if (error == 0) error = 0.0;
            break;
        case RotationDirection::SHORTEST:
        default:
            error = angular_error(current, targetDeg); // entre -180 y +180
            break;
    }

    if (std::abs(error) < kAngleTolerance) {
        actuator_.send(MotorCommand::STOP);
        return;
    }

    // TODO: podés usar velocidad también, si querés extenderlo
    double absError = std::abs(error);
    MotorCommand cmd;

    if (error > 0) {
        if (absError > 30)      cmd = MotorCommand::ALTA_CW;
        else if (absError > 10) cmd = MotorCommand::MEDIA_CW;
        else                    cmd = MotorCommand::BAJA_CW;
    } else {
        if (absError > 30)      cmd = MotorCommand::ALTA_CCW;
        else if (absError > 10) cmd = MotorCommand::MEDIA_CCW;
        else                    cmd = MotorCommand::BAJA_CCW;
    }

    actuator_.send(cmd);
}




void MotorControl::rotateAtFixedSpeed(double speedDegPerSec) {
    if (std::abs(speedDegPerSec) < 1e-2) {
        actuator_.send(MotorCommand::STOP);
        return;
    }

    double absSpeed = std::abs(speedDegPerSec);
    MotorCommand cmd;

    if (speedDegPerSec > 0) {
        if (absSpeed > 30)      cmd = MotorCommand::ALTA_CW;
        else if (absSpeed > 10) cmd = MotorCommand::MEDIA_CW;
        else                    cmd = MotorCommand::BAJA_CW;
    } else {
        if (absSpeed > 30)      cmd = MotorCommand::ALTA_CCW;
        else if (absSpeed > 10) cmd = MotorCommand::MEDIA_CCW;
        else                    cmd = MotorCommand::BAJA_CCW;
    }

    actuator_.send(cmd);
}

void MotorControl::stopMotor() {
    actuator_.send(MotorCommand::STOP);
}
