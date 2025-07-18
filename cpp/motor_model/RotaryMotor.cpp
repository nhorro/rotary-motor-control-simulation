// RotaryMotor.cpp
#include "RotaryMotor.hpp"
#include <algorithm>

RotaryMotor::RotaryMotor(double inertia, double friction_viscous, double max_torque,
                         double friction_coulomb, double external_torque)
    : inertia_(inertia),
      friction_viscous_(friction_viscous),
      friction_coulomb_(friction_coulomb),
      external_torque_(external_torque),
      max_torque_(max_torque),
      throttle_(0.0),
      position_(0.0),
      position_unwrapped_(0.0),
      velocity_(0.0),
      last_net_torque_(0.0) {}


void RotaryMotor::reset(double position) {
    position_ = position;
    position_unwrapped_ = position;
    throttle_ = 0;
    velocity_ = 0;
    last_net_torque_ = 0;
}

void RotaryMotor::set_throttle(double throttle) {
    // Nuevo rango: [-1.0, 1.0]
    throttle_ = std::clamp(throttle, -1.0, 1.0);
}

double RotaryMotor::get_throttle() const{
    return throttle_;
}

void RotaryMotor::update(double dt) {
    double drive_torque = max_torque_ * throttle_;
    double viscous = -friction_viscous_ * velocity_;

    double coulomb = 0.0;
    if (std::abs(velocity_) > 1e-4) {
        coulomb = -friction_coulomb_ * std::copysign(1.0, velocity_);
    } else {
        if (std::abs(drive_torque + external_torque_) > friction_coulomb_) {
            coulomb = -friction_coulomb_ * std::copysign(1.0, drive_torque + external_torque_);
        } else {
            coulomb = -(drive_torque + external_torque_);
            viscous = 0.0;
        }
    }

    double net_torque = drive_torque + viscous + coulomb + external_torque_;
    last_net_torque_ = net_torque;

    double angular_acc = net_torque / inertia_;
    velocity_ += angular_acc * dt;

    position_ += velocity_ * dt;
    position_ = std::fmod(position_, 2 * M_PI);
    if (position_ < 0) position_ += 2 * M_PI;

    position_unwrapped_ += velocity_ * dt;
}

double RotaryMotor::get_position_deg() const {
    return position_ * 180.0 / M_PI;
}

double RotaryMotor::get_position_unwrapped_deg() const {
    return position_unwrapped_ * 180.0 / M_PI;
}

double RotaryMotor::get_velocity_deg_s() const {
    return velocity_ * 180.0 / M_PI;
}

double RotaryMotor::get_last_net_torque() const {
    return last_net_torque_;
}
