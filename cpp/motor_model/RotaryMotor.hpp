// RotaryMotor.hpp
#pragma once
#include <cmath>
#include "IMotor.hpp"

class RotaryMotor : public IMotor {
public:
    RotaryMotor(double inertia, double friction_viscous, double max_torque,
                double friction_coulomb = 0.0, double external_torque = 0.0);

    void reset(double position);
    void set_throttle(double throttle) override;
    double get_throttle() const override;
    void update(double dt) override;

    double get_position_deg() const override;
    double get_position_unwrapped_deg() const;
    double get_velocity_deg_s() const override;
    double get_last_net_torque() const;

private:
    double inertia_;
    double friction_viscous_;
    double friction_coulomb_;
    double external_torque_;
    double max_torque_;

    double throttle_;             // [-1.0, 1.0]  => sentido y magnitud
    double position_;
    double position_unwrapped_;
    double velocity_;
    double last_net_torque_;
};

