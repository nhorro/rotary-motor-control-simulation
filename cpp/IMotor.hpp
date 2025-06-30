// IMotor.hpp
#pragma once

class IMotor {
public:
    virtual void set_throttle(double t) = 0;
    virtual void update(double dt) = 0;

    virtual double get_position_deg() const = 0;
    virtual double get_velocity_deg_s() const = 0;

    virtual ~IMotor() = default;
};
