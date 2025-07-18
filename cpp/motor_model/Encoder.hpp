// Encoder.hpp
#pragma once
#include <random>
#include "IMotor.hpp"

class Encoder {
public:
    Encoder(IMotor* motor, double noise_std_deg = 0.1);

    double read_position_deg();

private:
    IMotor* motor_;
    double noise_std_deg_;
    std::mt19937 rng_;
    std::normal_distribution<double> dist_;
};
