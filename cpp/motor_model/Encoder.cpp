// Encoder.cpp
#include "Encoder.hpp"
#include <cmath>

Encoder::Encoder(IMotor* motor, double noise_std_deg)
    : motor_(motor),
      noise_std_deg_(noise_std_deg),
      rng_(std::random_device{}()),
      dist_(0.0, noise_std_deg) {}

double Encoder::read_position_deg() {
    double true_position = motor_->get_position_deg();
    double noise = dist_(rng_);
    double noisy_position = std::fmod(true_position + noise, 360.0);
    if (noisy_position < 0) noisy_position += 360.0;
    return noisy_position;
}
