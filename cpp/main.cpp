#include "RotaryMotor.hpp"
#include "Encoder.hpp"
#include <iostream>
#include <fstream>
#include <vector>

int main() {
    // Motor: usa tus parámetros reales
    RotaryMotor motor(2.0, 0.05, 0.5, 0.02);
    Encoder encoder(&motor, 0.2);

    double dt = 0.01;

    // --- 1) Genera la rampa trapezoidal bidireccional ---
    std::vector<double> throttle_profile;

    int ramp_steps = 200;
    int hold_steps = 200;

    // Ramp up: 0 → 1.0
    for (int i = 0; i < ramp_steps; ++i) {
        throttle_profile.push_back(static_cast<double>(i) / ramp_steps);
    }
    // Hold at 1.0
    for (int i = 0; i < hold_steps; ++i) {
        throttle_profile.push_back(1.0);
    }
    // Ramp down: 1.0 → -1.0
    for (int i = 0; i < ramp_steps * 2; ++i) {
        double frac = static_cast<double>(i) / (ramp_steps * 2);
        throttle_profile.push_back(1.0 - 2.0 * frac);
    }
    // Hold at -1.0
    for (int i = 0; i < hold_steps * 4; ++i) {
        throttle_profile.push_back(-1.0);
    }
    // Ramp up: -1.0 → 0.0
    for (int i = 0; i < ramp_steps; ++i) {
        double frac = static_cast<double>(i) / ramp_steps;
        throttle_profile.push_back(-1.0 + frac);
    }

    // --- 2) Abre archivo CSV ---
    std::ofstream csv_file("motor_log.csv");
    csv_file << "time,position_deg,velocity_deg_s,throttle,net_torque\n";

    // --- 3) Simula ---
    for (size_t i = 0; i < throttle_profile.size(); ++i) {
        double t = i * dt;

        motor.set_throttle(throttle_profile[i]);
        motor.update(dt);

        double pos = motor.get_position_deg();
        double vel = motor.get_velocity_deg_s();
        double net_torque = motor.get_last_net_torque();

        csv_file << t << "," << pos << "," << vel << ","
                 << throttle_profile[i] << "," << net_torque << "\n";
    }

    csv_file.close();

    std::cout << "Log saved to motor_log.csv\n";
    return 0;
}
