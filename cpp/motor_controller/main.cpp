#include "RotaryMotor.hpp"
#include "Encoder.hpp"
#include "MotorControl.h"
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>

class SimMotorActuator : public IMotorActuator {
public:
    explicit SimMotorActuator(RotaryMotor& motor) : motor_(motor) {}

    void send(MotorCommand cmd) override {
        if (cmd == lastCmd_)
            return;  // Evitar spam si no cambió

        lastCmd_ = cmd;
        double throttle = 0.0;
        switch (cmd) {
            case MotorCommand::ALTA_CW:   throttle =  1.0; break;
            case MotorCommand::MEDIA_CW:  throttle =  0.6; break;
            case MotorCommand::BAJA_CW:   throttle =  0.2; break;
            case MotorCommand::BAJA_CCW:  throttle = -0.2; break;
            case MotorCommand::MEDIA_CCW: throttle = -0.6; break;
            case MotorCommand::ALTA_CCW:  throttle = -1.0; break;
            case MotorCommand::STOP:      throttle =  0.0; break;
        }
        motor_.set_throttle(throttle);
    }

    MotorCommand get_last_command() const { return lastCmd_; }

private:
    RotaryMotor& motor_;
    MotorCommand lastCmd_;
};
    

int main() {
    const double dt = 0.01;  // 10ms
    const double total_time = 10.0;

    // Crear simulador
    RotaryMotor motor(
        2.0, // inertia
        0.3, // friction_viscous
        0.5,  // max_torque
        0.3, // friction_coulomb
        0. // external_torque
    );

    motor.reset(160.0);

    Encoder encoder(&motor, 0.);
    SimMotorActuator actuator(motor);
    MotorControl control(actuator);

    control.start();

    // Definir objetivo
    control.setManualTarget(90.0);
    control.setMode(MotorControl::Mode::MANUAL);

    std::ofstream log("manual_control_test.csv");
    log << "t,pos_deg,vel_deg_s,target,throttle,cmd\n";

    // Simulación
    for (double t = 0.0; t < total_time; t += dt) {

        // Medir posición
        double pos = encoder.read_position_deg();

        // Pasar lectura al controlador
        control.updateEncoder(pos);

        // Simular motor
        motor.update(dt);

        // Loguear
        log << t << "," << pos << "," << motor.get_velocity_deg_s() << "," << 90.0 << "," << motor.get_throttle() << "," << static_cast<int>(actuator.get_last_command()) << "\n";

        
        // Esperar para simular tiempo real
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    control.stop();
    log.close();

    std::cout << "Test finished. Log saved to manual_control_test.csv\n";
    return 0;
}
