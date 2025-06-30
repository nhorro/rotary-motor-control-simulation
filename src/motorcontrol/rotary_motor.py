import math
import numpy as np
import matplotlib.pyplot as plt

class RotaryMotor:
    def __init__(self, inertia, friction_viscous, max_torque,
                 friction_coulomb=0.0, external_torque=0.0):
        self.inertia = inertia
        self.friction_viscous = friction_viscous
        self.friction_coulomb = friction_coulomb
        self.external_torque = external_torque
        self.max_torque = max_torque

        self.throttle = 0.0
        self.position = 0.0
        self.position_unwrapped = 0.0
        self.velocity = 0.0

        self.last_net_torque = 0.0

    def set_throttle(self, throttle):
        self.throttle = max(0.0, min(throttle, 1.0))

    def update(self, dt):
        drive_torque = self.max_torque * self.throttle
        viscous = -self.friction_viscous * self.velocity

        if abs(self.velocity) > 1e-4:
            coulomb = -self.friction_coulomb * math.copysign(1, self.velocity)
        else:
            if abs(drive_torque + self.external_torque) > self.friction_coulomb:
                coulomb = -self.friction_coulomb * math.copysign(1, drive_torque + self.external_torque)
            else:
                coulomb = -(drive_torque + self.external_torque)
                viscous = 0.0

        net_torque = drive_torque + viscous + coulomb + self.external_torque
        self.last_net_torque = net_torque  # Guarda para logging

        angular_acc = net_torque / self.inertia

        self.velocity += angular_acc * dt
        self.position += self.velocity * dt
        self.position = self.position % (2 * math.pi)
        self.position_unwrapped += self.velocity * dt

    def get_position_deg(self):
        return math.degrees(self.position)

    def get_position_unwrapped_deg(self):
        return math.degrees(self.position_unwrapped)

    def get_velocity_deg_s(self):
        return math.degrees(self.velocity)

class MotorLogger:
    """
    Guarda el historial de la simulación de un motor rotatorio.
    """
    def __init__(self):
        self.time = []
        self.position_deg = []
        self.velocity_deg_s = []
        self.throttle = []
        self.torque_net = []

    def log(self, t, motor):
        """
        Guarda el estado actual del motor en t.
        """
        self.time.append(t)
        self.position_deg.append(motor.get_position_unwrapped_deg())
        self.velocity_deg_s.append(motor.get_velocity_deg_s())
        self.throttle.append(motor.throttle)
        self.torque_net.append(motor.last_net_torque)

    def as_dict(self):
        import numpy as np
        return {
            "time": np.array(self.time),
            "position": np.array(self.position_deg),
            "velocity": np.array(self.velocity_deg_s),
            "throttle": np.array(self.throttle),
            "torque_net": np.array(self.torque_net)
        }

    
def simulate_motor(motor, throttle_signal,
                               dt=0.01, max_time=30.0,
                               velocity_epsilon=1e-3, min_steps=10, verbose=False):
    logger = MotorLogger()

    steps = len(throttle_signal)
    for step in range(steps):
        t = step * dt
        if t > max_time:
            break

        motor.set_throttle(throttle_signal[step])
        motor.update(dt)

        logger.log(t, motor)

        if verbose:
            print(f"Step {step:04d}: t={t:.2f}s, throttle={motor.throttle:.3f}, "
                  f"vel={motor.get_velocity_deg_s():.3f} deg/s, "
                  f"net torque={motor.last_net_torque:.5f} Nm")

        if step > min_steps and np.allclose([motor.throttle, motor.get_velocity_deg_s()],
                                            [0.0, 0.0],
                                            atol=velocity_epsilon):
            if verbose:
                print(f"Stopped at step {step} (vel & throttle close to zero)")
            break

    return logger

def plot_motor_log(logger):
    data = logger.as_dict()

    fig, axes = plt.subplots(4, 1, figsize=(16, 10), sharex=True)

    axes[0].plot(data["time"], data["position"])
    axes[0].set_ylabel("Position (deg)")
    axes[0].set_title("Angular Position (Unwrapped)")
    axes[0].grid()

    axes[1].plot(data["time"], data["velocity"])
    axes[1].set_ylabel("Velocity (deg/s)")
    axes[1].set_title("Angular Velocity")
    axes[1].grid()

    axes[2].plot(data["time"], data["throttle"])
    axes[2].set_ylabel("Throttle")
    axes[2].set_title("Throttle")
    axes[2].grid()

    axes[3].plot(data["time"], data["torque_net"])
    axes[3].set_ylabel("Torque Net (Nm)")
    axes[3].set_xlabel("Time (s)")
    axes[3].set_title("Net Torque")
    axes[3].grid()

    plt.tight_layout()
    plt.show()


def trapezoidal_profile(ramp_steps=100, hold_steps=200, peak_value=20.0):
    ramp_up = np.linspace(0, peak_value, ramp_steps)
    hold = np.ones(hold_steps) * peak_value
    ramp_down = np.linspace(peak_value, 0.0, ramp_steps)
    return np.concatenate([ramp_up, hold, ramp_down])