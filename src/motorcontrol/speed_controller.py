import numpy as np
import matplotlib.pyplot as plt

class SpeedControllerPID:
    def __init__(self, motor, encoder, velocity_filter=None,
                 kp=0.2, ki=0.03, kd=0.02,
                 integral_limit=50.0,
                 deadband=1.0, 
                 derivative_alpha=0.2):
        """
        deadband: zona muerta (en deg/s) para ignorar errores minúsculos.
        """
        self.motor = motor
        self.encoder = encoder
        self.velocity_filter = velocity_filter

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.deadband = deadband

        self.integral_error = 0.0
        self.prev_error = 0.0
        self.prev_pos = None
        self.prev_derivative = 0.0  # para suavizar derivada
        self.derivative_alpha=derivative_alpha

    def estimate_velocity_deg_s(self, dt):
        pos = self.encoder.read_position_deg()

        if self.prev_pos is None:
            self.prev_pos = pos
            return 0.0

        delta = pos - self.prev_pos
        if delta > 180.0:
            delta -= 360.0
        elif delta < -180.0:
            delta += 360.0

        raw_velocity = delta / dt
        self.prev_pos = pos

        if self.velocity_filter:
            return self.velocity_filter.filter(raw_velocity)
        else:
            return raw_velocity

    def update(self, desired_velocity_deg_s, dt):
        measured_velocity = self.estimate_velocity_deg_s(dt)
        error = desired_velocity_deg_s - measured_velocity

        # ---------- DEADZONE ----------
        if abs(error) < self.deadband:
            error = 0.0

        # ---------- INTEGRAL ----------
        self.integral_error += error * dt
        self.integral_error = max(min(self.integral_error, self.integral_limit),
                                  -self.integral_limit)

        # ---------- DERIVATIVE ----------
        raw_derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        # Derivative smoothing        
        derivative_error = (1 - self.derivative_alpha) * self.prev_derivative + self.derivative_alpha * raw_derivative

        output = (self.kp * error +
                  self.ki * self.integral_error +
                  self.kd * derivative_error)

        throttle = max(0.0, min(output, 1.0))
        self.motor.set_throttle(throttle)

        self.prev_error = error
        self.prev_derivative = derivative_error

        return {
            "measured_velocity": measured_velocity,
            "error": error,
            "proportional": self.kp * error,
            "integral": self.ki * self.integral_error,
            "derivative": self.kd * derivative_error,
            "throttle": throttle
        }
    

# MotorLogger reutilizable
class SpeedControllerLogger:
    def __init__(self):
        self.time = []
        self.vel_ref = []
        self.vel_real = []
        self.vel_est = []
        self.throttle = []
        self.error = []
        self.P = []
        self.I = []
        self.D = []

    def log(self, t, vel_ref, status, vel_real):
        self.time.append(t)
        self.vel_ref.append(vel_ref)
        self.vel_real.append(vel_real)
        self.vel_est.append(status["measured_velocity"])
        self.throttle.append(status["throttle"])
        self.error.append(status["error"])
        self.P.append(status["proportional"])
        self.I.append(status["integral"])
        self.D.append(status["derivative"])

    def as_dict(self):
        return {k: np.array(v) for k, v in self.__dict__.items()}
    

def plot_speed_pid(logger):
    data = logger.as_dict()

    fig, axes = plt.subplots(4, 1, figsize=(18, 10), sharex=True)

    axes[0].plot(data["time"], data["vel_ref"], label="Ref")
    axes[0].plot(data["time"], data["vel_real"], label="Real")
    axes[0].plot(data["time"], data["vel_est"], label="Estimated", alpha=0.7)
    axes[0].set_ylabel("Velocity (deg/s)")
    axes[0].set_title("Velocity Tracking")
    axes[0].legend()
    axes[0].grid()

    axes[1].plot(data["time"], data["throttle"])
    axes[1].set_ylabel("Throttle")
    axes[1].grid()

    axes[2].plot(data["time"], data["error"])
    axes[2].set_ylabel("Error (deg/s)")
    axes[2].grid()

    axes[3].plot(data["time"], data["P"], label="P")
    axes[3].plot(data["time"], data["I"], label="I")
    axes[3].plot(data["time"], data["D"], label="D")
    axes[3].set_ylabel("PID Terms")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend()
    axes[3].grid()

    plt.tight_layout()
    plt.show()