import math
import matplotlib.pyplot as plt
import numpy as np

class TrapezoidalPlanner:
    def __init__(self, pos0, posf, vmax, amax):
        self.pos0 = pos0
        self.posf = posf
        self.vmax = vmax
        self.amax = amax

        self.direction = 1 if posf >= pos0 else -1
        self.D = abs(posf - pos0)

        self.Ta = self.vmax / self.amax
        self.Da = 0.5 * self.amax * self.Ta ** 2

        if 2 * self.Da < self.D:
            self.Tc = (self.D - 2 * self.Da) / self.vmax
        else:
            self.Ta = math.sqrt(self.D / self.amax)
            self.vmax = self.amax * self.Ta
            self.Tc = 0.0

        self.Tf = 2 * self.Ta + self.Tc

    def get_ref(self, t):
        if t < 0:
            pos = self.pos0
            vel = 0.0
        elif t < self.Ta:
            vel = self.amax * t
            pos = self.pos0 + 0.5 * self.amax * t ** 2
        elif t < self.Ta + self.Tc:
            vel = self.vmax
            pos = self.pos0 + self.Da + self.vmax * (t - self.Ta)
        elif t < self.Tf:
            td = t - (self.Ta + self.Tc)
            vel = self.vmax - self.amax * td
            pos = self.pos0 + self.Da + self.vmax * self.Tc + self.vmax * td - 0.5 * self.amax * td ** 2
        else:
            pos = self.posf
            vel = 0.0

        return self.direction * pos, self.direction * vel

class PositionControllerPID:
    def __init__(self, kp=2.0, ki=0.0, kd=0.0, integral_limit=50.0, deadband=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.deadband = deadband
        self.integral_error = 0.0
        self.prev_error = 0.0

    def update(self, pos_ref, pos_real, dt):
        error = pos_ref - pos_real
        if abs(error) < self.deadband:
            error = 0.0

        self.integral_error += error * dt
        self.integral_error = max(min(self.integral_error, self.integral_limit),
                                  -self.integral_limit)

        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        vel_ref = self.kp * error + self.ki * self.integral_error + self.kd * derivative
        self.prev_error = error

        return {
            "vel_ref": vel_ref,
            "error": error,
            "P": self.kp * error,
            "I": self.ki * self.integral_error,
            "D": self.kd * derivative
        }

class PositionControllerLogger:
    def __init__(self):
        self.time = []
        self.pos_ref = []
        self.pos_real = []
        self.vel_ref = []
        self.vel_real = []
        self.throttle = []
        self.pos_error = []
        self.P = []
        self.I = []
        self.D = []

    def log(self, t, pos_ref, pos_real, vel_pid_status, speed_pid_status):
        self.time.append(t)
        self.pos_ref.append(pos_ref)
        self.pos_real.append(pos_real)
        self.vel_ref.append(vel_pid_status["vel_ref"])
        self.vel_real.append(speed_pid_status["measured_velocity"])
        self.throttle.append(speed_pid_status["throttle"])
        self.pos_error.append(vel_pid_status["error"])
        self.P.append(vel_pid_status["P"])
        self.I.append(vel_pid_status["I"])
        self.D.append(vel_pid_status["D"])

    def as_dict(self):
        return {k: np.array(v) for k, v in self.__dict__.items()}

def plot_position_pid(logger):
    data = logger.as_dict()

    fig, axes = plt.subplots(4, 1, figsize=(18, 12), sharex=True)

    axes[0].plot(data["time"], data["pos_ref"], label="Ref")
    axes[0].plot(data["time"], data["pos_real"], label="Real")
    axes[0].set_ylabel("Position (deg)")
    axes[0].set_title("Position Tracking")
    axes[0].legend()
    axes[0].grid()

    axes[1].plot(data["time"], data["vel_ref"], label="Ref")
    axes[1].plot(data["time"], data["vel_real"], label="Real")
    axes[1].set_ylabel("Velocity (deg/s)")
    axes[1].set_title("Velocity Tracking")
    axes[1].legend()
    axes[1].grid()

    axes[2].plot(data["time"], data["throttle"])
    axes[2].set_ylabel("Throttle")
    axes[2].grid()

    axes[3].plot(data["time"], data["P"], label="P")
    axes[3].plot(data["time"], data["I"], label="I")
    axes[3].plot(data["time"], data["D"], label="D")
    axes[3].set_ylabel("Position PID Terms")
    axes[3].set_xlabel("Time (s)")
    axes[3].legend()
    axes[3].grid()

    plt.tight_layout()
    plt.show()
