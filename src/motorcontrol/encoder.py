import random

class Encoder:
    def __init__(self, motor, noise_std_deg=0.1):
        """
        motor: instancia de RotaryMotor
        noise_std_deg: desviación estándar del ruido en grados
        """
        self.motor = motor
        self.noise_std_deg = noise_std_deg

    def read_position_deg(self):
        """
        Devuelve la posición envuelta (0–360°) con ruido gaussiano
        """
        true_position = self.motor.get_position_deg()
        noise = random.gauss(0, self.noise_std_deg)
        noisy_position = (true_position + noise) % 360.0
        return noisy_position