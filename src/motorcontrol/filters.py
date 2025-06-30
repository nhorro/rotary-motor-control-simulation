from collections import deque

class MovingAverageyFilter:
    def __init__(self, window_size=5):
        """
        Filtro de media móvil.
        window_size: número de muestras a promediar.
        """
        self.history = deque(maxlen=window_size)

    def filter(self, value):
        self.history.append(value)
        return sum(self.history) / len(self.history)
    
class ExponentialFilter:
    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self.last = None

    def filter(self, value):
        if self.last is None:
            self.last = value
        else:
            self.last = self.alpha * value + (1 - self.alpha) * self.last
        return self.last    