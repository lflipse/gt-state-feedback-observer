import numpy as np

class Reference:
    def __init__(self):
        self.a = 1

    def generate_reference(self, T, f_max, f_min, increments):
        x_d = 30 * np.pi / 180
        fs = f_max
        r = x_d * np.sin(2 * np.pi * fs * T)

        #
        # for i in range(increments):
        #     fs = f_min + i * (f_max - f_min) / increments
        #     phi = np.random.randn()
        #     r += x_d * np.sin(2 * np.pi * fs * T + phi)

        return r