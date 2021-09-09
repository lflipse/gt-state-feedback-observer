import numpy as np
import matplotlib.pyplot as plt
import math

class Reference:
    def __init__(self, duration):
        bw = 4
        period = np.array([5, 8, 11, 17, 26, 37, 49, 57, 73, 97])
        self.duration = (period[7] * 2 * np.pi) / bw
        frequencies = 2 * np.pi * period / self.duration
        phases = [5.79893804, 0.2467642, -1.362783088, 2.56145111, -1.92977185, -1.1689079, -0.61034581,
                  -0.75180265, -0.03232366, 3.2509144]
        print("duration = ", self.duration)
        print("frequencies = ", frequencies)
        self.amp = 0.3
        amplitude = self.amp * np.array([1, 1, 1, 1, 1, 1, 1, 0.1, 0.1, 0.1])
        self.forcing_function = {
            'period': period,
            'phases': phases,
            'amplitude': amplitude,
        }

        self.lw = 4
        self.title_size = 24
        self.label_size = 22
        self.legend_size = 13
        self.csfont = {'fontname': 'Georgia', 'size': self.title_size}
        self.hfont = {'fontname': 'Georgia', 'size': self.label_size}

        # Colors
        tud_blue = "#0066A2"
        tud_black = "#000000"
        tud_grey = "#808080"
        tud_red = "#c3312f"
        tud_green = "#00A390"
        tud_yellow = "#F1BE3E"

        plt.figure()
        for i in range(len(period)):
            plt.plot(frequencies[i], amplitude[i], color=tud_blue, linewidth=self.lw, marker='o')
            plt.plot([frequencies[i], frequencies[i]], [0, amplitude[i]], tud_blue, linewidth=self.lw, alpha=0.7)

        plt.title("Frequency domain reference", **self.csfont)
        plt.xlabel("Frequency (rad/s)", **self.hfont)
        plt.ylabel("Amplitude (-)", **self.hfont)
        plt.xlim(frequencies[0] - 0.1, frequencies[-1] + 10)
        plt.ylim(0.01, amplitude[0] + 0.1)
        plt.yscale("log")
        plt.xscale("log")
        plt.tight_layout(pad=1)

        # Show forcing function:
        fs = 100
        n = int(fs * self.duration)
        t = np.array(range(n)) / fs
        r = np.zeros(n)
        for i in range(n):
            ref = self.generate_reference(t[i]+5)
            r[i] = ref[0]

        plt.figure()
        plt.plot(t, r, tud_blue, linewidth=self.lw)
        plt.title("Time domain reference", **self.csfont)
        plt.xlabel("Time (s)", **self.hfont)
        plt.ylabel("Amplitude (-)", **self.hfont)
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)


    def generate_reference(self, t):
        period = self.forcing_function["period"]
        phases = self.forcing_function["phases"]
        amplitude = self.forcing_function["amplitude"]
        reference_position = 0
        reference_velocity = 0
        for i in range(10):
            wt = (period[i] / self.duration) * (2 * np.pi)
            reference_position += amplitude[i] * math.sin(wt * t + phases[i])
            reference_velocity += amplitude[i] * wt * math.cos(wt * t + phases[i])
        ref = np.array([reference_position, reference_velocity])
        return ref
