import numpy as np
import matplotlib.pyplot as plt
import math
import pandas as pd

class Reference:
    def __init__(self, duration):
        bw = 5
        self.amp = 0.3
        self.duration = (self.periods[10] * 2 * np.pi) / bw
        frequencies = 2 * np.pi * self.periods / self.duration
        print("duration = ", self.duration)
        print("frequencies = ", frequencies)
        self.forcing_function = {
            'periods': self.periods,
            'phases': self.phases,
            'amplitude': self.amp * self.amplitudes,
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
        for i in range(len(self.periods)):
            plt.plot(frequencies[i], self.amplitudes[i], color=tud_blue, linewidth=self.lw, marker='o')
            plt.plot([frequencies[i], frequencies[i]], [0, self.amplitudes[i]], tud_blue, linewidth=self.lw, alpha=0.7)

        plt.title("Frequency domain reference", **self.csfont)
        plt.xlabel("Frequency (rad/s)", **self.hfont)
        plt.ylabel("Amplitude (-)", **self.hfont)
        plt.xlim(frequencies[0] - 0.1, frequencies[-1] + 10)
        plt.ylim(0.01, self.amplitudes[0] + 0.1)
        plt.yscale("log")
        plt.xscale("log")
        plt.tight_layout(pad=1)

        # Show forcing function:
        fs = 100
        n = int(fs * self.duration)
        t = np.array(range(n)) / fs
        r = np.zeros(n)
        for i in range(n):
            ref = self.generate_reference(t[i])
            r[i] = ref[0]

        plt.figure()
        plt.plot(t, r, tud_blue, linewidth=self.lw)
        plt.title("Time domain reference", **self.csfont)
        plt.xlabel("Time (s)", **self.hfont)
        plt.ylabel("Amplitude (-)", **self.hfont)
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)


    def generate_reference(self, t):
        period = self.forcing_function["periods"]
        phases = self.forcing_function["phases"]
        amplitude = self.forcing_function["amplitudes"]
        reference_position = 0
        reference_velocity = 0
        for i in range(len(period)):
            wt = (period[i] / self.duration) * (2 * np.pi)
            reference_position += amplitude[i] * math.sin(wt * t + phases[i])
            reference_velocity += amplitude[i] * wt * math.cos(wt * t + phases[i])
        ref = np.array([reference_position, reference_velocity])
        return ref

    def load_data(self):
        try:
            df_data = pd.read_csv("ID_phases.csv", index_col=0)
            data = df_data.to_dict(orient='list')
            self.phases = data['phases']
            self.periods = data['periods']
            self.amplitudes = data['amplitudes']
        except:
            exit("Missing data")

