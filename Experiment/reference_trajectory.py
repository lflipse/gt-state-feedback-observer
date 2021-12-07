import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib.backends.backend_pdf import PdfPages

class Reference:
    def __init__(self, duration):
        bw = 3
        self.amp = 0.25
        self.load_data()
        if len(self.periods) == 15:
            self.duration = (self.periods[10] * 2 * np.pi) / bw
        else:
            self.duration = (self.periods[7] * 2 * np.pi) / bw
        frequencies = 2 * np.pi * self.periods / self.duration
        print("duration should be: ", self.duration)
        print("frequencies = ", frequencies)
        self.forcing_function = {
            'periods': self.periods,
            'phases': self.phases,
            'amplitudes': self.amp * self.amplitudes,
        }

        print(self.forcing_function)

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

        plt.title("Frequency domain response", **self.csfont)
        plt.xlabel("Frequency ($rad/s$)", **self.hfont)
        plt.ylabel("Amplitude ($rad$)", **self.hfont)
        plt.xlim(frequencies[0] - 0.02, frequencies[-1] + 1)
        plt.ylim(0.1, self.amplitudes[0] + 0.1)
        plt.yscale("log")
        plt.xscale("log")
        plt.tight_layout(pad=1)

        # Show forcing function:
        fs = 100
        n = int(fs * self.duration)
        t = np.array(range(n)) / fs
        r = np.zeros(n)
        for i in range(n):
            ref = self.generate_reference(t[i], sigma=0, player=None, ref_sign=1)
            r[i] = ref[0]

        plt.figure()
        plt.plot(t, r, tud_blue, linewidth=self.lw)
        plt.title("Time domain signal", **self.csfont)
        plt.xlabel("Time ($s$)", **self.hfont)
        plt.ylabel("Amplitude ($rad$)", **self.hfont)
        plt.xlim(0, 15)
        plt.tight_layout(pad=1)
        self.save_all_figures()

    def load_data(self):
        try:
            df_data = pd.read_csv("..\\Steering_Wheel_Dynamics\\Experiment_phases.csv", index_col=0)
            data = df_data.to_dict(orient='list')
            self.phases = np.array(data['phases'])
            self.periods = np.array(data['periods'])
            self.amplitudes = np.array(data['amplitudes'])
        except:
            exit("Missing data")

    def save_all_figures(self):
        pp = PdfPages('..\\Experiment\\figures\\reference.pdf')
        figs = None
        if figs is None:
            figs = [plt.figure(n) for n in plt.get_fignums()]
        for fig in figs:
            fig.savefig(pp, format='pdf')
        pp.close()

    def generate_reference(self, t, sigma, player, ref_sign):
        period = self.forcing_function["periods"]
        phases = self.forcing_function["phases"]
        amplitude = 1.5*self.forcing_function["amplitudes"]
        ref_position = 0
        ref_velocity = 0

        v = np.random.normal(0, sigma)
        w = np.random.normal(0, 1.5*sigma)

        if ref_sign == 0:
            mag = 1
            fac = 1
        elif ref_sign == 1:
            mag = 1
            fac = -1
        elif ref_sign == 2:
            mag = -1
            fac = 1
        elif ref_sign == 3:
            mag = -1
            fac = -1
        else:
            exit("error in reference signal")

        for i in range(len(period)):
            wt = (period[i] / self.duration) * (2 * np.pi)
            ref_position += mag * amplitude[i] * math.sin(fac * wt * t + phases[i])
            ref_velocity += fac * mag * amplitude[i] * wt * math.cos(fac * wt * t + phases[i])

        reference_position = ref_position * math.tanh(0.5 * t)
        reference_velocity = ref_velocity * math.tanh(0.5 * t)

        if player == "robot":
            ref = np.array([reference_position, reference_velocity])
        else:
            ref = np.array([reference_position + v, w])

        return ref

# ref = Reference(1)
