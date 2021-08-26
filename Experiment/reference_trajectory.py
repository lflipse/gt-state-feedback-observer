import numpy as np
import math
import matplotlib.pyplot as plt

class Reference:
    def __init__(self, duration):
        bw = 6
        period = np.array([5, 8, 11, 17, 29, 43, 57, 73, 97, 121])
        self.duration = (period[7] * 2 * np.pi) / bw
        frequencies = 2 * np.pi * period / self.duration
        phases = [1.79893804, 4.3567642, -2.362783088, 3.58145111, -1.92977185, -1.1689079, -0.61034581,
                  -1.75180265, -2.03232366, 3.2509144]
        print("duration = ", self.duration)
        print("frequencies = ", frequencies)
        self.amp = 0.3
        amplitude = self.amp * np.array([1, 1, 1, 1, 1, 1, 1, 0.1, 0.1, 0.1])
        self.forcing_function = {
            'period': period,
            'phases': phases,
            'amplitude': amplitude,
        }

        csfont = {'fontname': 'Georgia'}
        hfont = {'fontname': 'Georgia'}

        # Colors
        tud_blue = "#0066A2"
        tud_black = "#000000"
        tud_grey = "#808080"
        tud_red = "#c3312f"
        tud_green = "#00A390"
        tud_yellow = "#F1BE3E"

        plt.figure()
        for i in range(len(period)):
            plt.plot(frequencies[i], amplitude[i], color=tud_blue, marker='o')
            plt.plot([frequencies[i], frequencies[i]], [0, amplitude[i]], tud_blue, alpha=0.7, linewidth=2.5)

        plt.title("Reference trajectory in frequency domain", **csfont)
        plt.xlabel("Frequency (rad/s)", **hfont)
        plt.ylabel("Amplitude (-)", **hfont)
        plt.xlim(frequencies[0] - 0.1, frequencies[-1] + 10)
        plt.ylim(0.01, amplitude[0] + 0.1)
        plt.yscale("log")
        plt.xscale("log")

        # Show forcing function:
        fs = 100
        n = int(fs * self.duration)
        t = np.array(range(n)) / fs
        r = np.zeros(n)
        for i in range(n):
            ref = self.generate_reference(t[i]+5, 0, "else", 0)
            r[i] = ref[0]

        plt.figure()
        plt.plot(t, r, tud_blue, linewidth=2.5)
        plt.title("Reference trajectory in time domain", **csfont)
        plt.xlabel("Time (s)", **hfont)
        plt.ylabel("Amplitude (-)", **hfont)
        plt.xlim(0, 10)


    def generate_reference(self, t, sigma, player, ref_sign):
        period = self.forcing_function["period"]
        phases = self.forcing_function["phases"]
        amplitude = self.forcing_function["amplitude"]
        reference_position = 0
        reference_position_noise = 0
        reference_velocity = 0

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

        for i in range(10):
            wt = (period[i] / self.duration) * (2 * np.pi)
            reference_position += mag * amplitude[i] * math.sin(fac * wt * t + phases[i])
            reference_position_noise += sigma * 1 * amplitude[i] * wt * math.sin(-0.7 * wt * t + phases[i] + 0.1)
            reference_velocity += fac * mag * amplitude[i] * wt * math.cos(fac * wt * t + phases[i])


        if player == "robot":
            ref = np.array([reference_position, reference_velocity])
        else:
            ref = np.array([reference_position + v, w])

        return ref
