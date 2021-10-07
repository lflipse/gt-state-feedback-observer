# generate random Gaussian values
from numpy.random import seed
from numpy.random import randn
import numpy as np
import matplotlib.pyplot as plt
import math
import pandas as pd
# seed random number generator
seed(1)


def to_csv(phases, periods, amplitudes, file):
    to_be_saved = {
        'phases': phases,
        'periods': periods,
        'amplitudes': amplitudes,
    }
    df = pd.DataFrame(data=to_be_saved)
    df.to_csv(file)

def generate_reference(amplitudes, frequencies, phases, t):
    fs = 0
    for i in range(len(amplitudes)):
        fs += amplitudes[i] * math.sin(2 * math.pi * frequencies[i] * t + phases[i])
    return fs

def low_pass_filter(bandwidth, periods):
    amplitudes = []
    if len(periods) == 15:
        duration = (periods[10] * 2 * np.pi) / bandwidth
    elif len(periods) == 10:
        duration = (periods[7] * 2 * np.pi) / bandwidth
    frequencies = 2 * np.pi * periods / duration
    for i in range(len(periods)):
        mag = 1 / np.sqrt(1 + (frequencies[i]/bandwidth)**2)
        amplitudes.append(mag)

    return amplitudes

def crest_phases(n, bandwidth, periods, amplitudes):
    # pre-allocate vectors
    phases = np.zeros((n, len(periods)))
    CF = np.zeros(n)
    if len(periods) == 15:
        duration = (periods[10] * 2 * np.pi) / bandwidth  # Run for 2 times the
    elif len(periods) == 10:
        duration = (periods[7] * 2 * np.pi) / bandwidth  # Run for 2 times the
    print(duration)
    frequencies = 2 * np.pi * periods / duration

    # Loop over n instances
    for i in range(n):
        phases[i, :] = randn(len(periods))

        # Compute crest factor
        fs = 30
        m = int(fs * duration)
        t = np.array(range(m)) / fs
        r = np.zeros(m)
        for j in range(m):
            r[j] = generate_reference(amplitudes, frequencies, phases[i, :], t[j])

        r_abs = np.abs(r)
        f_max = np.max(r_abs)
        sigma = np.var(r)

        CF[i] = f_max / np.sqrt(sigma)

    # Take the two best phase sets
    ind1 = np.argmin(CF)
    np.delete(CF, ind1)
    ind2 = np.argmin(CF)
    indmax = np.argmax(CF)
    phase1 = phases[ind1, :]
    phase2 = phases[ind2, :]
    phasemax = phases[indmax, :]

    return phase1, phase2, phasemax

def show_forcing_function_freq(bandwidth, periods, amplitudes):
    lw = 4
    title_size = 24
    label_size = 22
    legend_size = 15

    lw_box = 2.5
    boxw = 0.5

    # self.colors
    tud_blue = "#0066A2"
    tud_blue2 = "#61A4B4"
    tud_blue3 = "#007188"
    tud_black = "#000000"
    tud_grey = "#808080"
    tud_red = "#c3312f"
    tud_orange = "#EB7245"
    tud_yellow = "#F1BE3E"
    tud_green = "#00A390"

    csfont = {'fontname': 'Georgia', 'size': title_size}
    hfont = {'fontname': 'Georgia', 'size': label_size}

    if len(periods) == 15:
        duration = (periods[10] * 2 * np.pi) / bandwidth  # Run for 2 times the
    elif len(periods) == 10:
        duration = (periods[7] * 2 * np.pi) / bandwidth  # Run for 2 times the
    frequencies = 2 * np.pi * periods / duration

    plt.figure()
    for i in range(len(periods)):
        plt.plot(frequencies[i], amplitudes[i], color=tud_blue, linewidth=lw, marker='o')
        plt.plot([frequencies[i], frequencies[i]], [0, amplitudes[i]], tud_blue, linewidth=lw, alpha=0.7)

    plt.title("Frequency domain reference", **csfont)
    plt.xlabel("Frequency (rad/s)", **hfont)
    plt.ylabel("Amplitude (-)", **hfont)
    plt.xlim(0.5 * frequencies[0], 2 * frequencies[-1])
    plt.ylim(amplitudes[-1] * 0.5, amplitudes[0] * 1.2)
    plt.yscale("log")
    plt.xscale("log")
    plt.tight_layout(pad=1)

def show_forcing_function(bandwidth, periods, amplitudes, phases):
    lw = 4
    title_size = 24
    label_size = 22
    legend_size = 15

    lw_box = 2.5
    boxw = 0.5

    # self.colors
    tud_blue = "#0066A2"
    tud_blue2 = "#61A4B4"
    tud_blue3 = "#007188"
    tud_black = "#000000"
    tud_grey = "#808080"
    tud_red = "#c3312f"
    tud_orange = "#EB7245"
    tud_yellow = "#F1BE3E"
    tud_green = "#00A390"

    csfont = {'fontname': 'Georgia', 'size': title_size}
    hfont = {'fontname': 'Georgia', 'size': label_size}

    if len(periods) == 15:
        duration = (periods[10] * 2 * np.pi) / bandwidth  # Run for 2 times the
    elif len(periods) == 10:
        duration = (periods[7] * 2 * np.pi) / bandwidth
    frequencies = 2 * np.pi * periods / duration

    print(duration)

    # Show forcing function:
    fs = 100
    n = int(fs * duration)
    t = np.array(range(n)) / fs
    r = np.zeros(n)
    for i in range(n):
        r[i] = generate_reference(amplitudes, frequencies, phases, t[i])

    plt.figure()
    plt.plot(t, r, tud_blue, linewidth=lw)
    plt.title("Time domain reference", **csfont)
    plt.xlabel("Time (s)", **hfont)
    plt.ylabel("Amplitude (-)", **hfont)
    plt.xlim(0, duration)
    plt.tight_layout(pad=1)

# Two reference trajectories are to be chosen

# Example figure
bw = 1
bw2 = 3
periods = np.array([2, 3, 5, 7, 13, 19, 29, 37, 47, 61, 79, 101, 127, 163, 211])
periods2 = np.array([2, 3, 5, 7, 13, 19, 29, 37, 47, 61])
amplitudes = low_pass_filter(bw, periods)
amplitudes2 = low_pass_filter(bw2, periods2)

# amplitudes fixen als een filter
# amplitudes = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0.707, 0.1, 0.1, 0.1, 0.1])

n = 1000

phase1b, phase2b, phasemax2 = crest_phases(n, bw2, periods2, amplitudes2)
# phase1, phase2, phasemax = crest_phases(n, bw, periods, amplitudes)


# to_csv(phase1, periods, amplitudes, "ID_phases.csv")
# to_csv(phase2, periods, amplitudes, "Validation_phases.csv")
to_csv(phase2b, periods2, amplitudes2, "Experiment_phases.csv")

# show_forcing_function_freq(bw, periods, amplitudes)
show_forcing_function_freq(bw2, periods2, amplitudes2)
show_forcing_function(bw2, periods2, amplitudes2, phase1b)
show_forcing_function(bw2, periods2, amplitudes2, phasemax2)
plt.show()