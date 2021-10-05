import scipy.linalg as cp
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


def save_all_figures():
    pp = PdfPages('strategy.pdf')
    figs = None
    if figs is None:
        figs = [plt.figure(n) for n in plt.get_fignums()]
    for fig in figs:
        fig.savefig(pp, format='pdf')
    pp.close()

def compute_gains(Qr, Qh, A, B):
    # Iterative procedure for calculating gains
    Lh = np.matmul(B.transpose(), cp.solve_continuous_are(A, B, Qh, 1))
    Lr = np.matmul(B.transpose(), cp.solve_continuous_are(A, B, Qr, 1))
    for i in range(10):
        Lh = np.matmul(B.transpose(), cp.solve_continuous_are(A - B * Lr, B, Qh, 1))
        Lr = np.matmul(B.transpose(), cp.solve_continuous_are(A - B * Lh, B, Qr, 1))
    C = (Lr[0, 0] - Lh[0, 0]) / (Lr[0, 0] + Lh[0, 0])
    return C, Lr, Lh

# Take 100 steps of possible gains and costs
n = 200  # steps
m = 3  # strategies

csfont = {'fontname': 'Georgia', 'size': 24}
hfont = {'fontname': 'Georgia', 'size': 22}
linewidth = 4

# Colors
tud_black = "#000000"
tud_blue = "#0066A2"
tud_red = "#c3312f"
tud_green = "#00A390"
tud_yellow = "#F1BE3E"
tud_orange = "#EB7245"
tud_lightblue = "#00B7D3"

# Compute possible costs from -100 to 100
q1h = np.linspace(0, 100, num=n)
Qh = np.zeros((n, 2, 2))
Qr1 = np.zeros((n, 2, 2))
Qr2 = np.zeros((n, 2, 2))
Qr3 = np.zeros((n, 2, 2))
Q0 = np.array([[0, 0], [0, 0]])
Lh1 = np.zeros((n, 2))
Lr1 = np.zeros((n, 2))
Lh2 = np.zeros((n, 2))
Lr2 = np.zeros((n, 2))
Lh3 = np.zeros((n, 2))
Lr3 = np.zeros((n, 2))
C1 = np.zeros(n)
C2 = np.zeros(n)
C3 = np.zeros(n)
Qh[:, 0, 0] = q1h

C = np.array([[12.5, 0], [0, 0.1]])

# Dynamics
Jw = 0.04914830792783059
Bw = 0.3  # Max = 0.5
Kw = 0.0  # Max = 2.5
A = np.array([[0, 1], [- Kw / Jw, - Bw / Jw]])
B = np.array([[0], [1 / Jw]])

for i in range(n):
    if Qh[i, 0, 0] < 1 * C[0, 0]:
        Qr1[i, :, :] = 0.5 * C - 0.5 * Qh[i, :, :]
    Qr2[i, :, :] = 0.5 * C
    Qr3[i, :, :] = 0.5 * C + 0.8 * np.abs(Qh[i, :, :])

    C1[i], Lr1[i, :], Lh1[i, :] = compute_gains(Qr1[i, :, :], Qh[i, :, :], A, B)
    C2[i], Lr2[i, :], Lh2[i, :] = compute_gains(Qr2[i, :, :], Qh[i, :, :], A, B)
    C3[i], Lr3[i, :], Lh3[i, :] = compute_gains(Qr3[i, :, :], Qh[i, :, :], A, B)

plt.figure()
plt.plot(Qh[:, 0, 0], Qr1[:, 0, 0], linewidth=linewidth)
plt.title("Negative reinforcement", csfont)
plt.xlabel('Human cost weight (-)', hfont)
plt.ylabel('Robot cost weight (-)', hfont)
# plt.legend(prop={"size": 14}, loc='upper right')
plt.xlim(Qh[0, 0, 0], Qh[-1, 0, 0])
plt.tight_layout(pad=1)

plt.figure()
plt.plot(Qh[:, 0, 0], Qr2[:, 0, 0], linewidth=linewidth)
plt.title("No reinforcement", csfont)
plt.xlabel('Human cost weight (-)', hfont)
plt.ylabel('Robot cost weight (-)', hfont)
# plt.legend(prop={"size": 14}, loc='upper right')
plt.xlim(Qh[0, 0, 0], Qh[-1, 0, 0])
plt.tight_layout(pad=1)

plt.figure()
plt.plot(Qh[:, 0, 0], Qr3[:, 0, 0], linewidth=linewidth)
plt.title("Positive reinforcement", csfont)
plt.xlabel('Human cost weight (-)', hfont)
plt.ylabel('Robot cost weight (-)', hfont)
# plt.legend(prop={"size": 14}, loc='upper right')
plt.xlim(Qh[0, 0, 0], Qh[-1, 0, 0])
plt.tight_layout(pad=1)

plt.figure()
plt.plot(Lh1[:, 0], Lr1[:, 0], linewidth=linewidth)
plt.title("Negative reinforcement", csfont)
plt.xlabel('Human gain (Nm)', hfont)
plt.ylabel('Robot gain (Nm)', hfont)
# plt.legend(prop={"size": 14}, loc='upper right')
plt.xlim(Lh1[0, 0], Lh1[-1, 0])
plt.tight_layout(pad=1)

plt.figure()
plt.plot(Lh2[:, 0], Lr2[:, 0], linewidth=linewidth)
plt.title("No reinforcement", csfont)
plt.xlabel('Human gain (Nm)', hfont)
plt.ylabel('Robot gain (Nm)', hfont)
# plt.legend(prop={"size": 14}, loc='upper right')
plt.xlim(Lh2[0, 0], Lh2[-1, 0])
plt.tight_layout(pad=1)

plt.figure()
plt.plot(Lh3[:, 0], Lr3[:, 0], linewidth=linewidth)
plt.title("Positive reinforcement", csfont)
plt.xlabel('Human gain (Nm)', hfont)
plt.ylabel('Robot gain (Nm)', hfont)
# plt.legend(prop={"size": 14}, loc='upper right')
plt.xlim(Lh3[0, 0], Lh3[-1, 0])
plt.tight_layout(pad=1)

plt.figure()
plt.plot(Qh[:, 0, 0], Qr1[:, 0, 0], tud_blue, linewidth=linewidth, label="Negative Reinforcement")
plt.plot(Qh[:, 0, 0], Qr2[:, 0, 0], tud_red, linewidth=linewidth, label="Constant Interaction Strategy")
plt.plot(Qh[:, 0, 0], Qr3[:, 0, 0], tud_green, linewidth=linewidth, label="Positive Reinforcement")
plt.title("Interaction strategies: Costs", csfont)
plt.xlabel('Human cost weight (-)', hfont)
plt.ylabel('Robot cost weight (-)', hfont)
plt.legend(prop={"size": 14}, loc='upper right')
xmin = min(Qh[:, 0, 0])
xmax = max(Qh[:, 0, 0])
ymin = min(min(Qr1[:, 0, 0]), min(Qr2[:, 0, 0]), min(Qr3[:, 0, 0])) - 0.1
ymax = max(max(Qr1[:, 0, 0]), max(Qr2[:, 0, 0]), max(Qr3[:, 0, 0])) + 0.1
plt.xlim(xmin, xmax)
plt.ylim(ymin, ymax)
plt.tight_layout(pad=1)

plt.figure()
plt.plot(Lh1[:, 0], Lr1[:, 0], tud_blue, linewidth=linewidth, label="Negative Reinforcement")
plt.plot(Lh2[:, 0], Lr2[:, 0], tud_red, linewidth=linewidth, label="Constant Interaction Strategy")
plt.plot(Lh3[:, 0], Lr3[:, 0], tud_green, linewidth=linewidth, label="Positive Reinforcement")
plt.title("Interaction strategies: Gains", csfont)
plt.xlabel('Human gain (Nm)', hfont)
plt.ylabel('Robot gain (Nm)', hfont)
plt.legend(prop={"size": 14}, loc='upper right')
xmin = min(min(Lh1[:, 0]), min(Lh2[:, 0]), min(Lh3[:, 0]))
xmax = min(max(Lh1[:, 0]), max(Lh2[:, 0]), max(Lh3[:, 0]))
ymin = min(min(Lr1[:, 0]), min(Lr2[:, 0]), min(Lr3[:, 0])) - 0.1
ymax = max(max(Lr1[:, 0]), max(Lr2[:, 0]), max(Lr3[:, 0])) + 0.1
plt.xlim(xmin, xmax)
plt.ylim(ymin, ymax)
plt.tight_layout(pad=1)

plt.figure()
plt.plot(Lh1[:, 0], C1, tud_blue, linewidth=linewidth, label="Negative Reinforcement")
plt.plot(Lh2[:, 0], C2, tud_red, linewidth=linewidth, label="Constant Interaction Strategy")
plt.plot(Lh3[:, 0], C3, tud_green, linewidth=linewidth, label="Positive Reinforcement")
plt.title("Interaction strategies: Authority", csfont)
plt.xlabel('Human gain (Nm)', hfont)
plt.ylabel('Robot gain (Nm)', hfont)
plt.legend(prop={"size": 14}, loc='upper right')
xmin = min(min(Lh1[:, 0]), min(Lh2[:, 0]), min(Lh3[:, 0]))
xmax = min(max(Lh1[:, 0]), max(Lh2[:, 0]), max(Lh3[:, 0]))
ymin = min(min(C1), min(C2), min(C3)) - 0.1
ymax = max(max(C1), max(C2), max(C3)) + 0.1
plt.xlim(xmin, xmax)
plt.ylim(ymin, ymax)
plt.tight_layout(pad=1)

save_all_figures()

plt.show()