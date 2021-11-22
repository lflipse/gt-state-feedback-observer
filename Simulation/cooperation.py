import numpy as np
import scipy.linalg as cp
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages

def save_all_figures():
    pp = PdfPages('cooperation.pdf')
    figs = None
    if figs is None:
        figs = [plt.figure(n) for n in plt.get_fignums()]
    for fig in figs:
        fig.savefig(pp, format='pdf')
    pp.close()

def solve_coupled_riccati(it, Qr, Qh, A, B):
    S = B * B.transpose()
    P_it = np.zeros((it + 1, 2, 2))
    Ph_it = np.zeros((it + 1, 2, 2))
    Pr0 = cp.solve_continuous_are(A, B, Qr, 1)
    Ph0 = cp.solve_continuous_are(A, B, Qh, 1)
    P_it[0, :, :] = Pr0
    Ph_it[0, :, :] = Ph0

    for i in range(it):
        Acl = A - np.matmul(S, (Ph_it[i, :, :]))
        Aclh = A - np.matmul(S, (P_it[i, :, :]))
        P_it[i + 1, :, :] = cp.solve_continuous_are(Acl, B, Qr, 1)
        Ph_it[i + 1, :, :] = cp.solve_continuous_are(Aclh, B, Qh, 1)

    Ph = Ph_it[it, :, :]
    Pr = P_it[it, :, :]
    Lr = np.matmul(B.transpose(), Pr)
    Lh = np.matmul(B.transpose(), Ph)
    Lr0 = np.matmul(B.transpose(), Pr0)
    Lh0 = np.matmul(B.transpose(), Ph0)
    return Lr, Lh, Lr0, Lh0


N = 50
C_1 = np.array([[50, 0], [0, 1]])
C_2 = np.array([[20, 0], [0, 0.5]])
C_3 = np.array([[5, 0], [0, 0.1]])
# Qr_1 = np.zeros((N, 2, 2))
# Qr_2 = Qr_1
# Qr_3 = Qr_1
# Qh_1 = Qr_1
# Qh_2 = Qr_1
# Qh_3 = Qr_1

Lr_1 = np.zeros((N, 2))
Lr_2 = np.zeros((N, 2))
Lr_3 = np.zeros((N, 2))
Lh_1 = np.zeros((N, 2))
Lh_2 = np.zeros((N, 2))
Lh_3 = np.zeros((N, 2))

Lr01 = np.zeros((N, 2))
Lr02 = np.zeros((N, 2))
Lr03 = np.zeros((N, 2))
Lh01 = np.zeros((N, 2))
Lh02 = np.zeros((N, 2))
Lh03 = np.zeros((N, 2))

qr1a = np.linspace(0, 50, N)
qr2a = np.linspace(0, 20, N)
qr3a = np.linspace(0, 5, N)
qr1b = np.linspace(0, 1, N)
qr2b = np.linspace(0, 0.5, N)
qr3b = np.linspace(0, 0.1, N)

# print(qr3a)

A = np.array([[0, 1], [0, -10]])
B = np.array([[0], [20]])

for i in range(N):
    Qr1 = np.array([[qr1a[i], 0], [0, qr1b[i]]])
    Qr2 = np.array([[qr2a[i], 0], [0, qr2b[i]]])
    Qr3 = np.array([[qr3a[i], 0], [0, qr3b[i]]])
    Qh1 = C_1 - Qr1
    Qh2 = C_2 - Qr2
    Qh3 = C_3 - Qr3
    Lr_1[i, :], Lh_1[i, :], Lr01[i, :], Lh01[i, :] = solve_coupled_riccati(10, Qr1, Qh1, A, B)
    Lr_2[i, :], Lh_2[i, :], Lr02[i, :], Lh02[i, :] = solve_coupled_riccati(10, Qr2, Qh2, A, B)
    Lr_3[i, :], Lh_3[i, :], Lr03[i, :], Lh03[i, :] = solve_coupled_riccati(10, Qr3, Qh3, A, B)
    # Qh_1[i, :, :] = Qh1
    # Qh_2[i, :, :] = Qh2
    # Qh_3[i, :, :] = Qh3

tud_blue = "#0066A2"
tud_blue2 = "#61A4B4"
tud_blue3 = "#007188"
tud_black = "#000000"
tud_grey = "#808080"
tud_red = "#c3312f"
tud_orange = "#EB7245"
tud_yellow = "#F1BE3E"
tud_green = "#00A390"
colors = [tud_blue, tud_red]

lw = 4
title_size = 24
label_size = 22
legend_size = 15

csfont = {'fontname': 'Georgia', 'size': title_size}
hfont = {'fontname': 'Georgia', 'size': label_size}

plt.figure()
plt.plot(Lh_1[:, 0], Lr_1[:, 0], tud_blue, label="C = diag(50, 1)", linewidth=lw)
plt.plot(Lh_2[:, 0], Lr_2[:, 0], tud_blue2, label="C = diag(20, 0.5)", linewidth=lw)
plt.plot(Lh_3[:, 0], Lr_3[:, 0], tud_blue3, label="C = diag(5, 0.1)", linewidth=lw)
plt.title("Differential Game Gain Distribution", **csfont)
plt.xlabel("Human Gain (Nm/rad)", **hfont)
plt.ylabel("Robot Gain (Nm/rad)", **hfont)
plt.xlim(0, 7.5)
plt.ylim(0, 7.5)
plt.legend(prop={"size": legend_size}, loc='upper right')
plt.tight_layout(pad=1)


plt.figure()
plt.plot(Lh_1[:, 0], Lr_1[:, 0], tud_blue2, label="Differential Game", linewidth=lw)
plt.plot(Lh01[:, 0], Lr01[:, 0], tud_orange, label="Optimal Control", linewidth=lw)
plt.xlabel("Human Gain (Nm/rad)", **hfont)
plt.ylabel("Robot Gain (Nm/rad)", **hfont)
plt.title("Differential Game vs. Optimal Control", **csfont)
plt.xlim(0, 7.5)
plt.ylim(0, 7.5)
plt.legend(prop={"size": legend_size}, loc='upper right')
plt.tight_layout(pad=1)


plt.figure()
plt.stackplot(Lh_1[:, 0], Lh_1[:, 0], Lr_1[:, 0], labels=["Human Gain", "Robot Gain"], colors=colors)
plt.xlabel("Human Gain (Nm/rad)", **hfont)
plt.ylabel("Total system gain (Nm/rad)", **hfont)
plt.title("Differential Game Gain Distribution", **csfont)
plt.xlim(0, 7.0)
# plt.ylim(0, 7.5)
plt.legend(prop={"size": legend_size}, loc='upper right')
plt.tight_layout(pad=1)


plt.figure()
plt.stackplot(Lh01[:, 0], Lh01[:, 0], Lr01[:, 0], labels=["Human Gain", "Robot Gain"], colors=colors)
plt.xlabel("Human Gain (Nm/rad)", **hfont)
plt.ylabel("Total system gain (Nm/rad)", **hfont)
plt.title("Optimal Control Distribution", **csfont)
plt.xlim(0, 7.2)
# plt.ylim(0, 7.5)
plt.legend(prop={"size": legend_size}, loc='upper right')
plt.tight_layout(pad=1)


save_all_figures()

plt.show()
