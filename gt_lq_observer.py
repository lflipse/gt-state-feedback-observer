# FILE TO TEST THE CONTROL PYTHON PACKAGE #

import numpy as np
import scipy.linalg as cp
import scipy.signal as sig
import matplotlib.pyplot as plt

class DynamicsModel:
    def __init__(self, A, B, I, D, Gamma, alpha):
        self.A = A
        self.B = B
        self.I = I
        self.D = D
        self.Gamma = Gamma
        self.alpha = alpha

    def RK4(self, y, R, Q, Qh, h):
        k1 = h * self.ydot(y, R, Q, Qh)
        k2 = h * self.ydot(y + 0.5 * k1, R, Q, Qh)
        k3 = h * self.ydot(y + 0.5 * k2, R, Q, Qh)
        k4 = h * self.ydot(y + k3, R, Q, Qh)

        # Update next value of y
        y_new = y + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return y_new

    def ydot(self, y, R, Q, Qh):
        # Define variables
        x = np.array([[y[0]], [y[1]]])
        x_hat = np.array([[y[2]], [y[3]]])
        x_tilde = np.array([[y[4]], [y[5]]])
        Ph_hat = np.array([y[6:8], y[8:10]])
        Lh_hat = 1/R * np.matmul(B.transpose(), Ph_hat)
        S = 1 / R * np.matmul(B, B.transpose())

        # Compute robot response
        Ar = self.A - self.B * Lh_hat
        P = cp.solve_continuous_are(Ar, B, Q, R)
        L = 1 / R * np.matmul(self.B.transpose(), P)

        # Compute actual human response to robot
        Ah = self.A - self.B * L
        Ph = cp.solve_continuous_are(Ah, B, Qh, R)
        Lh = 1 / R * np.matmul(self.B.transpose(), Ph)

        # Compute derivatives
        x_dot = np.matmul((self.A - self.B * (L + Lh)), x)
        x_hat_dot = np.matmul(- self.B * (L + Lh_hat), x) + np.matmul(self.A, x_hat) - np.matmul(self.Gamma, x_tilde)
        x_tilde_dot = np.matmul(- self.B * (Lh_hat - Lh), x) + np.matmul((self.A - self.Gamma), x_tilde)

        # Compute P matrix derivative
        Ph_hat_dot_p = self.alpha * (x_tilde - x) * x.transpose()
        Ph_hat_dot = Ph_hat_dot_p
        # Ph_hat_dot = (Ph_hat_dot_p + Ph_hat_dot_p.transpose()) / 2 # Make symmetric
        # Ph_hat_dot = np.array([[Ph_hat_dot_p[0,0], 0] ,[0, Ph_hat_dot_p[1,1]] ])
        y_dot = np.concatenate((x_dot, x_hat_dot, x_tilde_dot, Ph_hat_dot[0], Ph_hat_dot[1]), axis=None)
        return y_dot

    def simulate(self, N, x_d, h, y0, C, Qh0, Qhhat0, R):
        m = len(y0)
        y = np.zeros((m, N + 1))
        L = np.zeros((2, N + 1))
        Lh = np.zeros((2, N + 1))
        Lhhat = np.zeros((2, N + 1))
        y[:, 0] = y0

        # Cost matrices
        Q = np.zeros((N, 2, 2))
        Qh = np.zeros((N, 2, 2))
        Qhhat = np.zeros((N + 1, 2, 2))
        Qhhat[0, :, :] = Qhhat0

        for i in range(N):
            Q[i, :, :] = C - Qhhat[i, :, :]
            Qh[i, :, :] = Qh0
            y[:, i + 1] = self.RK4(y[:, i], R, Q[i, :, :], Qh0,  h)

            # Update Qhhat
            Ph_hat = np.array([y[6:8, i + 1], y[8:10, i + 1]])
            Lhhat[:, i] = 1 / R * np.matmul(self.B.transpose(), Ph_hat)

            # Robot response
            Ar = self.A - self.B * Lhhat[:, i]
            P = cp.solve_continuous_are(Ar, B, Q[i, :, :], R)
            L[:, i] = 1 / R * np.matmul(self.B.transpose(), P)

            # Actual human response
            Ah = self.A - self.B * L[:, i]
            Ph = cp.solve_continuous_are(Ah, B, Qh0, R)
            Lh[:, i] = 1 / R * np.matmul(self.B.transpose(), Ph)

            eigsAr, eigsvalc = np.linalg.eig(Ar)
            eigsAh, eigsvalh = np.linalg.eig(Ah)
            # print("eigenvalues Ar = ", eigsAr)
            # print("eigenvalues Ah = ", eigsAh)

            # print('ph, phhat', Ph, Ph_hat)

            Qhhat_t = 1/R * np.matmul(np.matmul(Ph_hat, self.B * self.B.transpose()), Ph_hat) - np.matmul(Ah.transpose(), Ph_hat) - np.matmul(Ph_hat, Ah)
            Qhhat[i + 1, :, :] = np.array([[Qhhat_t[0,0], 0],[0, Qhhat_t[1, 1]]])
            # Qhhat[i + 1, :, :] = (Qhhat_t + Qhhat_t.transpose()) / 2
            # print(Qhhat[i, :, :], Qh0)
            # exit()

        return y, Qh, Qhhat, Q, L, Lh, Lhhat

# Dynamics
I = 6 #kg
D = -0.2 #N/m
# TODO: Note that xd is fixed! If xd_dot =/= 0 this does not work!
A = np.array([[0, 1], [0, -D/I]])
B = np.array([[0], [1/I]])
Gamma = np.array([[10, 0], [0, 1]])
alpha = 1000

# Initial values
pos0 = 0
vel0 = 0
x_d = 0.1

# Simulation
time = 5
h = 0.01
N = round(time/h)
T = np.array(range(N)) * h

# Simulated Human Settings
# True cost values
Qh_e = 100
Qh_v = 0
Qh0 = np.array([[Qh_e, 0], [0, Qh_v]])

# Estimated cost value
Qh_e_hat = 10
Qh_v_hat = 1
Qhhat0 = np.array([[Qh_e_hat, 0], [0, Qh_v_hat]] )

# Robot cost values
Cval = 200
C = np.array([[Cval, 0], [0, 0]])
R = np.array([[1]])

# Initial values
x0 = np.array([pos0-x_d, vel0])
x_hat0 = x0
x_tilde0 = np.array([0, 0])
Lhhat0 = np.array([0, 0])
Phhat0 = np.array([[1, 0], [0, 0.1]])
Phhat0 = cp.solve_continuous_are(A, B, Qhhat0, R)
# print(Phhat0)
# print(Phhat0[0])
u0 = 0
uh0 = 0
# print(x0)
y0o = np.array([x0, x_hat0, x_tilde0, Phhat0[0], Phhat0[1]])
y0 = y0o.flatten()
# print(y0o, y0)

# Initialize model
dynamics_model = DynamicsModel(A, B, I, D, Gamma, alpha)

figa, (ax1a, ax2a) = plt.subplots(2)
figa.suptitle('State values')
plt.xlabel("Time [s]")
# plt.ylabel("Position, Velocity [m, m/s]")

figb, (ax1b, ax2b) = plt.subplots(2)
figb.suptitle('Values of the gain vectors')
plt.xlabel("Time [s]")
# plt.ylabel("Force [N]")

figc, (ax1c, ax2c) = plt.subplots(2)
figb.suptitle('Values of the cost matrix')
plt.xlabel("Time [s]")
# plt.ylabel("Gain [-]")

# First plot a distinct GT vs LQ response
# Qh0 = np.array([[Qh_e, 0],[0, Qh_v]])
y, Qh, Qhhat, Q, L, Lh, Lhhat = dynamics_model.simulate(N, x_d, h, y0, C, Qh0, Qhhat0, R)



labels_LQ = "LQ Qh = " + str(Qh_e)
labels_GT = "GT Qh = " + str(Qh_e)
labels_Lh = "Gain for actual human"
labels_Lhhat = "Gain for estimated human"
labels_L = "Gain for robot"
labels_Q = "cost weight robot"
labels_Qhhat = "estimated cost weight human"
labels_Qh = "actual cost weight human"

ax1a.plot(T, y[0, :-1], label=labels_LQ)
# ax1a.plot(T, x_GT0[1, :-1],'--', label=labels_GT)
ax1a.set_title("Position error, C = " + str(Cval))
ax1a.legend()
ax2a.plot(T, y[1, :-1], label=labels_LQ)
# ax2a.plot(T, x_GT0[1, :-1],'--', label=labels_GT)
ax2a.set_title("Velocity error, C = " + str(Cval))
ax2a.legend()

ax1b.plot(T, Lh[0, :-1],  'r-.', label=labels_Lh)
ax1b.plot(T, L[0, :-1], 'b', label=labels_L)
ax1b.plot(T, Lhhat[0, :-1],'g--', label=labels_Lhhat)
ax1b.set_title("Error gain, C = " + str(Cval))
ax1b.legend()
ax2b.plot(T, Lh[1, :-1], 'r-.', label=labels_Lh)
ax2b.plot(T, L[1, :-1], 'b', label=labels_L)
ax2b.plot(T, Lhhat[1, :-1],'g--', label=labels_Lhhat)
ax2b.set_title("Velocity gain, C = " + str(Cval))
ax2b.legend()

ax1c.plot(T, Q[:, 0, 0], 'b', label=labels_Q)
ax1c.plot(T, Qhhat[:-1, 0, 0],'g--', label=labels_Qhhat)
ax1c.plot(T, Qh[:, 0, 0],'r-.', label=labels_Qh)
ax1c.set_title("Error weight, C = " + str(Cval))
ax1c.legend()
ax2c.plot(T, Q[:, 1, 1], 'b', label=labels_Q)
ax2c.plot(T, Qhhat[:-1, 1, 1],'g--', label=labels_Qhhat)
ax2c.plot(T, Qh[:, 1, 1],'r-.', label=labels_Qh)
ax2c.set_title("Velocity weight, C " + str(Cval))
ax2c.legend()

# ax1b.plot(T, u_LQ0, label=labels_LQ)
# ax1b.plot(T, u_GT0,'--', label=labels_GT)
# ax1b.set_title("Robot control action, C = " + str(Cval))
# ax1b.legend()
# ax2b.plot(T, uh_LQ0, label=labels_LQ)
# ax2b.plot(T, uh_GT0,'--', label=labels_GT)
# ax2b.set_title("Human control action, C = " + str(Cval))
# ax2b.legend()
#
# ax1c.plot(T, Lh_tot_LQ[:-1, 0], label=labels_LQ)
# ax1c.plot(T, Lh_tot_GT[:-1, 0],'--', label=labels_GT)
# ax1c.set_title("Gain on, C = " + str(Cval))
# ax1c.legend()
# ax2c.plot(T, Lh_tot_LQ[:-1, 1], label=labels_LQ)
# ax2c.plot(T, Lh_tot_GT[:-1, 1],'--', label=labels_GT)
# ax2c.set_title("Human control action, C = " + str(Cval))
# ax2c.legend()
#
#
# # Reproduce figure 1b
# # Simulate multiple human costs
# fractions = [0.1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
# n = len(fractions)
# QdivQh = np.zeros(n)
# x_LQ = np.zeros((n, 2, N + 1))
# u_LQ = np.zeros((n, N))
# uh_LQ = np.zeros((n, N))
#
# x_GT = np.zeros((n, 2, N + 1))
# u_GT = np.zeros((n, N))
# uh_GT = np.zeros((n, N))
#
# Lhv_LQ = np.zeros(n)
# Lhe_LQ = np.zeros(n)
# Lhv_GT = np.zeros(n)
# Lhe_GT = np.zeros(n)
#
# for i in range(n):
#     Qh_e = Cval*fractions[i]/(1+fractions[i])
#     Qh0 = np.array([[Qh_e, 0],[0, Qh_v]])
#     u_LQ[i, :], uh_LQ[i, :], x_LQ[i, : ,:], Lhe_LQ[i], Lhv_LQ[i], Lh_LQ = dynamics_model.simulate(N, h, x0, u0, uh0, C, Qh0, R, GT=0)
#     u_GT[i, :], uh_GT[i, :], x_GT[i, :, :], Lhe_GT[i], Lhv_GT[i], Lh_GT = dynamics_model.simulate(N, h, x0, u0, uh0, C, Qh0, R, GT=1)
#
# plt.figure()
# plt.title("Controller gains")
# plt.plot(fractions, Lhe_LQ, '+', label='LQ')
# plt.plot(fractions, Lhe_GT, 'o', label='GT')
# plt.legend()
# plt.xlabel("Q_h/Q")
# plt.ylabel("Lh_e")
#
# plt.figure()
# plt.title("Controller gains")
# plt.plot(fractions, Lhv_LQ, '+', label='LQ')
# plt.plot(fractions, Lhv_GT, 'o', label='GT')
# plt.legend()
# plt.xlabel("Q_h/Q")
# plt.ylabel("Lh_v")

plt.show()


