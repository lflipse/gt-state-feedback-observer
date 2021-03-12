# FILE TO TEST THE CONTROL PYTHON PACKAGE #

import numpy as np
import scipy.linalg as cp
import scipy.signal as sig
import matplotlib.pyplot as plt
import control


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
        x = y[0:2].transpose()
        x_hat = y[2:4].transpose()
        x_tilde = y[4:6].transpose()
        Ph_hat = np.array([y[6:8],y[8:10]])
        # print(Ph_hat)
        Lh_hat = 1/R * np.matmul(B.transpose(), Ph_hat)

        # Compute robot response
        Ar = self.A - self.B * Lh_hat
        P = cp.solve_continuous_are(Ar, B, Q, R)
        L = 1/R * np.matmul(self.B.transpose(), P)

        # Compute actual human response to robot
        Ah = self.A - self.B * L
        Ph = cp.solve_continuous_are(Ah, B, Qh, R)
        Lh = 1/R * np.matmul(self.B.transpose(), Ph)
        # print(L, Lh)

        # Compute derivatives
        x_dot = np.matmul((self.A - self.B * (L + Lh)), x)
        x_hat_dot = np.matmul(- self.B * (L + Lh_hat), x) + np.matmul(self.A, x_hat) - np.matmul(self.Gamma, x_tilde)
        x_tilde_dot = np.matmul(- self.B * (Lh_hat - Lh), x) + np.matmul((self.A - self.Gamma), x_tilde)
        x_trans = np.array([ [x[0]],[x[1]] ])
        # print(x_tilde - x)
        Ph_hat_dot = self.alpha * (x_tilde - x) * x_trans
        # print(Ph_hat_dot)
        y_dot = np.concatenate((x_dot, x_hat_dot, x_tilde_dot, Ph_hat_dot.flatten()), axis=None)
        # print(y_dot)
        return y_dot

    def simulate(self, N, h, y0, C, Qh0, Qhhat0, R):
        m = len(y0)
        y = np.zeros((m, N + 1))
        # print(y0)
        y[:, 0] = y0

        # Cost matrices
        Q = np.zeros((N, 2, 2))
        Qhhat = np.zeros((N + 1, 2, 2))
        Qhhat[0, :, :] = Qhhat0

        for i in range(N):
            Q[i, :, :] = C - Qhhat[i, :, :]
            y[:, i + 1] = self.RK4(y[:, i], R, Q[i, :, :], Qh0,  h)
            # print(self.B * y[6:8, i + 1])

            # Compute robot response
            # print(Q[i, :, :])
            Ph_hat = np.array([y[6:8, i + 1], y[8:10, i + 1]])
            print("Ph_hat", Ph_hat)
            Lh_hat = 1 / R * np.matmul(B.transpose(), Ph_hat)
            Ar = self.A - self.B * Lh_hat
            P = cp.solve_continuous_are(Ar, B, Q[i, :, :], R)
            L = 1 / R * self.B.transpose() * P
            print(Q[i, :, :])

            # Update Qhhat
            Ah = self.A - self.B * L
            print("Ah = ", Ah)
            wr, vr = np.linalg.eig(Ar)
            wh, vh = np.linalg.eig(Ah)
            print("eigenvalues Ar and Ah: ", wr, wh)
            print("testje", self.B * self.B.transpose())
            # print(P, Ah, Ah.transpose())
            print("linkboven iets? ",- np.matmul(Ah.transpose(), Ph_hat) - np.matmul(Ph_hat, Ah))
            print("en hier? ", 1/R * np.matmul(np.matmul(Ph_hat, self.B * self.B.transpose()), Ph_hat))
            Qhhat_t = 1/R * np.matmul(np.matmul(Ph_hat, self.B * self.B.transpose()), Ph_hat) - np.matmul(Ah.transpose(), Ph_hat) - np.matmul(Ph_hat, Ah)
            print("Qh_hat: ", Qhhat_t, "Qh_tilde = ", Qhhat_t - Qh0)
            # Qhhat[i + 1,:,:] = Qhhat_t
            Qhhat[i + 1, :, :] = np.array([[Qhhat_t[0, 0], 0], [0, Qhhat_t[1, 1]]])
            # print(Qhhat[i + 1, :, :])

        return y, Qhhat



# Dynamics
I = 6 #kg
D = -0.2 #N/m
# TODO: Note that xd is fixed! If xd_dot =/= 0 this does not work!
A = np.array([ [0, 1],[0, -D/I]])
B = np.array([[0], [1/I]])
Gamma = np.array([[10, 0], [0, 1]])
alpha = 10000

# Initial values
pos0 = 0
vel0 = 0
x_d = 0.1

# Simulation
time = 10
h = 0.05
N = round(time/h)
T = np.array(range(N)) * h

# Simulated Human Settings
# True cost values
Qh_e = 100
Qh_v = 0
Qh0 = np.array([[Qh_e, 0],[0, Qh_v]])

# Estimated cost value
Qh_e_hat = 10
Qh_v_hat = 1
Qhhat0 = np.array([[Qh_e_hat, 0],[0, Qh_v_hat]] )

# Robot cost values
Cval = 200
C = np.array( [[Cval, 0],[0, 0]] )
R = np.array([[1]])

# Initial values
x0 = np.array([pos0-x_d, vel0])
x_hat0 = x0
x_tilde0 = np.array([0, 0])
Lhhat0 = np.array([1, 1])
u0 = 0
uh0 = 0
# print(x0)
y0o = np.array([x0, x_hat0, x_tilde0, Lhhat0, Lhhat0])
y0 = y0o.flatten()
# print(y0)

# Initialize model
dynamics_model = DynamicsModel(A, B, I, D, Gamma, alpha)

figa, (ax1a, ax2a) = plt.subplots(2)
figa.suptitle('State values')
plt.xlabel("Time [s]")
plt.ylabel("Position, Velocity [m, m/s]")

figb, (ax1b, ax2b) = plt.subplots(2)
figb.suptitle('Input values')
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")

figc, (ax1c, ax2c) = plt.subplots(2)
figb.suptitle('Values of the gain matrix')
plt.xlabel("Time [s]")
plt.ylabel("Gain [-]")

# First plot a distinct GT vs LQ response
# Qh0 = np.array([[Qh_e, 0],[0, Qh_v]])
y, Qhhat = dynamics_model.simulate(N, h, y0, C, Qh0, Qhhat0, R)

labels_LQ = "LQ Qh = " + str(Qh_e)
labels_GT = "GT Qh = " + str(Qh_e)

ax1a.plot(T, y[0, :-1], label=labels_LQ)
# ax1a.plot(T, x_GT0[1, :-1],'--', label=labels_GT)
ax1a.set_title("Position error, C = " + str(Cval))
ax1a.legend()
ax2a.plot(T, y[1, :-1], label=labels_LQ)
# ax2a.plot(T, x_GT0[1, :-1],'--', label=labels_GT)
ax2a.set_title("Velocity error, C = " + str(Cval))
ax2a.legend()

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


