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


    def RK4_x(self, x, u, uh, h):
        k1 = h * self.xdot(x, u, uh)
        k2 = h * self.xdot(x + 0.5 * k1, u, uh)
        k3 = h * self.xdot(x + 0.5 * k2, u, uh)
        k4 = h * self.xdot(x + k3, u, uh)

        # Update next value of y
        x_new = x + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return x_new

    def RK4_Ph(self, x, xtilde):
        k1 = h * self.Phdot(x, xtilde)
        k2 = h * self.xdot(x + 0.5 * k1, u, uh)
        k3 = h * self.xdot(x + 0.5 * k2, u, uh)
        k4 = h * self.xdot(x + k3, u, uh)

        # Update next value of y
        x_new = x + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return x_new

    def xdot(self, x, u, uh):
         return np.matmul(self.A, x) + np.matmul(self.B, np.array([u+uh]))

    def Phdot(self, x, xtilde):
        return alpha * (xtilde - x) * x.transpose()

    def simulate(self, N, h, x0, u0, uh0, C, Qh0, Qhhat, R, GT):
        Q = C - Qh0

        P_LQ = cp.solve_continuous_are(A, B, Q, R)
        Ph_LQ = cp.solve_continuous_are(A, B, Qh0, R)
        Phhat = cp.solve_continuous_are(A, B, Qhhat, R)
        S = 1 / R * np.matmul(B, B.transpose())

        Lh0 = 1/R*np.matmul(B.transpose(), Ph_LQ)
        Lhhat0 = np.matmul(B.transpose(), Phhat)
        L0 = 1/R*np.matmul(B.transpose(), P_LQ)
        # Lhe = Lh[0,0]
        # Lhv = Lh[0,1]
        # print(Lh, Lhe, Lhv)

        # print(Lh, Lh_hat, L)

        x = np.zeros((2, N + 1))
        u = np.zeros(N)
        uh = np.zeros(N)
        x[:, 0] = x0
        u[0] = u0
        uh[0] = uh0

        L = np.zeros((N + 1, 2))
        Lh = np.zeros((N + 1, 2))
        Lhhat = np.zeros((N + 1, 2))
        Lh[0, :] = Lh0
        L[0, :] = L0
        Lhhat[0, :] = Lhhat0

        for i in range(N):
            # Acl = A - B * (Lh[i, :] + L[i, :])
            Acl = A - B * (Lh[i, :])
            Acl = A - B * (Lh[i, :])
            Aclh = A - B * (L[i, :])
            if GT == 1:
                Lh[i + 1, :] = 1 / R * np.matmul(B.transpose(), cp.solve_continuous_are(Aclh, B, Qh0, R))
                Ph_delta =
                Lhhat[i + 1, :] = Lhhat[i,:] + 1 / R * np.matmul(B.transpose(), Ph_delta)
                L[i + 1, :] = 1 / R * np.matmul(B.transpose(), cp.solve_continuous_are(Acl, B, Q, R))
            else:
                Lh[i + 1, :] = Lh[i, :]
                L[i + 1, :] = Lh[i, :]
            u[i] = np.matmul(-L[i + 1, :], x[:, i])
            uh[i] = np.matmul(-Lh[i + 1, :], x[:, i])
            x[:, i + 1] = dynamics_model.RK4(x[:, i], u[i], uh[i], h)

        return u, uh, x, Lh[-1, 0], Lh[-1, 1], Lh



# Dynamics
I = 6 #kg
D = -0.2 #N/m
# TODO: Note that xd is fixed! If xd_dot =/= 0 this does not work!
A = np.array([ [0, 1],[0, -D/I]])
B = np.array([[0], [1/I]])
Gamma = np.array([[100, 0], [0, 1]])
alpha = 1000

# Initial values
pos0 = 0
vel0 = 0
x_d = 0.1

# Simulation
time = 4
h = 0.05
N = round(time/h)
T = np.array(range(N)) * h

# Simulated Human Settings
# True cost values
Qh_e = 100
Qh_v = 0
Qh0 = np.array([[Qh_e, 0],[0, Qh_v]])

# Estimated cost value
Qh_e_hat = 100
Qh_v_hat = 4
Qhhat = np.array([[Qh_e_hat, 0],[0, Qh_v_hat]] )

# Robot cost values
Cval = 200
C = np.array( [[Cval, 0],[0, 0]] )
R = np.array([[1]])

# Initial values
x0 = np.array([pos0-x_d, vel0])
u0 = 0
uh0 = 0

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
u_LQ0, uh_LQ0, x_LQ0, Lhe_LQ0, Lhv_LQ0, Lh_tot_LQ = dynamics_model.simulate(N, h, x0, u0, uh0, C, Qh0, Qhhat, R, GT=0)
u_GT0, uh_GT0, x_GT0, Lhe_GT0, Lhv_GT0, Lh_tot_GT = dynamics_model.simulate(N, h, x0, u0, uh0, C, Qh0, Qhhat, R, GT=1)
labels_LQ = "LQ Qh = " + str(Qh_e)
labels_GT = "GT Qh = " + str(Qh_e)

ax1a.plot(T, x_LQ0[1, :-1], label=labels_LQ)
ax1a.plot(T, x_GT0[1, :-1],'--', label=labels_GT)
ax1a.set_title("Position error, C = " + str(Cval))
ax1a.legend()
ax2a.plot(T, x_LQ0[1, :-1], label=labels_LQ)
ax2a.plot(T, x_GT0[1, :-1],'--', label=labels_GT)
ax2a.set_title("Velocity error, C = " + str(Cval))
ax2a.legend()

ax1b.plot(T, u_LQ0, label=labels_LQ)
ax1b.plot(T, u_GT0,'--', label=labels_GT)
ax1b.set_title("Robot control action, C = " + str(Cval))
ax1b.legend()
ax2b.plot(T, uh_LQ0, label=labels_LQ)
ax2b.plot(T, uh_GT0,'--', label=labels_GT)
ax2b.set_title("Human control action, C = " + str(Cval))
ax2b.legend()

ax1c.plot(T, Lh_tot_LQ[:-1, 0], label=labels_LQ)
ax1c.plot(T, Lh_tot_GT[:-1, 0],'--', label=labels_GT)
ax1c.set_title("Gain on, C = " + str(Cval))
ax1c.legend()
ax2c.plot(T, Lh_tot_LQ[:-1, 1], label=labels_LQ)
ax2c.plot(T, Lh_tot_GT[:-1, 1],'--', label=labels_GT)
ax2c.set_title("Human control action, C = " + str(Cval))
ax2c.legend()


# Reproduce figure 1b
# Simulate multiple human costs
fractions = [0.1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
n = len(fractions)
QdivQh = np.zeros(n)
x_LQ = np.zeros((n, 2, N + 1))
u_LQ = np.zeros((n, N))
uh_LQ = np.zeros((n, N))

x_GT = np.zeros((n, 2, N + 1))
u_GT = np.zeros((n, N))
uh_GT = np.zeros((n, N))

Lhv_LQ = np.zeros(n)
Lhe_LQ = np.zeros(n)
Lhv_GT = np.zeros(n)
Lhe_GT = np.zeros(n)

for i in range(n):
    Qh_e = Cval*fractions[i]/(1+fractions[i])
    Qh0 = np.array([[Qh_e, 0],[0, Qh_v]])
    u_LQ[i, :], uh_LQ[i, :], x_LQ[i, : ,:], Lhe_LQ[i], Lhv_LQ[i], Lh_LQ = dynamics_model.simulate(N, h, x0, u0, uh0, C, Qh0, R, GT=0)
    u_GT[i, :], uh_GT[i, :], x_GT[i, :, :], Lhe_GT[i], Lhv_GT[i], Lh_GT = dynamics_model.simulate(N, h, x0, u0, uh0, C, Qh0, R, GT=1)

plt.figure()
plt.title("Controller gains")
plt.plot(fractions, Lhe_LQ, '+', label='LQ')
plt.plot(fractions, Lhe_GT, 'o', label='GT')
plt.legend()
plt.xlabel("Q_h/Q")
plt.ylabel("Lh_e")

plt.figure()
plt.title("Controller gains")
plt.plot(fractions, Lhv_LQ, '+', label='LQ')
plt.plot(fractions, Lhv_GT, 'o', label='GT')
plt.legend()
plt.xlabel("Q_h/Q")
plt.ylabel("Lh_v")

plt.show()


