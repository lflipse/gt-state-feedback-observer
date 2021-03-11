# FILE TO TEST THE CONTROL PYTHON PACKAGE #

import numpy as np
import scipy.linalg as cp
import scipy.signal as sig
import matplotlib.pyplot as plt
import control


class DynamicsModel:
    def __init__(self, A, B, I, D):
        self.A = A
        self.B = B
        self.I = I
        self.D = D

    def RK4(self, x, u, uh, h):
        k1 = h * self.xdot(x, u, uh)
        k2 = h * self.xdot(x + 0.5 * k1, u, uh)
        k3 = h * self.xdot(x + 0.5 * k2, u, uh)
        k4 = h * self.xdot(x + k3, u, uh)

        # Update next value of y
        x_new = x + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return x_new

    def xdot(self, x, u, uh):
         return np.matmul(self.A, x) + np.matmul(self.B, np.array([u+uh]))

    def simulate(self, N, h, x0, u0, uh0, C, Qh0, R, GT):
        Q = C - Qh0

        # Check which paradigm
        if GT == 0:
            # Plain LQ control, solve riccati equations
            Ph = cp.solve_continuous_are(A, B, Qh0, R)
            # Ph_hat = cp.solve_continuous_are(A, B, Qh_hat, R)
            P = cp.solve_continuous_are(A, B, Q, R)
            # print(Ph.shape, P.shape)

        else:
            # Differential game LQ control, solve coupled riccati equation
            # print(S)
            # Ph_hat = cp.solve_continuous_are(A, B, Qh_hat, R)
            P_LQ = cp.solve_continuous_are(A, B, Q, R)
            Ph_LQ = cp.solve_continuous_are(A, B, Qh0, R)
            S = 1/R * np.matmul(B, B.transpose())


            # Newon-Raphson method
            # Use LQ gains as starting point
            # Check stability
            # eigtot, eigv = np.linalg.eig(A - np.matmul(S, (P_LQ + Ph_LQ)))
            # eigr, eigv = np.linalg.eig(A - np.matmul(S, (P_LQ)))
            # eigh, eigv = np.linalg.eig(A - np.matmul(S, (Ph_LQ)))
            # # print("eigenvalues total system LQ, robot and human: ", eigtot, eigr, eigh)

            it = 50
            P_it = np.zeros((it + 1, 2, 2))
            Ph_it = np.zeros((it + 1, 2, 2))
            P_it[0, :, :] = P_LQ
            Ph_it[0, :, :] = Ph_LQ

            for i in range(it):
                Acl = A - np.matmul(S, (Ph_it[i, :, :] + P_it[i, :, :]))
                # eigsAcl, eigsvalc = np.linalg.eig(Acl)
                # print("eigenvalues Acl = ", eigsAcl, "and i = ", i)
                P_it[i + 1, :, :] = cp.solve_continuous_are(Acl, B, Q, R)
                Ph_it[i + 1, :, :] = cp.solve_continuous_are(Acl, B, Qh0, R)


            Ph = Ph_it[it, :, :]
            P = P_it[it, :, :]

        Lh = 1/R*np.matmul(B.transpose(), Ph)
        # Lh_hat = np.matmul(B.transpose(), Ph_hat)
        L = 1/R*np.matmul(B.transpose(), P)
        Lhe = Lh[0,0]
        Lhv = Lh[0,1]
        # print(Lh, Lhe, Lhv)

        # print(Lh, Lh_hat, L)

        x = np.zeros((2, N + 1))
        u = np.zeros(N)
        uh = np.zeros(N)
        x[:, 0] = x0
        u[0] = u0
        uh[0] = uh0

        for i in range(N):
            u[i] = np.matmul(-L, x[:, i])
            uh[i] = np.matmul(-Lh, x[:, i])
            x[:, i + 1] = dynamics_model.RK4(x[:, i], u[i], uh[i], h)

        return u, uh, x, Lhe, Lhv



# Dynamics
I = 6 #kg
D = -0.2 #N/m
# TODO: Note that xd is fixed! If xd_dot =/= 0 this does not work!
A = np.array([ [0, 1],[0, -D/I]])
B = np.array([[0], [1/I]])
Gamma = np.array([[100, 0], [0, 1]])

# Initial values
pos0 = 0
vel0 = 0
x_d = 0.1

# Simulation
time = 4
h = 0.005
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
Qh_hat = np.array([[Qh_e_hat, 0],[0, Qh_v_hat]] )

# Robot cost values
Cval = 200
C = np.array( [[Cval, 0],[0, 0]] )
R = np.array([[1]])

# Initial values
x0 = np.array([pos0-x_d, vel0])
u0 = 0
uh0 = 0

# Initialize model
dynamics_model = DynamicsModel(A, B, I, D)

figa, (ax1a, ax2a) = plt.subplots(2)
figa.suptitle('State values')
plt.xlabel("Time [s]")
plt.ylabel("Position, Velocity [m, m/s]")

figb, (ax1b, ax2b) = plt.subplots(2)
figb.suptitle('Input values')
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")

# First plot a distinct GT vs LQ response
# Qh0 = np.array([[Qh_e, 0],[0, Qh_v]])
u_LQ0, uh_LQ0, x_LQ0, Lhe_LQ0, Lhv_LQ0 = dynamics_model.simulate(N, h, x0, u0, uh0, C, Qh0, R, GT=0)
u_GT0, uh_GT0, x_GT0, Lhe_GT0, Lhv_GT0 = dynamics_model.simulate(N, h, x0, u0, uh0, C, Qh0, R, GT=1)
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
    u_LQ[i, :], uh_LQ[i, :], x_LQ[i, : ,:], Lhe_LQ[i], Lhv_LQ[i] = dynamics_model.simulate(N, h, x0, u0, uh0, C, Qh0, R, GT=0)
    u_GT[i, :], uh_GT[i, :], x_GT[i, :, :], Lhe_GT[i], Lhv_GT[i] = dynamics_model.simulate(N, h, x0, u0, uh0, C, Qh0, R, GT=1)
    # labels_LQ = "LQ Qh = " + str(Qh_e[i])
    # labels_GT = "GT Qh = " + str(Qh_e[i])

    # ax1a.plot(T, x_LQ[i,0,:-1], label=labels_LQ)
    # ax1a.plot(T, x_GT[i, 0, :-1],'--', label=labels_GT)
    # ax1a.set_title("Position error")
    # ax1a.legend()
    # ax2a.plot(T, x_LQ[i,1,:-1], label=labels_LQ)
    # ax2a.plot(T, x_GT[i, 1, :-1],'--', label=labels_GT)
    # ax2a.set_title("Velocity error")
    # ax2a.legend()
    #
    # ax1b.plot(T, u_LQ[i,:], label=labels_LQ)
    # ax1b.plot(T, u_GT[i, :],'--', label=labels_GT)
    # ax1b.set_title("Robot control action")
    # ax1b.legend()
    # ax2b.plot(T, uh_LQ[i,:], label=labels_LQ)
    # ax2b.plot(T, uh_GT[i, :],'--', label=labels_GT)
    # ax2b.set_title("Human control action")
    # ax2b.legend()




plt.figure(3)
plt.plot(fractions, Lhe_LQ, '+')
plt.plot(fractions, Lhe_GT, 'o')

plt.figure(4)
plt.plot(fractions, Lhv_LQ, '+')
plt.plot(fractions, Lhv_GT, 'o')

plt.show()


