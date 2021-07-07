# FILE TO TEST THE CONTROL PYTHON PACKAGE #

import numpy as np
import scipy.linalg as cp
import matplotlib.pyplot as plt


class DynamicsModel:
    def __init__(self, A, B):
        self.A = A
        self.B = B

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

    def simulate(self, N, h, x0, u0, uh0, C, Qh0, R):
        Q = C - Qh0
        Ph = cp.solve_continuous_are(A, B, Qh0, R)
        # Ph_hat = cp.solve_continuous_are(A, B, Qh_hat, R)
        P = cp.solve_continuous_are(A, B, Q, R)
        # print(Ph.shape, P.shape)

        Lh = np.matmul(B.transpose(), Ph)
        # Lh_hat = np.matmul(B.transpose(), Ph_hat)
        L = np.matmul(B.transpose(), P)

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

        return u, uh, x



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
time = 3
h = 0.005
N = round(time/h)
T = np.array(range(N)) * h

# Simulated Human Settings
# True cost values
Qh_e = 150
Qh_v = 0
Qh0 = np.array([[Qh_e, 0],[0, Qh_v]])

# Estimated cost value
Qh_e_hat = 100
Qh_v_hat = 4
Qh_hat = np.array([[Qh_e_hat, 0],[0, Qh_v_hat]] )

# Robot cost values
C = np.array( [[1000, 0],[0, 0.1]] )
R = np.array([[1]])

# Initial values
x0 = np.array([pos0-x_d, vel0])
u0 = 0
uh0 = 0

# Initialize model
dynamics_model = DynamicsModel(A, B)

# Simulate multiple human costs
Qh_e = [10, 100, 500, 1000]
n = len(Qh_e)
x = np.zeros((n,2, N + 1))
u = np.zeros((n,N))
uh = np.zeros((n,N))

figa, (ax1a, ax2a) = plt.subplots(2)
figa.suptitle('State values')
plt.xlabel("Time [s]")
plt.ylabel("Position, Velocity [m, m/s]")

figb = plt.figure(1)
figb, (ax1b, ax2b) = plt.subplots(2)
figb.suptitle('Input values')
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")

for i in range(n):
    Qh0 = np.array([[Qh_e[i], 0],[0, Qh_v]])
    u[i,:], uh[i,:], x[i,:,:] = dynamics_model.simulate(N, h, x0, u0, uh0, C, Qh0, R)
    labels = "Qh = " + str(Qh_e[i])

    ax1a.plot(T, x[i,0,:-1], label=labels)
    ax1a.set_title("Position error, C= 1000")
    ax1a.legend()
    ax2a.plot(T, x[i,1,:-1], label=labels)
    ax2a.set_title("Velocity error, C= 1000")
    ax2a.legend()

    ax1b.plot(T, u[i,:], label=labels)
    ax1b.set_title("Robot control action, C= 1000")
    ax1b.legend()
    ax2b.plot(T, uh[i,:], label=labels)
    ax2b.set_title("Human control action, C= 1000")
    ax2b.legend()


plt.show()


