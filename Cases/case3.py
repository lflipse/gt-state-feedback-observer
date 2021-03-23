# CASE 2: Fix L_h, estimate L_r online #

import numpy as np
import scipy.linalg as cp
import matplotlib.pyplot as plt

class DynamicsModel:
    def __init__(self, A, B, I, D, alpha, Gamma):
        self.A = A
        self.B = B
        self.I = I
        self.D = D
        self.alpha = alpha
        self.Gamma = Gamma

    def RK4(self, Q, L_h, r, y, h):
        k1 = h * self.ydot(Q, L_h, r, y)
        k2 = h * self.ydot(Q, L_h, r, y + 0.5 * k1)
        k3 = h * self.ydot(Q, L_h, r, y + 0.5 * k2)
        k4 = h * self.ydot(Q, L_h, r, y + k3)

        # Update next value of y
        y_new = y + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return y_new

    def ydot(self, Q, L_h, r, y):
        x = np.array([[y[0]], [y[1]]])
        e = x - np.array([[r[0]], [r[1]]])
        x_hat = np.array([[y[2]], [y[3]]])
        x_tilde = np.array([[y[4]], [y[5]]])

        P_hhat = np.array([[y[6], y[7]], [y[8], y[9]]])
        L_hhat = 1 / R * np.matmul(B.transpose(), P_hhat)
        A_r = A - B * L_hhat
        P_r = cp.solve_continuous_are(A_r, B, Q, R)
        L_r = 1 / R * np.matmul(B.transpose(), P_r)


        # Calculate derivatives
        x_dot = np.matmul(self.A, x) - np.matmul(self.B * (L_h + L_r), e)
        x_hat_dot = np.matmul(self.A, x_hat) - np.matmul(B * (L_hhat + L_r), e) - np.matmul(self.Gamma, x_tilde)
        x_tilde_dot = np.matmul((self.A - self.Gamma), x_tilde) - np.matmul(self.B * (L_hhat - L_h), e)
        P_hhat_dot = self.alpha * (x_tilde) * e.transpose()
        # P_hhat_dot0 = self.alpha * (e_tilde[0] - e[0]) * e[0]
        # P_hhat_dot1 = self.alpha * (e_tilde[0] - e[0]) * e[1]
        # P_hhat_dot2 = self.alpha * (e_tilde[1] - e[1]) * e[0]
        # P_hhat_dot3 = self.alpha * (e_tilde[1] - e[1]) * e[1]
        # # print(P_hhat_dot)
        # # ydot = np.array([x_dot.transpose(), e_hat_dot.transpose(), e_tilde_dot.transpose(), np.array([P_hhat_dot0, P_hhat_dot1]).transpose(), np.array([P_hhat_dot2, P_hhat_dot3]).transpose()]).flatten()
        ydot = np.array([x_dot.transpose(), x_hat_dot.transpose(), x_tilde_dot.transpose(),
                         [P_hhat_dot[0]], [P_hhat_dot[1]]]).flatten()
        # print(ydot)
        return ydot


    def simulate(self, N, h, r, y0, u0, uh0, C, Qh0, R, GT):
        Q = C - Qh0

        # Fix human gain
        Ph = cp.solve_continuous_are(A, B, Qh0, R)
        L_h0 = 1/R * np.matmul(B.transpose(), Ph)
        # print("L_h = ", L_h)
        # Lh_hat = np.matmul(B.transpose(), Ph_hat)
        # print(Lh, Lhe, Lhv)

        # print(Lh, Lh_hat, L)

        # y = [x, x_hat, x_tilde, Ph1, Ph2]
        y = np.zeros((10, N + 1))
        P_hhat = np.zeros((2, 2, N))
        L_hhat = np.zeros((2, N))
        L_h = np.zeros((2, N))
        L_r = np.zeros((2, N))
        x = np.zeros((2, N))
        ref = np.zeros((2, N))
        x_hat = np.zeros((2, N))
        x_tilde = np.zeros((2, N))
        u = np.zeros(N)
        uh = np.zeros(N)
        y[:, 0] = y0
        u[0] = u0
        uh[0] = uh0

        for i in range(N):
            P_hhat[:, :, i] = np.array([[y[6, i], y[7, i]], [y[8, i], y[9, i]]])
            L_hhat[:, i] = 1 / R * np.matmul(B.transpose(), P_hhat[:, :, i])
            A_r = A - B * L_hhat[:, i]
            P_r = cp.solve_continuous_are(A_r, B, Q, R)
            L_r[:, i] = 1 / R * np.matmul(B.transpose(), P_r)
            A_h = A - B * L_r[:, i]
            P_h = cp.solve_continuous_are(A_h, B, Qh0, R)
            L_h[:, i] = 1 / R * np.matmul(B.transpose(), P_h)
            if i>0:
                ref[:,i] = np.array([r[i], (r[i]-r[i-1])/h])
            else:
                ref[:,i] = np.array([r[i], 0])
            u[i] = np.inner(-L_r[:, i], (y[0:2, i] - ref[:,i]))
            uh[i] = np.inner(-L_h[:, i], (y[0:2, i] - ref[:,i]))

            x[:, i] = y[0:2, i]
            x_hat[:, i] = y[2:4, i]
            x_tilde[:, i] = y[4:6, i]


            # print(ref)

            y[:, i + 1] = dynamics_model.RK4(Q, L_h[:, i], ref[:,i], y[:, i], h)

        return u, uh, y, L_hhat, L_h, L_r, x, x_hat, x_tilde, ref



# Dynamics
I = 6 #kg
D = -0.2 #N/m
# TODO: Note that xd is fixed! If xd_dot =/= 0 this does not work!
A = np.array([ [0, 1],[0, -D/I]])
B = np.array([[0], [1/I]])
Gamma = np.array([[100, 0], [0, 1]])
alpha = 100000

# Initial values
pos0 = 0
vel0 = 0
x_d = 0.1

# Simulation
time = 20
h = 0.01
N = round(time/h)
T = np.array(range(N)) * h

# Simulated Human Settings
# True cost values
Qh_e = 120
Qh_v = 0
Qh0 = np.array([[Qh_e, 0], [0, Qh_v]])

# Estimated cost value
Qh_e_hat = 20
Qh_v_hat = 0.01
Qh_hat = np.array([[Qh_e_hat, 0], [0, Qh_v_hat]])

# Robot cost values
Cval = 200
C = np.array([[Cval, 0], [0, 0]])
R = np.array([[1]])

# Initial values
# r = np.array([x_d * T, x_d * np.ones(N)])
# r = np.array([x_d * np.ones(N), x_d * np.zeros(N)])
fs = 1/5
# r = np.array([x_d * np.sin(2*np.pi*fs*T), 2*np.pi*fs * x_d * np.cos(2*np.pi*fs*T)])
r = x_d * np.sin(2*np.pi*fs*T)
# r = np.array([x_d * np.sin(2*np.pi*fs*T), np.zeros(N)])

x0 = np.array([pos0, vel0])
# e0 = x0 - r[:, 0]
x_hat0 = x0
x_tilde0 = np.array([0, 0])
Ph0 = cp.solve_continuous_are(A, B, Qh_hat, R)
y0o = np.array([x0, x_hat0, x_tilde0, Ph0[0], Ph0[1]])
y0 = y0o.flatten()

u0 = 0
uh0 = 0

# Initialize model
dynamics_model = DynamicsModel(A, B, I, D, alpha, Gamma)

figa, (ax1a, ax2a) = plt.subplots(2)
figa.suptitle('State values')
plt.xlabel("Time [s]")
plt.ylabel("Position, Velocity [m, m/s]")

figb, (ax1b, ax2b) = plt.subplots(2)
figb.suptitle('Input values')
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")

figc, (ax1c, ax2c) = plt.subplots(2)
figc.suptitle('Controller gains')
plt.xlabel("Time [s]")
plt.ylabel("Gain [N]")

figd, (ax1d, ax2d) = plt.subplots(2)
figd.suptitle('Errors')
plt.xlabel("Time [s]")
plt.ylabel("Error [N]")

# First plot a distinct GT vs LQ response
# Qh0 = np.array([[Qh_e, 0],[0, Qh_v]])
# u_LQ0, uh_LQ0, x_LQ0, Lhe_LQ0, Lhv_LQ0 = dynamics_model.simulate(N, h, r, y0, u0, uh0, C, Qh0, R, GT=0)
u, uh, y, L_hhat, L_h, L_r, e, e_hat, e_tilde, ref = dynamics_model.simulate(N, h, r, y0, u0, uh0, C, Qh0, R, GT=1)
labels_LQ = "LQ Qh = " + str(Qh_e)
labels_GT = "GT Qh = " + str(Qh_e)

x_GT0 = y[0:2, :]


ax1a.plot(T, x_GT0[0, :-1],'--', label=labels_GT)
ax1a.plot(T, ref[0, :],'-', label="Reference signals")
ax1a.set_title("Position error, C = " + str(Cval))
ax1a.legend()
ax2a.plot(T, x_GT0[1, :-1],'--', label=labels_GT)
ax2a.plot(T, ref[1, :],'-', label="Reference signals")
ax2a.set_title("Velocity error, C = " + str(Cval))
ax2a.legend()

ax1d.plot(T, e[0, :], 'r--', label="Error value")
ax1d.plot(T, e_hat[0, :], 'b-', label="Estimated error value")
ax1d.plot(T, e_tilde[0, :], 'y-', label="Estimation error")
ax1d.set_title("Position error, C = " + str(Cval))
ax1d.legend()
ax2d.plot(T, e[1, :], 'r--', label="Error value")
ax2d.plot(T, e_hat[1, :], 'b-', label="Estimated error value")
ax2d.plot(T, e_tilde[1, :], 'y-', label="Estimation error")
ax2d.set_title("Velocity error, C = " + str(Cval))
ax2d.legend()


ax1b.plot(T, u ,'--', label=labels_GT)
ax1b.set_title("Robot control action, C = " + str(Cval))
ax1b.legend()
ax2b.plot(T, uh ,'--', label=labels_GT)
ax2b.set_title("Human control action, C = " + str(Cval))
ax2b.legend()

ax1c.plot(T, L_hhat[0,:],'--', label="Estimated gain")
ax1c.plot(T, L_h[0,:],'--', label="Real gain")
ax1c.plot(T, L_r[0,:],'--', label="Robot gain")
ax1c.set_title("error gain, C = " + str(Cval))
ax1c.legend()
ax2c.plot(T, L_hhat[1,:],'--', label="Estimated gain")
ax2c.plot(T, L_h[1,:],'--', label="Real gain")
ax2c.plot(T, L_r[1,:],'--', label="Robot gain")
ax2c.set_title("velocity gain, C = " + str(Cval))
ax2c.legend()

plt.show()

