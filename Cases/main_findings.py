# CASE 5: Fix Q_0, estimate Q_h and Q_r online, where Q_r changes and noise due to human action is added #

import numpy as np
import scipy.linalg as cp
import matplotlib.pyplot as plt

class DynamicsModel:
    def __init__(self, A, B, I, D, alpha, Gamma, mu, sigma):
        self.A = A
        self.B = B
        self.I = I
        self.D = D
        self.alpha = alpha
        self.Gamma = Gamma
        self.mu = mu
        self.sigma = sigma

    def numerical_integration(self, r, ur, uh, urhat, uhhat, y, h):
        k1 = h * self.ydot(r, ur, uh, urhat, uhhat, y)
        k2 = h * self.ydot(r, ur, uh, urhat, uhhat, y + 0.5 * k1)
        k3 = h * self.ydot(r, ur, uh, urhat, uhhat, y + 0.5 * k2)
        k4 = h * self.ydot(r, ur, uh, urhat, uhhat, y + k3)

        # Update next value of y
        y_new = y + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return y_new

    def ydot(self, r, ur, uh, urhat, uhhat, y):
        # Unpack y vector
        x = np.array([[y[0]], [y[1]]])
        x_r_tilde = np.array([[y[2]], [y[3]]])
        x_h_tilde = np.array([[y[4]], [y[5]]])
        x_r_hat = np.array([[y[6]], [y[7]]])
        x_h_hat = np.array([[y[8]], [y[9]]])

        # Calculate error vector
        e = x - np.array([[r[0]], [r[1]]])

        # Calculate derivatives
        # Real response
        x_dot = np.matmul(self.A, x) + self.B * (ur + uh)

        # Estimated responses
        x_r_hat_dot = np.matmul(self.A, x_r_hat) + self.B * (ur + uhhat) - np.matmul(self.Gamma, x_r_tilde)
        x_h_hat_dot = np.matmul(self.A, x_h_hat) + self.B * (urhat + uh) - np.matmul(self.Gamma, x_h_tilde)

        # Estimation error
        x_r_tilde_dot = x_r_hat_dot - x_dot
        x_h_tilde_dot = x_h_hat_dot - x_dot

        # Update estimates of P
        P_hhat_dot = self.alpha * (x_r_tilde) * e.transpose()
        P_rhat_dot = self.alpha * (x_h_tilde) * e.transpose()

        ydot = np.array([x_dot.transpose(), x_r_tilde_dot.transpose(), x_h_tilde_dot.transpose(), x_r_hat_dot.transpose(),
                         x_h_hat_dot.transpose(), [P_hhat_dot[0]], [P_hhat_dot[1]], [P_rhat_dot[0]], [P_rhat_dot[1]]]).flatten()

        return ydot

    def compute_gain(self, A, B, Q, R, Phat, E):
        # Game-theoretical
        if E == 1:
            Lhat = 1 / R * np.matmul(B.transpose(), Phat)
            A_c = A - B * Lhat
            P = cp.solve_continuous_are(A_c, B, Q, R)

        # Plain LQ
        else:
            P = cp.solve_continuous_are(A, B, Q, R)

        L = 1 / R * np.matmul(B.transpose(), P)
        return L

    def update_cost_estimate(self, A, B, Q, R, Phat, E):
        L = self.compute_gain(A, B, Q, R, Phat, E)
        Ahat = A - B * L
        Qhat_new = 1 / R * np.matmul(np.matmul(Phat, self.B * self.B.transpose()),
                                    Phat.transpose()) - np.matmul(Ahat.transpose(), Phat.transpose()) - np.matmul(Phat, Ahat)
        return Qhat_new

    def simulate(self, N, h, r, y0, C, Qh0, Qh_hat, Qr_hat, R, E):
        # E: 0. H - GT + Obs & R - GT + Obs, 1. H - GT + Obs & R - GT Static, 2. H - GT + Obs & R - LQ static,
        # y = [x, x_r_tilde, x_h_tilde, x_r_hat, x_h_hat, Ph1hat, Ph2hat, Pr1hat, Pr2hat]
        y = np.zeros((N + 1, 18))

        # Estimator vectors
        Qhhat = np.zeros((N + 1, 2, 2))
        Qrhat = np.zeros((N + 1, 2, 2))
        Qh = np.zeros((N + 1, 2, 2))
        Qr = np.zeros((N + 1, 2, 2))
        Phhat = np.zeros((N + 1, 2, 2))
        Lhhat = np.zeros((N + 1, 2))
        Prhat = np.zeros((N + 1, 2, 2))
        Lrhat = np.zeros((N + 1, 2))
        urhat = np.zeros(N)
        uhhat = np.zeros(N)

        # Real vectors
        Lh = np.zeros((N + 1, 2))
        Lr = np.zeros((N + 1, 2))
        ref = np.zeros((N, 2))
        ur = np.zeros(N)
        uhbar = np.zeros(N)
        vh = np.zeros(N)
        uh = np.zeros(N)
        y[0, :] = y0

        # Initialize cost matrices
        Qhhat[0, :, :] = Qh_hat
        Qrhat[0, :, :] = Qr_hat

        # Initialize estimators
        Phhat[0, :, :] = np.array([[y[0, 10], y[0, 11]], [y[0, 12], y[0, 13]]])
        Prhat[0, :, :] = np.array([[y[0, 14], y[0, 15]], [y[0, 16], y[0, 17]]])


        for i in range(N):
            # Human cost is fixed, Robot cost based on estimator
            if E == 0:
                Qh[i, :, :] = Qh0
                Qr[i, :, :] = C - Qhhat[i, :, :]
            else:
                Qh[i, :, :] = Qh0
                Qr[i, :, :] = C - Qh0

            Lr[i, :] = self.compute_gain(A, B, Qr[i, :, :], R, Phhat[i, :, :], E)
            Lh[i, :] = self.compute_gain(A, B, Qh[i, :, :], R, Prhat[i, :, :], E=0)
            Lhhat[i, :] = 1 / R * np.matmul(B.transpose(), Phhat[i, :, :])
            Lrhat[i, :] = 1 / R * np.matmul(B.transpose(), Prhat[i, :, :])

            # Calcuate derivative(s) of reference
            if i > 0:
                ref[i, :] = np.array([r[i], (r[i]-r[i-1])/h])
            else:
                ref[i, :] = np.array([r[i], (r[i])/h])

            # Compute inputs
            ur[i] = np.inner(-Lr[i, :], (y[i, 0:2] - ref[i, :]))
            uhbar[i] = np.inner(-Lh[i, :], (y[i, 0:2] - ref[i, :]))
            vh[i] = np.random.normal(self.mu, self.sigma, 1)
            uh[i] = uhbar[i] + vh[i]

            # Compute expected inputs
            urhat[i] = np.inner(-Lrhat[i, :], (y[i, 0:2] - ref[i, :]))
            uhhat[i] = np.inner(-Lhhat[i, :], (y[i, 0:2] - ref[i, :]))

            # Integrate a time-step
            y[i + 1, :] = dynamics_model.numerical_integration(ref[i, :], ur[i], uh[i], urhat[i], uhhat[i], y[i, :], h)

            # Update cost matrices
            Phhat[i + 1, :, :] = np.array([[y[i + 1, 10], y[i + 1, 11]], [y[i + 1, 12], y[i + 1, 13]]])
            Prhat[i + 1, :, :] = np.array([[y[i + 1, 14], y[i + 1, 15]], [y[i + 1, 16], y[i + 1, 17]]])
            Qhhat[i + 1, :, :] = self.update_cost_estimate(A, B, Qr[i, :, :], R, Phhat[i + 1, :, :], E)
            Qrhat[i + 1, :, :] = self.update_cost_estimate(A, B, Qh[i, :, :], R, Prhat[i + 1, :, :], E=0)

        return ref, ur, uhbar, vh, uh, y, Lhhat, Lh, Lrhat, Lr, Qhhat, Qh, Qrhat, Qr

# Dynamics
I = 6 #kg
D = -0.2 #N/m
A = np.array([[0, 1], [0, -D/I]])
B = np.array([[0], [1/I]])
Gamma = np.array([[100, 0], [0, 1]])
alpha = 10000
mu = 0.0
sigma = 0.0

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
Qh_e = 40
Qh_v = 0
Qh0 = np.array([[Qh_e, 0], [0, Qh_v]])

# Estimated cost value
Qh_e_hat = 0
Qh_v_hat = 0
Qh_hat = np.array([[Qh_e_hat, 0], [0, Qh_v_hat]])

# Robot cost values
Cval = 200
C = np.array([[Cval, 0], [0, 0]])
R = np.array([[1]])
Qr_hat = C - Qh_hat

# Initial values
fs = 1/5
r = x_d * np.sin(2*np.pi*fs*T)

# Robot estimate has an estimator for the human cost
# Human estimate has an estimator for the robot cots
x0 = np.array([pos0, vel0])
x_r_hat0 = x0
x_h_hat0 = x0
x_r_tilde0 = np.array([0, 0])
x_h_tilde0 = np.array([0, 0])
Ph0 = cp.solve_continuous_are(A, B, Qh_hat, R)
Pr0 = cp.solve_continuous_are(A, B, Qr_hat, R)

# y = [x, x_r_hat, x_r_tilde, Ph_hat, x_h_hat, x_h_tilde, Pr_hat]
y0o = np.array([x0, x_r_tilde0, x_h_tilde0, x_r_hat0, x_h_hat0, Ph0[0], Ph0[1], Pr0[0], Pr0[1]])
y0 = y0o.flatten()

# Initialize model
dynamics_model = DynamicsModel(A, B, I, D, alpha, Gamma, mu, sigma)

# Simulate
ref, u_0, uh_bar_0, vh_0, uh_0, y_0, L_hhat_0, L_h_0, L_rhat_0, L_r_0, Qhhat_0, Qh_0, Qrhat_0, Qr_0 = dynamics_model.simulate(
    N, h, r, y0, C, Qh0, Qh_hat, Qr_hat, R, E=0)


labels_robot = "Robot"
labels_human = "Human"
label_E0 = "A. Robot GT + Q-Obs"
label_E1 = "B. Robot static gain"
label_E0r = "A. Human"
label_E1r = "B. Human"

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
plt.ylabel("Gain")

fige, (ax1e, ax2e, ax3e) = plt.subplots(3)
fige.suptitle('Error weights')
plt.xlabel("Time [s]")
plt.ylabel("Error weight")

ax1a.plot(T, y_0[:-1, 0], 'r-', label=label_E0)
ax1a.plot(T, ref[:, 0], 'k-', label="Reference signal")
ax1a.set_title("Position")
ax1a.legend()
ax2a.plot(T, y_0[:-1, 1], 'r-', label=label_E0)
ax2a.plot(T, ref[:, 1], 'k-', label="Reference signal")
ax2a.set_title("Velocity")
ax2a.legend()

ax1b.plot(T, u_0, 'r-', label=label_E0)
ax1b.set_title("Robot control action")
ax1b.legend()
ax2b.plot(T, uh_0, 'r-', alpha=0.3, label=label_E0)
ax2b.plot(T, uh_bar_0, 'r-', label="Noiseless Signal")
ax2b.set_title("Human control action")
ax2b.legend()

ax1c.plot(T, L_hhat_0[:-1, 0],'r-', label=label_E0)
ax1c.plot(T, L_h_0[:-1, 0],'r-', alpha=0.3, label="Real human gain")
ax1c.plot(T, L_rhat_0[:-1, 0],'b-', label=label_E0r)
ax1c.plot(T, L_r_0[:-1, 0],'b-', alpha=0.3, label="Real robot gain")
ax1c.set_title("Human position error gains")
ax1c.legend()
ax2c.plot(T, L_hhat_0[:-1, 1],'r-', label=label_E0)
ax2c.plot(T, L_h_0[:-1, 1],'r-', alpha=0.3, label="Real human gain")
ax2c.plot(T, L_rhat_0[:-1, 1],'b-', label=label_E0r)
ax2c.plot(T, L_r_0[:-1, 1],'b-', alpha=0.3, label="Real human gain")
ax2c.set_title("Human velocity error gains" )
ax2c.legend()

plt.show()