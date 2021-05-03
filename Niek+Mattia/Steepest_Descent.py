# CASE 5: Fix Q_0, estimate Q_h and Q_r online, where Q_r changes and noise due to human action is added #

import numpy as np
import scipy.linalg as cp
import matplotlib.pyplot as plt
import time

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
        self.beta = B[1, 0]
        self.alpha_1 = A[1, 0]
        self.alpha_2 = A[1, 1]


    def numerical_integration(self, r, ur, uh, uhhat, y, h):
        k1 = h * self.ydot(r, ur, uh, uhhat, y)
        k2 = h * self.ydot(r, ur, uh, uhhat, y + 0.5 * k1)
        k3 = h * self.ydot(r, ur, uh, uhhat, y + 0.5 * k2)
        k4 = h * self.ydot(r, ur, uh, uhhat, y + k3)

        # Update next value of y
        y_new = y + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return y_new

    def ydot(self, r, ur, uh, uhhat, y):
        # Unpack y vector
        x = np.array([[y[0]], [y[1]]])
        x_tilde = np.array([[y[2]], [y[3]]])
        x_hat = np.array([[y[4]], [y[5]]])

        # Calculate error vector
        e = x - np.array([[r[0]], [r[1]]])

        # Calculate derivatives
        # Real response
        x_dot = np.matmul(self.A, x) + self.B * (ur + uh)

        # Estimated responses
        x_hat_dot = np.matmul(self.A, x) + self.B * (ur + uhhat)

        # Estimation error
        x_tilde_dot = x_hat_dot - x_dot

        pseudo_B = 1 / (np.matmul(B.transpose(), B)) * B.transpose()
        u_h_tilde = np.matmul(pseudo_B, x_tilde_dot)
        m_squared = 1 + 2 * np.matmul(e.transpose(), e)
        theta_dot = (1 / (self.beta * m_squared) ) * u_h_tilde * np.matmul(self.Gamma, e)
        # print(theta_dot)

        ydot = np.array([x_dot.transpose(), x_tilde_dot.transpose(), x_hat_dot.transpose(), theta_dot.transpose()]).flatten()

        return ydot

    def compute_gains(self, Q, R, Phat):
        Lhat = 1 / R * np.matmul(self.B.transpose(), Phat)
        A_c = self.A - self.B * Lhat
        P = cp.solve_continuous_are(A_c, self.B, Q, R)
        L = 1 / R * np.matmul(self.B.transpose(), P)
        return Lhat, L, P

    def compute_inputs(self, Q, R, Phat, e):
        Lhat, L, P = self.compute_gains(Q, R, Phat)
        u = np.inner(-L, e)
        uhat = np.inner(-Lhat, e)
        return u, uhat, L, Lhat, P

    def update_parameters(self, Q, R, Phat):
        Lhat, L, P = self.compute_gains(Q, R, Phat)
        gamma_1 = self.alpha_1 - self.beta ** 2 * (P[0, 1] )
        gamma_2 = self.alpha_2 - self.beta ** 2 * (P[1, 1] )
        a_hhat = - gamma_1 * Phat[1, 1] - gamma_2 * Phat[0, 1] + self.beta**2*Phat[0,1]*Phat[1,1]
        q_hhat1 = - 2 * gamma_1 * Phat[0,1] + self.beta**2*Phat[0,1]**2
        q_hhat2 = - 2 * Phat[0,1] - 2 * gamma_2 * Phat[1,1] + self.beta**2*Phat[1,1]**2
        phi = np.array([q_hhat1, q_hhat2, a_hhat])
        return phi



    def simulate(self, N, h, r, y0, Cval, Qh0, R):
        # E: 0. H - GT + Obs & R - GT + Obs, 1. H - GT + Obs & R - GT Static, 2. H - GT + Obs & R - LQ static,
        # y = [x, x_r_tilde, x_h_tilde, x_r_hat, x_h_hat, Ph1hat, Ph2hat, Pr1hat, Pr2hat]
        y = np.zeros((N + 1, 8))

        # Estimator vectors
        Qhhat = np.zeros((N + 1, 2, 2))
        Phhat = np.zeros((N + 1, 2, 2))
        Ph = np.zeros((N, 2, 2))
        Pr = np.zeros((N, 2, 2))
        Qh = np.zeros((N, 2, 2))
        Qr = np.zeros((N, 2, 2))
        Lhhat = np.zeros((N + 1, 2))
        uhhat = np.zeros(N)

        # Real vectors
        Lh = np.zeros((N, 2))
        Lr = np.zeros((N, 2))
        ref = np.zeros((N + 1, 2))
        e = np.zeros((N, 2))
        xhhat = np.zeros((N, 2))
        xrhat = np.zeros((N, 2))
        ur = np.zeros(N)
        uhbar = np.zeros(N)
        vh = np.zeros(N)
        uh = np.zeros(N)
        y[0, :] = y0


        for i in range(N):
            # Human cost is fixed, Robot cost based on estimator
            C = np.array([[Cval[i], 0], [0, 0.5*Cval[i]]])
            Qr[i, :, :] = 0.001 * C + Qhhat[i, :, :]
            Qh[i, :, :] = Qh0
            # print(Qr[i, :, :])

            # Calcuate derivative(s) of reference
            if i > 0:
                ref[i, :] = np.array([r[i], (r[i] - r[i - 1]) / h])
            else:
                ref[i, :] = np.array([r[i], (r[i]) / h])

            # Compute inputs
            xhhat[i, :] = y[i, 4:6]
            e[i, :] = (y[i, 0:2] - ref[i, :])
            ur[i], uhhat[i], Lr[i, :], Lhhat[i, :], Pr[i, :, :] = self.compute_inputs(Qr[i, :, :], R, Phhat[i, :], e[i, :])
            uhbar[i], urhat, Lh[i, :], Lrhat, Ph[i, :, :] = self.compute_inputs(Qh[i, :, :], R, Pr[i, :], e[i, :])
            vh[i] = np.random.normal(self.mu, self.sigma, 1)
            uh[i] = uhbar[i] + vh[i]

            # Integrate a time-step
            y[i + 1, :] = dynamics_model.numerical_integration(ref[i, :], ur[i], uh[i], uhhat[i], y[i, :], h)
            Phhat[i + 1, :, :] = np.array([[Phhat[i, 0, 0], y[i + 1, 6]], [y[i + 1, 6], y[i + 1, 7]]])

            # Update cost matrices
            phi = self.update_parameters(Qr[i, :, :], R, Phhat[i + 1, :, :])
            Qhhat[i + 1, 0, 0] = phi[0]
            Qhhat[i + 1, 1, 1] = phi[1]
            Phhat[i + 1, 0, 0] = phi[2]


        return ref, ur, uhbar, vh, uh, y, Lhhat, Lh, Lr, Qh, Qr, Qhhat, Ph, Pr, Phhat, e, xhhat

# Dynamics
I = 6 #kg
D = -0.2 #N/m

A = np.array([[0, 1], [0, -D/I]])
B = np.array([[0], [1/I]])

# alpha = 100000 * (Gamma)
# alpha = 100000 * np.array([[1, 0], [0, 1]])
alpha = 5000
Gamma = alpha * np.array([[1, 0], [0, 1]])
mu = 0.0
sigma = 0.0

# Initial values
pos0 = 0
vel0 = 0
x_d = 0.1

# Simulation
t = 40
h = 0.01
N = round(t/h)
T = np.array(range(N)) * h

# Simulated Human Settings
# True cost values
Qh_e = 1000
Qh_v = 500
Qh0 = np.array([[Qh_e, 0], [0, Qh_v]])

# Estimated cost value
Qh_e_hat = 0
Qh_v_hat = 0
Qh_hat = np.array([[Qh_e_hat, 0], [0, Qh_v_hat]])

# Robot cost values
# Cval = np.array([1200 * np.ones(round(0.5*N)), 3000 * np.ones(round(0.5*N))]).flatten()
Cval = np.array([3000 * np.ones(round(N))]).flatten()
C = np.array([[Cval[0], 0], [0, 100]])
R = np.array([[1]])
Qr_hat = C - Qh_hat

# Reference signal
fs1 = 1/8
fs2 = 1/20
fs3 = 1/37
fs4 = 1/27
r = x_d * (np.sin(2*np.pi*fs1*T) + np.sin(2*np.pi*fs2*T) + np.sin(2*np.pi*fs3*T) + np.sin(2*np.pi*fs4*T))

# Robot estimate has an estimator for the human cost
# Human estimate has an estimator for the robot cots
x0 = np.array([pos0, vel0])
x_r_hat0 = x0
x_h_hat0 = x0
x_r_tilde0 = np.array([0, 0])
x_h_tilde0 = np.array([0, 0])
# Ph0 = cp.solve_continuous_are(A, B, Qh_hat, R)
# Pr0 = cp.solve_continuous_are(A - 1/R * B * np.matmul(B.transpose(), Ph0), B, Qr_hat, R)
Ph0 = np.zeros((2, 2))
Pr0 = np.zeros((2, 2))

# y = [x, x_r_hat, x_r_tilde, Ph_hat, x_h_hat, x_h_tilde, Pr_hat]
y0o = np.array([x0, x_h_tilde0, x_h_hat0, Ph0[0]])
y0 = y0o.flatten()

# Initialize model
dynamics_model = DynamicsModel(A, B, I, D, alpha, Gamma, mu, sigma)

print("Starting simulation")
start = time.time()
# Simulate
ref, u_0, uh_bar_0, vh_0, uh_0, y_0, L_hhat_0, L_h_0, L_r_0, Qh_0, Qr_0, Qhhat, Ph, Pr, Phhat, e, xhhat = dynamics_model.simulate(N, h, r, y0, Cval, Qh0, R)
end = time.time()
print(t, " seconds simulation of took ", end - start, "seconds")

labels_robot = "Robot"
labels_human = "Human"
label_E0 = "Robot GT + Q-Obs"
label_E1 = "Robot static gain"
label_E0r = "Estimated human"
label_E1r = "Estimated robot"

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

figd, (ax1d, ax2d, ax3d) = plt.subplots(3)
figd.suptitle('Error weights')
plt.xlabel("Time [s]")
plt.ylabel("Error weight")

fige, ((ax1e, ax2e), (ax3e, ax4e)) = plt.subplots(2, 2)
fige.suptitle('P values')
plt.xlabel("Time [s]")
plt.ylabel("P value")

figf, ((ax1f, ax2f), (ax3f, ax4f)) = plt.subplots(2, 2)
fige.suptitle('Error and expected state values')
plt.xlabel("Time [s]")
plt.ylabel("P value")

ax1a.plot(T, y_0[:-1, 0], 'r-', label=label_E0)
ax1a.plot(T, ref[:-1, 0], 'k-', label="Reference signal")
ax1a.set_title("Position")
ax1a.legend()
ax1a.set(xlim=(0,t))
ax2a.plot(T, y_0[:-1, 1], 'r-', label=label_E0)
ax2a.plot(T, ref[:-1, 1], 'k-', label="Reference signal")
ax2a.set_title("Velocity")
ax2a.set(xlim=(0,t))
ax2a.legend()

ax1b.plot(T, u_0, 'r-', label=label_E0)
ax1b.set_title("Robot control action")
ax1b.legend()
ax1b.set(xlim=(0,t))
ax2b.plot(T, uh_0, 'r-', alpha=1, label=label_E0)
# ax2b.plot(T, uh_bar_0, 'r-', label="Noiseless Signal")
ax2b.set_title("Human control action")
ax2b.set(xlim=(0,t))
ax2b.legend()

ax1c.plot(T, L_hhat_0[:-1, 0] ,'r-', label="Estimated human gain")
ax1c.plot(T, L_h_0[:, 0] ,'r-', alpha=0.3, label="Real human gain")
# ax1c.plot(T, L_rhat_0[:, 0] ,'b-', label="Estimated robot gain")
ax1c.plot(T, L_r_0[:, 0] ,'b-', alpha=0.3, label="Real robot gain")
ax1c.set_title("Position error gains")
ax1c.set(xlim=(0,t))
ax1c.legend()
ax2c.plot(T, L_hhat_0[:-1, 1] ,'r-', label="Estimated human gain")
ax2c.plot(T, L_h_0[:, 1] ,'r-', alpha=0.3, label="Real human gain")
# ax2c.plot(T, L_rhat_0[:, 1] ,'b-', label="Estimated robot gain")
ax2c.plot(T, L_r_0[:, 1] ,'b-', alpha=0.3, label="Real robot gain")
ax2c.set_title("Velocity error gains")
ax2c.set(xlim=(0,t))
ax2c.legend()

ax1d.plot(T, Qhhat[:-1, 0, 0],'r-', label="Estimated human weight")
ax1d.plot(T, Qh_0[:, 0, 0],'r-', alpha=0.3, label="Real human weight ($Q_h = Q_{h,0}$)")
# ax1d.plot(T, Qrhat_0[:-1, 0, 0],'b-', label="Estimated robot weight")
ax1d.plot(T, Qr_0[:, 0, 0],'b-', alpha=0.3, label="Real robot weight ($Q_r = C - \hat{Q}_{h}$)")
ax1d.set_title("Position error weight")
ax1d.set(xlim=(0,t))
ax1d.legend()
ax2d.plot(T, Qhhat[:-1, 1, 1],'r-', label="Estimated human weight")
ax2d.plot(T, Qh_0[:, 1, 1],'r-', alpha=0.3, label="Real human weight ($Q_h = Q_{h,0}$)")
# ax2d.plot(T, Qrhat_0[:-1, 1, 1],'b-', label="Estimated robot weight")
ax2d.plot(T, Qr_0[:, 1, 1],'b-', alpha=0.3, label="Real robot weight ($Q_r = C - \hat{Q}_{h}$)")
ax2d.set_title("Velocity error weight")
ax2d.set(xlim=(0,t))
ax2d.legend()
ax3d.plot(T, Qhhat[:-1, 0, 1],'r-', label="Estimated human weight")
ax3d.plot(T, Qh_0[:, 0, 1],'r-', alpha=0.3, label="Real human weight ($Q_h = Q_{h,0}$)")
# ax3d.plot(T, Qrhat_0[:-1, 0, 1],'b-', label="Estimated robot weight")
ax3d.plot(T, Qr_0[:, 0, 1],'b-', alpha=0.3, label="Real robot weight ($Q_r = C - \hat{Q}_{h}$)")
ax3d.set_title("Off-diagonal error weight")
ax3d.set(xlim=(0,t))
ax3d.legend()

ax1e.plot(T, Phhat[:-1, 0, 0],'r-', label="Estimated human value")
ax1e.plot(T, Ph[:, 0, 0],'r-', alpha=0.3, label="Real human value")
# ax1e.plot(T, Prhat[:-1, 0, 0],'b-', label="Estimated robot value")
ax1e.plot(T, Pr[:, 0, 0],'b-', alpha=0.3, label="Real robot value")
ax1e.set_title("Value 0,0")
ax1e.set(xlim=(0,t))
ax1e.legend()
ax4e.plot(T, Phhat[:-1, 1, 1],'r-', label="Estimated human value")
ax4e.plot(T, Ph[:, 1, 1],'r-', alpha=0.3, label="Real human value")
# ax4e.plot(T, Prhat[:-1, 1, 1],'b-', label="Estimated robot value")
ax4e.plot(T, Pr[:, 1, 1],'b-', alpha=0.3, label="Real robot value")
ax4e.set_title("Value 1,1")
ax4e.set(xlim=(0,t))
ax4e.legend()
ax3e.plot(T, Phhat[:-1, 0, 1],'r-', label="Estimated human value")
ax3e.plot(T, Ph[:, 0, 1],'r-', alpha=0.3, label="Real human value")
# ax3e.plot(T, Prhat[:-1, 0, 1],'b-', label="Estimated robot value")
ax3e.plot(T, Pr[:, 0, 1],'b-', alpha=0.3, label="Real robot value")
ax3e.set_title("Value 0,1")
ax3e.set(xlim=(0,t))
ax3e.legend()
ax2e.plot(T, Phhat[:-1, 1, 0],'r-', label="Estimated human value")
ax2e.plot(T, Ph[:, 1, 0],'r-', alpha=0.3, label="Real human value")
# ax2e.plot(T, Prhat[:-1, 1, 0],'b-', label="Estimated robot value")
ax2e.plot(T, Pr[:, 1, 0],'b-', alpha=0.3, label="Real robot value")
ax2e.set_title("Value 1,0")
ax2e.set(xlim=(0,t))
ax2e.legend()

ax1f.plot(T, e[:, 0],'r-', label="Position error")
ax1f.set(xlim=(0,t))
ax1f.legend()
ax3f.plot(T, e[:, 1],'r-', label="Velocity error")
ax3f.set(xlim=(0,t))
ax3f.legend()
# ax2f.plot(T, xrhat[:, 0],'r-', label="Robots estimated value")
ax2f.plot(T, y_0[:-1, 0],'k-', alpha=0.3, label="Real value")
ax2f.plot(T, xhhat[:, 0],'b-', label="Humans estimated value")
ax2f.set_title("Position")
ax2f.set(xlim=(0,t))
ax2f.legend()
# ax4f.plot(T, xrhat[:, 1],'r-', label="Robots estimated value")
ax4f.plot(T, y_0[:-1, 1],'k-', alpha=0.3, label="Real value")
ax4f.plot(T, xhhat[:, 1],'b-', label="Humans estimated value")
ax4f.set_title("Velocity")
ax4f.set(xlim=(0,t))
ax4f.legend()


plt.show()