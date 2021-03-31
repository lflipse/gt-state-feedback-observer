# CASE 5: Fix Q_0, estimate Q_h and Q_r online, where Q_r changes and noise due to human action is added #

import numpy as np
import scipy.linalg as cp
import matplotlib.pyplot as plt

def compute_conflict(ur, uh, h):
    N = len(ur)
    conflicts = []
    n_conf = []
    f_conflicts = []
    n_f_conf = []
    for i in range(N):
        if ur[i]*uh[i] < 0:
            conflicts.append(abs(ur[i] - uh[i]))
            n_conf.append(i)
        if ur[i]*uh[i] < -0.05:
            f_conflicts.append(abs(ur[i] - uh[i]))
            n_f_conf.append(i)
    perc_conf_time = len(conflicts)/N
    perc_f_conf_time = len(f_conflicts) / N
    avg_abs_conf = np.mean(conflicts)
    avg_abs_f_conf = np.mean(f_conflicts)
    std_conf = np.std(conflicts)
    std_f_conf = np.std(f_conflicts)
    print("Average Conflict Magnitudes (unfiltered, filtered):", avg_abs_conf, avg_abs_f_conf)
    print("Conflict time (unfiltered, filtered):", perc_conf_time, perc_f_conf_time)
    avg_abs = np.array([avg_abs_conf, avg_abs_f_conf])
    c_time = np.array([perc_conf_time, perc_f_conf_time])
    c_std = np.array([std_conf, std_f_conf])
    return n_conf, conflicts, n_f_conf, f_conflicts, avg_abs, c_time, c_std

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

    def RK4(self, r, ur, uh, urhat, uhhat, y, h):
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
        if E < 2:
            Lhat = 1 / R * np.matmul(B.transpose(), Phat)
            A_c = A - B * Lhat
            P = cp.solve_continuous_are(A_c, B, Q, R)
            print('P', P)

        # Plain LQ
        else:
            P = cp.solve_continuous_are(A, B, Q, R)
        L = 1 / R * np.matmul(B.transpose(), P)
        return L

    def update_cost_estimate(self, A, B, Q, R, Phat, E):
        Lhat = 1 / R * np.matmul(B.transpose(), Phat)
        L = self.compute_gain(A, B, Q, R, Phat, E)
        Ahat = A - B * L
        Qhat_t = 1 / R * np.matmul(np.matmul(Phat, self.B * self.B.transpose()),
                                    Phat.transpose()) - np.matmul(Ahat.transpose(), Phat.transpose()) - np.matmul(Phat, Ahat)
        Qhat_new = Qhat_t

        return Qhat_new, L, Lhat

    def simulate(self, N, h, r, y0, C, Qr0, Qh_hat, Qr_hat, R, E):
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
        if E == 0:
            Qr[0, :, :] = C - Qh_hat
            Qr[0, :, :] = C - Qr_hat
        else:
            Qr[0, :, :] = Qr0
            Qr[0, :, :] = C - Qr_hat

        # Initialize estimators
        Phhat[0, :, :] = np.array([[y[0, 10], y[0, 11]], [y[0, 12], y[0, 13]]])
        Prhat[0, :, :] = np.array([[y[0, 14], y[0, 15]], [y[0, 16], y[0, 17]]])

        # Initialize gains
        Lr[0, :] = self.compute_gain(A, B, Qr[0, :, :], R, Phhat[0, :, :], E)
        Lh[0, :] = self.compute_gain(A, B, Qh[0, :, :], R, Prhat[0, :, :], E=0)


        for i in range(N):
            # Human cost is fixed, Robot cost based on estimator
            if E == 0:
                Qh[i, :, :] = C - Qrhat[i, :, :]
                Qr[i, :, :] = C - Qhhat[i, :, :]
            else:
                Qh[i, :, :] = C - Qrhat[i, :, :]
                Qr[i, :, :] = Qr0

            # Calcuate derivative(s) of reference
            if i > 0:
                ref[i, :] = np.array([r[i], (r[i]-r[i-1])/h])
            else:
                ref[i, :] = np.array([r[i], 0])

            # Compute inputs
            ur[i] = np.inner(-Lr[i, :], (y[i, 0:2] - ref[i, :]))
            uhbar[i] = np.inner(-Lh[i, :], (y[i, 0:2] - ref[i, :]))
            vh[i] = np.random.normal(self.mu, self.sigma, 1)
            uh[i] = uhbar[i] + vh[i]

            # Compute expected inputs
            urhat[i] = np.inner(-Lrhat[i, :], (y[i, 0:2] - ref[i, :]))
            uhhat[i] = np.inner(-Lhhat[i, :], (y[i, 0:2] - ref[i, :]))

            # Integrate a time-step
            y[i + 1, :] = dynamics_model.RK4(ref[i, :], ur[i], uh[i], urhat[i], uhhat[i], y[i, :], h)

            # Update cost matrices
            Phhat[i + 1, :, :] = np.array([[y[i + 1, 10], y[i + 1, 11]], [y[i + 1, 12], y[i + 1, 13]]])
            Prhat[i + 1, :, :] = np.array([[y[i + 1, 14], y[i + 1, 15]], [y[i + 1, 16], y[i + 1, 17]]])
            # print('Qr', Qr[i, :, :])
            Qhhat[i + 1, :, :], Lr[i + 1, :], Lhhat[i + 1, :] = self.update_cost_estimate(A, B, Qr[i, :, :], R, Phhat[i + 1, :, :], E)
            # print('Qh', Qh[i, :, :])
            Qrhat[i + 1, :, :], Lh[i + 1, :], Lrhat[i + 1, :] = self.update_cost_estimate(A, B, Qh[i, :, :], R, Prhat[i + 1, :, :], E=0)

        return ref, ur, uhbar, vh, uh, y, Lhhat, Lh, Lrhat, Lr, Qhhat, Qh, Qrhat, Qr



# Dynamics
I = 6 #kg
D = -0.2 #N/m
# TODO: Note that xd is fixed! If xd_dot =/= 0 this does not work!
A = np.array([[0, 1], [0, -D/I]])
B = np.array([[0], [1/I]])
Gamma = np.array([[100, 0], [0, 1]])
alpha = 1000
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
Qh_e = 100
Qh_v = 0
Qh0 = np.array([[Qh_e, 0], [0, Qh_v]])
Qr0 = Qh0

# Estimated cost value
Qh_e_hat = 100
Qh_v_hat = 0
Qh_hat = np.array([[Qh_e_hat, 0], [0, Qh_v_hat]])

# Estimated cost value
Qr_e_hat = 100
Qr_v_hat = 0
Qr_hat = np.array([[Qr_e_hat, 0], [0, Qr_v_hat]])

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

# Robot estimate has an estimator for the human cost
# Human estimate has an estimator for the robot cots
x0 = np.array([pos0, vel0])
# e0 = x0 - r[:, 0]
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

figa, (ax1a, ax2a) = plt.subplots(2)
figa.suptitle('State values')
plt.xlabel("Time [s]")
plt.ylabel("Position, Velocity [m, m/s]")

figb, (ax1b, ax2b) = plt.subplots(2)
figb.suptitle('Input values')
plt.xlabel("Time [s]")
plt.ylabel("Force [N]")

figc, (ax1c, ax2c) = plt.subplots(2)
figc.suptitle('Robot perspective: Human controller gains')
plt.xlabel("Time [s]")
plt.ylabel("Gain")

figd, (ax1d, ax2d) = plt.subplots(2)
figd.suptitle('Human perspective: Robot controller gains')
plt.xlabel("Time [s]")
plt.ylabel("Error")

fige, (ax1e, ax2e) = plt.subplots(2)
fige.suptitle('Robot perspective: Human error weights')
plt.xlabel("Time [s]")
plt.ylabel("Error weight")

figf, (ax1f, ax2f, ax3f) = plt.subplots(3)
figf.suptitle('Human perspective: Robot error weights')
plt.xlabel("Time [s]")
plt.ylabel("Error weight")

figg, (ax1g) = plt.subplots(1)
figg.suptitle('Conflict')
plt.xlabel("Conflict time")
plt.ylabel("Average magnitude")

# First plot a distinct GT vs LQ response
# Qh0 = np.array([[Qh_e, 0],[0, Qh_v]])
# u_LQ0, uh_LQ0, x_LQ0, Lhe_LQ0, Lhv_LQ0 = dynamics_model.simulate(N, h, r, y0, u0, uh0, C, Qh0, R, GT=0)
# ref, u_0, uh_bar_0, vh_0, uh_0, y_0, L_hhat_0, L_h_0, L_rhat_0, L_r_0, Qhhat_0, Qh_0, Qrhat_0, Qr_0 = dynamics_model.simulate(
    # N, h, r, y0, C, Qr0, Qh_hat, Qr_hat, R, E=0)
# exit()
ref, u_1, uh_bar_1, vh_1, uh_1, y_1, L_hhat_1, L_h_1, L_rhat_1, L_r_1, Qhhat_1, Qh_1, Qrhat_1, Qr_1 = dynamics_model.simulate(
    N, h, r, y0, C, Qr0, Qh_hat, Qr_hat, R, E=1)
exit()
ref, u_2, uh_bar_2, vh_2, uh_2, y_2, L_hhat_2, L_h_2, L_rhat_2, L_r_2, Qhhat_2, Qh_2, Qrhat_2, Qr_2 = dynamics_model.simulate(
    N, h, r, y0, C, Qr0, Qh_hat, Qr_hat, R, E=2)

print(0)
n_conf_0, conf_0, n_f_conf_0, f_conf_0, mags_0, ctime_0, c_std_0 = compute_conflict(u_0, uh_0, h)
print(1)
n_conf_1, conf_1, n_f_conf_1, f_conf_1, mags_1, ctime_1, c_std_1 = compute_conflict(u_1, uh_1, h)
print(2)
n_conf_2, conf_2, n_f_conf_2, f_conf_2, mags_2, ctime_2, c_std_2 = compute_conflict(u_2, uh_2, h)

labels_robot = "Robot"
labels_human = "Human"
label_E0 = "A. Robot GT + Q-Obs"
label_E1 = "B. Robot GT + static Q"
label_E2 = "C. Robot LQR (static gain)"

# x = y[:-1, 0:2]
# e = x - ref
# x_r_tilde = y[:-1, 2:4]
# x_h_tilde = y[:-1, 6:8]

ax1a.plot(T, y_0[:-1, 0], 'r-', label=label_E0)
ax1a.plot(T, y_1[:-1, 0], 'g--', label=label_E1)
ax1a.plot(T, y_2[:-1, 0], 'b-.', label=label_E2)
ax1a.plot(T, ref[:, 0], 'k-', label="Reference signal")
ax1a.set_title("Position")
ax1a.legend()
ax2a.plot(T, y_0[:-1, 1], 'r-', label=label_E0)
ax2a.plot(T, y_1[:-1, 1], 'g--', label=label_E1)
ax2a.plot(T, y_2[:-1, 1], 'b-.', label=label_E2)
ax2a.plot(T, ref[:, 1], 'k-', label="Reference signal")
ax2a.set_title("Velocity")
ax2a.legend()

ax1b.plot(T, u_0, 'r-', label=label_E0)
ax1b.plot(T, u_1, 'g--', label=label_E1)
ax1b.plot(T, u_2, 'b-.', label=label_E2)
ax1b.set_title("Robot control action")
ax1b.legend()
ax2b.plot(T, uh_0, 'r-', alpha=0.3, label=label_E0)
ax2b.plot(T, uh_bar_0, 'r-', label="Noiseless Signal")
ax2b.plot(T, uh_1, 'g--', alpha=0.3, label=label_E1)
ax2b.plot(T, uh_bar_1, 'g--', label="Noiseless Signal")
ax2b.plot(T, uh_2, 'b-.', alpha=0.3, label=label_E2)
ax2b.plot(T, uh_bar_2, 'b-.', label="Noiseless Signal")
ax2b.set_title("Human control action")
ax2b.legend()

ax1c.plot(T, L_hhat_0[:-1, 0],'r-', label=label_E0)
ax1c.plot(T, L_h_0[:-1, 0],'r-', alpha=0.3, label="Real human gain")
ax1c.plot(T, L_hhat_1[:-1, 0],'g--', label=label_E1)
ax1c.plot(T, L_h_1[:-1, 0],'g--', alpha=0.3, label="Real human gain")
ax1c.plot(T, L_hhat_2[:-1, 0],'b-.', label=label_E2)
ax1c.plot(T, L_h_2[:-1, 0],'b-.', alpha=0.3, label="Real human gain")
ax1c.set_title("Human position error gains")
ax1c.legend()
ax2c.plot(T, L_hhat_0[:-1, 1],'r-', label=label_E0)
ax2c.plot(T, L_h_0[:-1, 1],'r-', alpha=0.3, label="Real human gain")
ax2c.plot(T, L_hhat_1[:-1, 1],'g--', label=label_E1)
ax2c.plot(T, L_h_1[:-1, 1],'g--', alpha=0.3, label="Real human gain")
ax2c.plot(T, L_hhat_2[:-1, 1],'b-.', label=label_E2)
ax2c.plot(T, L_h_2[:-1, 1],'b-.', alpha=0.3, label="Real human gain")
ax2c.set_title("Human velocity error gains" )
ax2c.legend()

ax1d.plot(T, L_rhat_0[:-1, 0],'r-', label=label_E0)
ax1d.plot(T, L_r_0[:-1, 0],'r-', alpha=0.3, label="Real robot gain")
ax1d.plot(T, L_rhat_1[:-1, 0],'g--', label=label_E1)
ax1d.plot(T, L_r_1[:-1, 0],'g--', alpha=0.3, label="Real robot gain")
ax1d.plot(T, L_rhat_2[:-1, 0],'b-.', label=label_E2)
ax1d.plot(T, L_r_2[:-1, 0],'b-.', alpha=0.3, label="Real robot gain")
ax1d.set_title("Robot position error gains")
ax1d.legend()
ax2d.plot(T, L_rhat_0[:-1, 1],'r-', label=label_E0)
ax2d.plot(T, L_r_0[:-1, 1],'r-', alpha=0.3, label="Real robot gain")
ax2d.plot(T, L_rhat_1[:-1, 1],'g--', label=label_E1)
ax2d.plot(T, L_r_1[:-1, 1],'g--', alpha=0.3, label="Real robot gain")
ax2d.plot(T, L_rhat_2[:-1, 1],'b-.', label=label_E2)
ax2d.plot(T, L_r_2[:-1, 1],'b-.', alpha=0.3, label="Real robot gain")
ax2d.set_title("Robot velocity error gains" )
ax2d.legend()

ax1e.plot(T, Qhhat_0[:-1,0,0], 'r-', label=label_E0)
ax1e.plot(T, Qh_0[:-1,0,0], 'r-', alpha=0.3, label="Real human weight")
ax1e.plot(T, Qhhat_1[:-1,0,0], 'g--', label=label_E1)
ax1e.plot(T, Qh_1[:-1,0,0], 'g--', alpha=0.3, label="Real human weight")
ax1e.plot(T, Qhhat_2[:-1,0,0], 'b-.', label=label_E2)
ax1e.plot(T, Qh_2[:-1,0,0], 'b-.', alpha=0.3, label="Real human weight")
ax1e.set_title("Human error weights")
ax1e.legend()
ax2e.plot(T, Qhhat_0[:-1,1,1], 'r-', label=label_E0)
ax2e.plot(T, Qh_0[:-1,1,1], 'r-', alpha=0.3, label="Real human weight")
ax2e.plot(T, Qhhat_1[:-1,1,1], 'g--', label=label_E1)
ax2e.plot(T, Qh_1[:-1,1,1], 'g--', alpha=0.3, label="Real human weight")
ax2e.plot(T, Qhhat_2[:-1,1,1], 'b-.', label=label_E2)
ax2e.plot(T, Qh_2[:-1,1,1], 'b-.', alpha=0.3, label="Real human weight")
ax2e.set_title("Human velocity error weight")
ax2e.legend()

ax1f.plot(T, Qrhat_0[:-1,0,0], 'r-', label=label_E0)
ax1f.plot(T, Qr_0[:-1,0,0], 'r-', alpha=0.3, label="Real robot weight")
ax1f.plot(T, Qrhat_1[:-1,0,0], 'g--', label=label_E1)
ax1f.plot(T, Qr_1[:-1,0,0], 'g--', alpha=0.3, label="Real robot weight")
ax1f.plot(T, Qrhat_2[:-1,0,0], 'b-.', label=label_E2)
ax1f.plot(T, Qr_2[:-1,0,0], 'b-.', alpha=0.3, label="Real robot weight")
ax1f.set_title("Robot error weights")
ax1f.legend()
ax2f.plot(T, Qrhat_0[:-1,1,1], 'r-', label=label_E0)
ax2f.plot(T, Qr_0[:-1,1,1], 'r-', alpha=0.3, label="Real robot weight")
ax2f.plot(T, Qrhat_1[:-1,1,1], 'g--', label=label_E1)
ax2f.plot(T, Qr_1[:-1,1,1], 'g--', alpha=0.3, label="Real robot weight")
ax2f.plot(T, Qrhat_2[:-1,1,1], 'b-.', label=label_E2)
ax2f.plot(T, Qr_2[:-1,1,1], 'b-.', alpha=0.3, label="Real robot weight")
ax2f.set_title("Robot velocity error weight")
ax2f.legend()
ax3f.plot(T, Qrhat_0[:-1,1,0], 'r-', label=label_E0)
ax3f.plot(T, Qr_0[:-1,1,0], 'r-', alpha=0.3, label="Real robot weight")
ax3f.plot(T, Qrhat_1[:-1,1,0], 'g--', label=label_E1)
ax3f.plot(T, Qr_1[:-1,1,0], 'g--', alpha=0.3, label="Real robot weight")
ax3f.plot(T, Qrhat_2[:-1,1,0], 'b-.', label=label_E2)
ax3f.plot(T, Qr_2[:-1,1,0], 'b-.', alpha=0.3, label="Real robot weight")
ax3f.set_title("Robot off-diagonal error weights")
ax3f.legend()


# ax1g.plot(ctime_0[0], mags_0[0], 'ro', label=label_E0)
ax1g.errorbar(ctime_0[0], mags_0[0], yerr=c_std_0[0], ecolor='r', linestyle="", marker='o', label=label_E0)
# ax1g.plot(ctime_0[1], mags_0[1], 'rx', label="Filtered")
ax1g.errorbar(ctime_0[1], mags_0[1], yerr=c_std_0[1], ecolor='r', linestyle="", marker='x', label="Filtered")
# ax1g.plot(ctime_1[0], mags_1[0], 'go', label=label_E1)
ax1g.errorbar(ctime_1[0], mags_1[0], yerr=c_std_1[0], ecolor='g', linestyle="", marker='o', label=label_E1)
# ax1g.plot(ctime_1[1], mags_1[1], 'gx', label="Filtered")
ax1g.errorbar(ctime_1[1], mags_1[1], yerr=c_std_1[1], ecolor='g', linestyle="", marker='x', label="Filtered")
# ax1g.plot(ctime_2[0], mags_2[0], 'bo', label=label_E2)
ax1g.errorbar(ctime_2[0], mags_2[0], yerr=c_std_2[0], ecolor='b', linestyle="", marker='o', label=label_E2)
# ax1g.plot(ctime_2[1], mags_2[1], 'bx', label="Filtered")
ax1g.errorbar(ctime_2[1], mags_2[1], yerr=c_std_2[1], ecolor='b', linestyle="", marker='x', label="Filtered")
ax1g.set_title("Conflict")
ax1g.legend()
plt.show()