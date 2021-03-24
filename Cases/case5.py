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

    def RK4(self, Qr, Qh, r, y, h):
        k1 = h * self.ydot(Qr, Qh, r, y)
        k2 = h * self.ydot(Qr, Qh, r, y + 0.5 * k1)
        k3 = h * self.ydot(Qr, Qh, r, y + 0.5 * k2)
        k4 = h * self.ydot(Qr, Qh, r, y + k3)

        # Update next value of y
        y_new = y + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return y_new

    def ydot(self, Qr, Qh, r, y):
        x = np.array([[y[0]], [y[1]]])
        e = x - np.array([[r[0]], [r[1]]])
        x_r_hat = np.array([[y[2]], [y[3]]])
        x_r_tilde = np.array([[y[4]], [y[5]]])
        x_h_hat = np.array([[y[10]], [y[11]]])
        x_h_tilde = np.array([[y[12]], [y[13]]])

        P_hhat = np.array([[y[6], y[7]], [y[8], y[9]]])
        P_rhat = np.array([[y[14], y[15]], [y[16], y[17]]])
        L_hhat = 1 / R * np.matmul(B.transpose(), P_hhat)
        L_rhat = 1 / R * np.matmul(B.transpose(), P_rhat)
        A_r = A - B * L_hhat
        P_r = cp.solve_continuous_are(A_r, B, Qr, R)
        L_r = 1 / R * np.matmul(B.transpose(), P_r)
        A_h = A - B * L_rhat
        P_h = cp.solve_continuous_are(A_h, B, Qh, R)
        L_h = 1 / R * np.matmul(B.transpose(), P_h)


        # Calculate derivatives
        # Real response
        x_dot = np.matmul(self.A, x) - np.matmul(self.B * (L_h + L_r), e)

        # Robot estimated response
        x_r_hat_dot = np.matmul(self.A, x_r_hat) - np.matmul(B * (L_hhat + L_r), e) - np.matmul(self.Gamma, x_r_tilde)
        x_r_tilde_dot = np.matmul((self.A - self.Gamma), x_r_tilde) - np.matmul(self.B * (L_hhat - L_h), e)
        P_hhat_dot = self.alpha * (x_r_tilde) * e.transpose()

        # Human estimated response
        x_h_hat_dot = np.matmul(self.A, x_h_hat) - np.matmul(B * (L_h + L_rhat), e) - np.matmul(self.Gamma, x_h_tilde)
        x_h_tilde_dot = np.matmul((self.A - self.Gamma), x_h_tilde) - np.matmul(self.B * (L_rhat - L_r), e)
        P_rhat_dot = self.alpha * (x_h_tilde) * e.transpose()

        ydot = np.array([x_dot.transpose(), x_r_hat_dot.transpose(), x_r_tilde_dot.transpose(),
                         [P_hhat_dot[0]], [P_hhat_dot[1]], x_h_hat_dot.transpose(), x_h_tilde_dot.transpose(),
                         [P_rhat_dot[0]], [P_rhat_dot[1]]]).flatten()

        return ydot


    def simulate(self, N, h, r, y0, C, Qh0, Qh_hat, Qr_hat, R, GT):

        # y = [x, x_hat, x_tilde, Ph1, Ph2]
        y = np.zeros((18, N + 1))

        # Estimators
        Qhhat = np.zeros((N + 1, 2, 2))
        Qrhat = np.zeros((N + 1, 2, 2))
        Qh = np.zeros((N + 1, 2, 2))
        Qr = np.zeros((N + 1, 2, 2))
        P_hhat = np.zeros((2, 2, N + 1))
        L_hhat = np.zeros((2, N + 1))
        P_rhat = np.zeros((2, 2, N + 1))
        L_rhat = np.zeros((2, N + 1))

        L_h = np.zeros((2, N+1))
        L_r = np.zeros((2, N+1))
        x = np.zeros((2, N))
        ref = np.zeros((2, N))
        x_r_hat = np.zeros((2, N))
        x_r_tilde = np.zeros((2, N))
        x_h_hat = np.zeros((2, N))
        x_h_tilde = np.zeros((2, N))
        u = np.zeros(N)
        uh = np.zeros(N)
        y[:, 0] = y0

        Qhhat[0, :, :] = Qh_hat
        Qrhat[0, :, :] = Qr_hat
        Qr[0, :, :] = C - Qh_hat
        Qh[0, :, :] = Qh0

        # Initialize estimators
        P_hhat[:, :, 0] = np.array([[y[6, 0], y[7, 0]], [y[8, 0], y[9, 0]]])
        P_rhat[:, :, 0] = np.array([[y[14, 0], y[15, 0]], [y[16, 0], y[17, 0]]])
        L_hhat[:, 0] = 1 / R * np.matmul(B.transpose(), P_hhat[:, :, 0])
        L_rhat[:, 0] = 1 / R * np.matmul(B.transpose(), P_rhat[:, :, 0])

        # Robot gain
        A_r = A - B * L_hhat[:, 0]
        P_r = cp.solve_continuous_are(A_r, B, Qr[0, :, :], R)
        L_r[:, 0] = 1 / R * np.matmul(B.transpose(), P_r)

        # Human gain
        A_h = A - B * L_rhat[:, 0]
        P_h = cp.solve_continuous_are(A_h, B, Qh[0, :, :], R)
        L_h[:, 0] = 1 / R * np.matmul(B.transpose(), P_h)

        for i in range(N):
            Qh[i, :, :] = Qh0
            Qr[i, :, :] = C - Qhhat[i, :, :]

            if i>0:
                ref[:, i] = np.array([r[i], (r[i]-r[i-1])/h])
            else:
                ref[:, i] = np.array([r[i], 0])
            u[i] = np.inner(-L_r[:, i], (y[0:2, i] - ref[:, i]))
            uh[i] = np.inner(-L_h[:, i], (y[0:2, i] - ref[:, i]))

            x[:, i] = y[0:2, i]
            x_r_hat[:, i] = y[2:4, i]
            x_r_tilde[:, i] = y[4:6, i]
            x_h_hat[:, i] = y[10:12, i]
            x_h_tilde[:, i] = y[12:14, i]
            y[:, i + 1] = dynamics_model.RK4(Qr[i, :, :], Qh[i, :, :], ref[:,i], y[:, i], h)

            P_hhat[:, :, i + 1] = np.array([[y[6, i + 1], y[7, i + 1]], [y[8, i + 1], y[9, i + 1]]])
            P_rhat[:, :, i + 1] = np.array([[y[14, i + 1], y[15, i + 1]], [y[16, i + 1], y[17, i + 1]]])

            # Update robot estimate of human
            L_hhat[:, i+1] = 1 / R * np.matmul(B.transpose(), P_hhat[:, :, i+1])
            A_r = A - B * L_hhat[:, i+1]
            P_r = cp.solve_continuous_are(A_r, B, Qr[i, :, :], R)
            L_r[:, i+1] = 1 / R * np.matmul(B.transpose(), P_r)
            A_hhat = A - B * L_r[:, i+1]
            Qhhat_t = 1 / R * np.matmul(np.matmul(P_hhat[:, :, i], self.B * self.B.transpose()),
                                        P_hhat[:, :, i]) - np.matmul(A_hhat.transpose(), P_hhat[:, :, i]) - np.matmul(
                P_hhat[:, :, i], A_hhat)
            # Qhhat[i + 1, :, :] = np.array([[Qhhat_t[0, 0], 0], [0, Qhhat_t[1, 1]]])
            Qhhat[i + 1, :, :] = (Qhhat_t + Qhhat_t.transpose())/2

            # update human estimate of the robot
            L_rhat[:, i + 1] = 1 / R * np.matmul(B.transpose(), P_rhat[:, :, i + 1])
            A_h = A - B * L_rhat[:, i + 1]
            P_h = cp.solve_continuous_are(A_h, B, Qh[i, :, :], R)
            L_h[:, i + 1] = 1 / R * np.matmul(B.transpose(), P_h)
            A_rhat = A - B * L_h[:, i + 1]
            Qrhat_t = 1 / R * np.matmul(np.matmul(P_rhat[:, :, i], self.B * self.B.transpose()),
                                        P_rhat[:, :, i]) - np.matmul(A_rhat.transpose(), P_rhat[:, :, i]) - np.matmul(
                P_rhat[:, :, i], A_rhat)
            Qrhat[i + 1, :, :] = (Qrhat_t + Qrhat_t.transpose()) / 2

        return u, uh, y, L_hhat, L_h, L_rhat, L_r, x, x_r_hat, x_r_tilde, x_h_hat, x_h_tilde, ref, Qhhat, Qrhat, Qh, Qr



# Dynamics
I = 6 #kg
D = -0.2 #N/m
# TODO: Note that xd is fixed! If xd_dot =/= 0 this does not work!
A = np.array([ [0, 1],[0, -D/I]])
B = np.array([[0], [1/I]])
Gamma = np.array([[100, 0], [0, 1]])
alpha = 10000

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
Qh_e = 140
Qh_v = 0
Qh0 = np.array([[Qh_e, 0], [0, Qh_v]])

# Estimated cost value
Qh_e_hat = 60
Qh_v_hat = 0
Qh_hat = np.array([[Qh_e_hat, 0], [0, Qh_v_hat]])

# Estimated cost value
Qr_e_hat = 40
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
y0o = np.array([x0, x_r_hat0, x_r_tilde0, Ph0[0], Ph0[1], x_h_hat0, x_h_tilde0, Pr0[0], Pr0[1]])
y0 = y0o.flatten()


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

fige, (ax1e, ax2e) = plt.subplots(2)
fige.suptitle('Errors')
plt.xlabel("Time [s]")
plt.ylabel("Error [N]")

# First plot a distinct GT vs LQ response
# Qh0 = np.array([[Qh_e, 0],[0, Qh_v]])
# u_LQ0, uh_LQ0, x_LQ0, Lhe_LQ0, Lhv_LQ0 = dynamics_model.simulate(N, h, r, y0, u0, uh0, C, Qh0, R, GT=0)
u, uh, y, L_hhat, L_h, L_rhat, L_r, x, x_r_hat, x_r_tilde, x_h_hat, x_h_tilde, ref, Qhhat, Qrhat, Qh, Qr = dynamics_model.simulate(N, h, r, y0, C, Qh0, Qh_hat, Qr_hat, R, GT=1)
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

ax1d.plot(T, x[0, :], 'r--', label="Error value")
ax1d.plot(T, x_h_hat[0, :], 'b-', label="Estimated human error value")
ax1d.plot(T, x_h_tilde[0, :], 'y-', label="Human estimation error")
ax1d.plot(T, x_r_hat[0, :], 'b--', label="Estimated robot error value")
ax1d.plot(T, x_r_tilde[0, :], 'y--', label="Robot estimation error")
ax1d.set_title("Position error, C = " + str(Cval))
ax1d.legend()
ax2d.plot(T, x[1, :], 'r--', label="Error value")
ax2d.plot(T, x_h_hat[1, :], 'b-', label="Estimated human error value")
ax2d.plot(T, x_h_tilde[1, :], 'y-', label="Human estimation error")
ax2d.plot(T, x_r_hat[1, :], 'b--', label="Estimated robot error value")
ax2d.plot(T, x_r_tilde[1, :], 'y--', label="Robot estimation error")
ax2d.set_title("Velocity error, C = " + str(Cval))
ax2d.legend()


ax1b.plot(T, u ,'--', label=labels_GT)
ax1b.set_title("Robot control action, C = " + str(Cval))
ax1b.legend()
ax2b.plot(T, uh ,'--', label=labels_GT)
ax2b.set_title("Human control action, C = " + str(Cval))
ax2b.legend()

ax1c.plot(T, L_hhat[0,:-1],'r--', label="Estimated human gain by robot")
ax1c.plot(T, L_rhat[0,:-1],'b--', label="Estimated robot gain by human")
ax1c.plot(T, L_h[0,:-1],'r-', label="Real human gain")
ax1c.plot(T, L_r[0,:-1],'b-', label="Real robot gain")
ax1c.set_title("error gain, C = " + str(Cval))
ax1c.legend()
ax2c.plot(T, L_hhat[1,:-1],'r--', label="Estimated human gain by robot")
ax2c.plot(T, L_rhat[1,:-1],'b--', label="Estimated robot gain by human")
ax2c.plot(T, L_h[1,:-1],'r-', label="Real human gain")
ax2c.plot(T, L_r[1,:-1],'b-', label="Real robot gain")
ax2c.set_title("velocity gain, C = " + str(Cval))
ax2c.legend()

ax1e.plot(T, Qhhat[:-1,0,0],'r--', label="Estimated human weight")
ax1e.plot(T, Qrhat[:-1,0,0],'b--', label="Estimated robot weight")
ax1e.plot(T, Qh[:-1,0,0],'r-', label="Real human weight")
ax1e.plot(T, Qr[:-1,0,0],'b-', label="Real robot weight")
ax1e.set_title("error weight, C = " + str(Cval))
ax1e.legend()
ax2e.plot(T, Qhhat[:-1,1,1],'r--', label="Estimated human weight")
ax2e.plot(T, Qrhat[:-1,1,1],'b--', label="Estimated robot weight")
ax2e.plot(T, Qh[:-1,1,1],'r-', label="Real human weight")
ax2e.plot(T, Qr[:-1,1,1],'b-', label="Real robot weight")
ax2e.set_title("velocity gain, C = " + str(Cval))
ax2e.legend()

plt.show()