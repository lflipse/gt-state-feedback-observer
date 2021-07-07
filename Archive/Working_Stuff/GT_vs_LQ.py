# CASE 1: Solve coupled riccati equations #

import numpy as np
import scipy.linalg as cp
import matplotlib.pyplot as plt



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

    def solve_coupled_riccati(self, S, Q, Qh, P0, Ph0, it):
        P_it = np.zeros((it + 1, 2, 2))
        Ph_it = np.zeros((it + 1, 2, 2))
        P_it[0, :, :] = P0
        Ph_it[0, :, :] = Ph0

        for i in range(it):
            # Acl = A - np.matmul(S, (Ph_it[i, :, :] + P_it[i, :, :]))
            Acl = A - np.matmul(S, (Ph_it[i, :, :]))
            Aclh = A - np.matmul(S, (P_it[i, :, :]))
            # eigsAcl, eigsvalc = np.linalg.eig(Acl)
            # print("eigenvalues Acl = ", eigsAcl, "and i = ", i)
            P_it[i + 1, :, :] = cp.solve_continuous_are(Acl, B, Q, R)
            Ph_it[i + 1, :, :] = cp.solve_continuous_are(Aclh, B, Qh, R)

        Ph = Ph_it[it, :, :]
        P = P_it[it, :, :]
        return P, Ph

    def simulate(self, N, h, x0, u0, uh0, C, Qh0, R, r, GT):
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
            # (https://link-springer-com.tudelft.idm.oclc.org/content/pdf/10.1007%2Fs10287-006-0030-z.pdf#page=28&zoom=100,-34,154)
            # Use LQ gains as starting point
            # Check stability
            # eigtot, eigv = np.linalg.eig(A - np.matmul(S, (P_LQ + Ph_LQ)))
            # eigr, eigv = np.linalg.eig(A - np.matmul(S, (P_LQ)))
            # eigh, eigv = np.linalg.eig(A - np.matmul(S, (Ph_LQ)))
            # # print("eigenvalues total system LQ, robot and human: ", eigtot, eigr, eigh)

            it = 10
            P, Ph = self.solve_coupled_riccati(S, Q, Qh0, P_LQ, Ph_LQ, it)




        Lh = 1/R*np.matmul(B.transpose(), Ph)
        # Lh_hat = np.matmul(B.transpose(), Ph_hat)
        L = 1/R*np.matmul(B.transpose(), P)
        Lhe = Lh[0,0]
        Lhv = Lh[0,1]
        # print(Lh, Lhe, Lhv)

        # print(Lh, Lh_hat, L)

        # x = np.zeros((2, N + 1))
        x = np.zeros((2, N +1))
        ref = np.zeros((N, 2))
        e = np.zeros((2, N))
        u = np.zeros(N)
        uh = np.zeros(N)
        Jr = np.zeros((N, 3))
        Jq = np.zeros((N, 3))
        Jt = np.zeros((N, 3))
        x[:, 0] = x0
        u[0] = u0
        uh[0] = uh0


        for i in range(N):
            # print(r[i])
            if i > 0:
                ref[i, :] = np.array([r[i], (r[i] - r[i - 1]) / h])
            else:
                ref[i, :] = np.array([r[i], (r[i]) / h])

            # Derive inputs
            e[:, i] = x[:, i] - ref[i, :]
            u[i] = np.matmul(-L, e[:, i])
            uh[i] = np.matmul(-Lh, e[:, i])

            # Cost function values
            Jr[i, 0] = uh[i] ** 2 * R
            Jr[i, 1] = u[i] ** 2 * R
            Jr[i, 2] = Jr[i, 0] + Jr[i, 1]
            Jq[i, 0] = np.matmul(np.matmul(e[:, i].transpose(), Qh0), e[:, i])
            Jq[i, 1] = np.matmul(np.matmul(e[:, i].transpose(), Q), e[:, i])
            Jq[i, 2] = Jq[i, 0] + Jq[i, 1]
            Jt[i, 0] = Jr[i, 0] + Jq[i, 0]
            Jt[i, 1] = Jr[i, 1] + Jq[i, 1]
            Jt[i, 2] = Jr[i, 2] + Jq[i, 2]


            x[:, i + 1] = dynamics_model.RK4(x[:, i], u[i], uh[i], h)

        return u, uh, x, Lhe, Lhv, ref, Jr, Jq, Jt

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
time = 10
h = 0.005
N = round(time/h)
T = np.array(range(N)) * h

# Initial values
fs = 1/8
r = x_d * np.sin(2*np.pi*fs*T)

# Simulated Human Settings
# True cost values
Qh_e = 80
Qh_v = 0
Qh0 = np.array([[Qh_e, 0],[0, Qh_v]])

# Estimated cost value
Qh_e_hat = 80
Qh_v_hat = 0
Qh_hat = np.array([[Qh_e_hat, 0],[0, Qh_v_hat]] )

# Robot cost values
Cval = 200
C = np.array( [[Cval, 0],[0, 0]] )
R = np.array([[1]])

# Initial values
x0 = np.array([pos0, vel0])
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

figc, (ax1c, ax2c, ax3c) = plt.subplots(3)
figc.suptitle('Cost function values')
plt.xlabel("Time [s]")

# First plot a distinct GT vs LQ response
# Qh0 = np.array([[Qh_e, 0],[0, Qh_v]])
u_LQ0, uh_LQ0, x_LQ0, Lhe_LQ0, Lhv_LQ0, ref, Jr_LQ, Jq_LQ, Jt_LQ = dynamics_model.simulate(N, h, x0, u0, uh0, C, Qh0, R, r, GT=0)
u_GT0, uh_GT0, x_GT0, Lhe_GT0, Lhv_GT0, ref, Jr_GT, Jq_GT, Jt_GT = dynamics_model.simulate(N, h, x0, u0, uh0, C, Qh0, R, r, GT=1)
labels_LQ = "LQ control"
labels_GT = "GT LQ control"

ax1a.plot(T, x_LQ0[0, :-1], 'r-.', label=labels_LQ)
ax1a.plot(T, x_GT0[0, :-1],'b--', label=labels_GT)
ax1a.plot(T, ref[:, 0],'k-', label="reference")
ax1a.set_title("Position")
ax1a.legend()
ax1a.set(xlim=(0,10))
ax2a.plot(T, x_LQ0[1, :-1], 'r-.', label=labels_LQ)
ax2a.plot(T, x_GT0[1, :-1],'b--', label=labels_GT)
ax2a.plot(T, ref[:, 1],'k-', label="reference")
ax2a.set_title("Velocity")
ax2a.set(xlim=(0,10))
ax2a.legend()

ax1b.plot(T, u_LQ0, 'r-.', label=labels_LQ)
ax1b.plot(T, u_GT0,'b--', label=labels_GT)
ax1b.set_title("Robot control action, Qr = " + str(Cval - Qh_e))
ax1b.set(xlim=(0, 10),ylim=(-0.5, 1))
ax1b.legend()
ax2b.plot(T, uh_LQ0, 'r-.', label=labels_LQ)
ax2b.plot(T, uh_GT0,'b--', label=labels_GT)
ax2b.set_title("Human control action, Qh = " + str(Qh_e))
ax2b.set(xlim=(0, 10),ylim=(-0.5, 1))
ax2b.legend()

ax1c.plot(T, Jr_LQ[:, 0], 'r-.', label="LQ: Human input cost")
ax1c.plot(T, Jr_LQ[:, 1], 'b-.', label="LQ: Robot input cost")
ax1c.plot(T, Jr_LQ[:, 2], 'k-.', label="LQ: Total input cost")
ax1c.plot(T, Jr_GT[:, 0], 'r-', label="GT: Human input cost")
ax1c.plot(T, Jr_GT[:, 1], 'b-', label="GT: Robot input cost")
ax1c.plot(T, Jr_GT[:, 2], 'k-', label="GT: Total input cost")
ax1c.set_title("Input costs")
ax1c.set(xlim=(0, 10))
ax1c.legend()
ax2c.plot(T, Jq_LQ[:, 0], 'r-.', label="LQ: Human error cost")
ax2c.plot(T, Jq_LQ[:, 1], 'b-.', label="LQ: Robot error cost")
ax2c.plot(T, Jq_LQ[:, 2], 'k-.', label="LQ: Total error cost")
ax2c.plot(T, Jq_GT[:, 0], 'r-', label="GT: Human error cost")
ax2c.plot(T, Jq_GT[:, 1], 'b-', label="GT: Robot error cost")
ax2c.plot(T, Jq_GT[:, 2], 'k-', label="GT: Total error cost")
ax2c.set_title("Error costs")
ax2c.set(xlim=(0, 10))
ax2c.legend()
ax3c.plot(T, Jt_LQ[:, 0], 'r-.', label="LQ: Human total cost")
ax3c.plot(T, Jt_LQ[:, 1], 'b-.', label="LQ: Robot total cost")
ax3c.plot(T, Jt_LQ[:, 2], 'k-.', label="LQ: Total total cost")
ax3c.plot(T, Jt_GT[:, 0], 'r-', label="GT: Human total cost")
ax3c.plot(T, Jt_GT[:, 1], 'b-', label="GT: Robot total cost")
ax3c.plot(T, Jt_GT[:, 2], 'k-', label="GT: Total total cost")
ax3c.set_title("Total costs")
ax3c.set(xlim=(0, 10))
ax3c.legend()

plt.show()


