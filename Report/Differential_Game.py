import numpy as np
import scipy.linalg as cp
import time

class ControllerDG:
    def __init__(self, A, B, mu, sigma):
        self.A = A
        self.B = B
        self.mu = mu
        self.sigma = sigma

    def numerical_integration(self, r, ur, uh, y, h):
        k1 = h * self.ydot(r, ur, uh, y)
        k2 = h * self.ydot(r, ur, uh, y + 0.5 * k1)
        k3 = h * self.ydot(r, ur, uh, y + 0.5 * k2)
        k4 = h * self.ydot(r, ur, uh, y + k3)

        # Update next value of y
        y_new = y + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return y_new

    def ydot(self, r, u, uh, y):
        x = y
        return np.matmul(self.A, x) + np.matmul(self.B, np.array([u+uh]))

    def solve_coupled_riccati(self, S, Q, Qh, it):
        P_it = np.zeros((it + 1, 2, 2))
        Ph_it = np.zeros((it + 1, 2, 2))
        P_it[0, :, :] = cp.solve_continuous_are(self.A, self.B, Q, 1)
        Ph_it[0, :, :] = cp.solve_continuous_are(self.A, self.B, Qh, 1)

        for i in range(it):
            # Acl = A - np.matmul(S, (Ph_it[i, :, :] + P_it[i, :, :]))
            Acl = self.A - np.matmul(S, (Ph_it[i, :, :]))
            Aclh = self.A - np.matmul(S, (P_it[i, :, :]))
            P_it[i + 1, :, :] = cp.solve_continuous_are(Acl, self.B, Q, 1)
            Ph_it[i + 1, :, :] = cp.solve_continuous_are(Aclh, self.B, Qh, 1)

        Ph = Ph_it[it, :, :]
        P = P_it[it, :, :]
        return P, Ph

    def compute_costs(self, x, u, Q):
        return np.matmul(np.matmul(x.transpose(), Q), x) + u**2

    def simulate(self, inputs):
        # Time entire operation
        print("Starting simulation")
        start = time.time()

        # Unpack dictionary
        N = inputs["simulation_steps"]
        h = inputs["step_size"]
        r = inputs["reference_signal"]
        Qh0 = inputs["human_weight"]
        Qr0 = inputs["robot_weight"]
        T = np.array(range(N)) * h


        # Newon-Raphson method
        # (https://link-springer-com.tudelft.idm.oclc.org/content/pdf/10.1007%2Fs10287-006-0030-z.pdf#page=28&zoom=100,-34,154)
        it = 10
        S = np.matmul(self.B, self.B.transpose())
        Pr0, Ph0 = self.solve_coupled_riccati(S, Qr0, Qh0, it)

        print("P matrices are computed as: P_r = ", Pr0, " and P_h = ", Ph0)

        Lh0 = np.matmul(self.B.transpose(), Ph0)
        Lr0 = np.matmul(self.B.transpose(), Pr0)

        print("Controller gains then are computed as: L_r = ", Lr0, " and L_h = ", Lh0)

        x = np.zeros((N + 1, 2))
        ref = np.zeros((N, 2))
        e = np.zeros((N, 2))
        ur = np.zeros(N)
        uh = np.zeros(N)
        Jr = np.zeros(N)
        Jh = np.zeros(N)
        Lh = np.zeros((N, 2))
        Lr = np.zeros((N, 2))
        Qh = np.zeros((N, 2, 2))
        Qr = np.zeros((N, 2, 2))


        for i in range(N):
            # print(r[i])
            if i > 0:
                ref[i, :] = np.array([r[i], (r[i] - r[i - 1]) / h])
            else:
                ref[i, :] = np.array([r[i], (r[i]) / h])

            # Derive inputs
            Qr[i, :, :] = Qr0
            Qh[i, :, :] = Qh0
            Lh[i, :] = Lh0
            Lr[i, :] = Lr0
            e[i, :] = x[i, :] - ref[i, :]
            ur[i] = np.matmul(-Lr0, e[i, :])
            uh[i] = np.matmul(-Lh0, e[i, :])
            Jh[i] = self.compute_costs(e[i, :], uh[i], Qh0)
            Jr[i] = self.compute_costs(e[i, :], ur[i], Qr0)

            x[i + 1, :] = self.numerical_integration(ref[i, :], ur[i], uh[i], x[i, :], h)
            v = np.random.normal(self.mu, self.sigma, 1)
            x[i + 1, 1] += v

        outputs = {
            "states": x,
            "time": T,
            "reference_signal": ref,
            "error_states": e,
            "human_input": uh,
            "robot_input": ur,
            "human_costs": Jh,
            "robot_costs": Jr,
            "human_gain": Lh,
            "robot_gain": Lr,
            "human_Q": Qh,
            "robot_Q": Qr,
        }

        return outputs