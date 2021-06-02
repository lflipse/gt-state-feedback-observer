import numpy as np
import scipy.linalg as cp
import time

class ControllerLQ:
    def __init__(self, A, B, mu, sigma):
        self.A = A
        self.B = B
        self.mu = mu
        self.sigma = sigma
        print(A, B)

    def numerical_integration(self, r, ur, uh, y, h):
        k1 = h * self.ydot(r, ur, uh, y)
        k2 = h * self.ydot(r, ur, uh, y + 0.5 * k1)
        k3 = h * self.ydot(r, ur, uh, y + 0.5 * k2)
        k4 = h * self.ydot(r, ur, uh, y + k3)

        # Update next value of y
        # y_new = y + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

        y_new = y + h * self.ydot(r, ur, uh, y)

        return y_new.flatten()

    def ydot(self, r, u, uh, y):
        x = y
        # Compensate friction
        mag = 0.2
        # tau_f = - mag / (1 + np.exp(-2*x[1])) + mag / (1 + np.exp(2*x[1]))
        tau_f = 0

        xdot = np.matmul(self.A, x) + self.B * (u + uh + tau_f)
        return xdot

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
        x0 = inputs["initial_state"]
        r0 = inputs["initial_ref"]
        T = np.array(range(N)) * h

        Pr = cp.solve_continuous_are(self.A, self.B, Qr0, 1)
        Ph = cp.solve_continuous_are(self.A, self.B, Qh0, 1)

        print("P matrices are computed as: P_r = ", Pr, " and P_h = ", Ph)

        Lh0 = np.matmul(self.B.transpose(), Ph)
        Lr0 = np.matmul(self.B.transpose(), Pr)

        print("Controller gains then are computed as: L_r = ", Lr0, " and L_h = ", Lh0)

        x = np.zeros((N + 1, 2))
        x[0, :] = x0
        ref = np.zeros((N, 2))
        e = np.zeros((N, 2))
        ur = np.zeros(N)
        uh = np.zeros(N)
        Jr = np.zeros(N)
        Jh = np.zeros(N)
        v = np.zeros(N)
        Lh = np.zeros((N, 2))
        Lr = np.zeros((N, 2))
        Qh = np.zeros((N, 2, 2))
        Qr = np.zeros((N, 2, 2))

        # r.flatten()



        for l in range(N):
            # print(r[i])
            if l > 0:
                ref[l, :] = np.array([r[l], (r[l] - r[l - 1]) / h])
            else:
                ref[l, :] = r0

            # Derive inputs
            Qr[l, :, :] = Qr0
            Qh[l, :, :] = Qh0
            Lh[l, :] = Lh0
            Lr[l, :] = Lr0
            e[l, :] = x[l, :] - ref[l, :]
            ur[l] = np.matmul(-Lr0, e[l, :])
            uh[l] = np.matmul(-Lh0, e[l, :])
            Jh[l] = self.compute_costs(e[l, :], uh[l], Qh0)
            Jr[l] = self.compute_costs(e[l, :], ur[l], Qr0)

            x_vec = np.array([[x[l, 0]], [x[l, 1]]])

            x[l + 1, :] = self.numerical_integration(ref[l, :], ur[l], uh[l], x_vec, h)
            v[l] = np.random.normal(self.mu, self.sigma, 1)
            x[l + 1, 1] += v[l]

        outputs = {
            "time": T,
            "states": x,
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