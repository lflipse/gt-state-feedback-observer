import numpy as np
import scipy.linalg as cp
import time

class ControllerLi:
    def __init__(self, A, B, mu, sigma, nonlin):
        self.A = A
        self.B = B
        self.mu = mu
        self.sigma = sigma
        self.nonlin = nonlin

    def numerical_integration(self, r, ur, uh, uhhat, y, h, alpha, Gamma):
        k1 = h * self.ydot(r, ur, uh, uhhat, y, alpha, Gamma)
        k2 = h * self.ydot(r, ur, uh, uhhat, y + 0.5 * k1, alpha, Gamma)
        k3 = h * self.ydot(r, ur, uh, uhhat, y + 0.5 * k2, alpha, Gamma)
        k4 = h * self.ydot(r, ur, uh, uhhat, y + k3, alpha, Gamma)

        # Update next value of y
        y_new = y + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return y_new

    def ydot(self, r, ur, uh, uhhat, y, alpha, Gamma):
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
        x_hat_dot = np.matmul(self.A, x_hat) + self.B * (ur + uhhat) - np.matmul(Gamma, x_tilde)

        # Estimation error
        x_tilde_dot = x_hat_dot - x_dot

        # P matrix update rule
        P_hhat_dot = alpha * (x_tilde) * (e).transpose()

        ydot = np.array([x_dot.transpose(), x_tilde_dot.transpose(), x_hat_dot.transpose(), [P_hhat_dot[0]], [P_hhat_dot[1]]]).flatten()

        return ydot

    def compute_gains(self, Q, Phat):
        Lhat = np.matmul(self.B.transpose(), Phat)
        A_c = self.A - self.B * Lhat
        P = cp.solve_continuous_are(A_c, self.B, Q, 1)
        L = np.matmul(self.B.transpose(), P)
        return Lhat, L, P

    def compute_inputs(self, Q, Phat, e):
        Lhat, L, P = self.compute_gains(Q, Phat)
        u = np.inner(-L, e)
        uhat = np.inner(-Lhat, e)
        return u, uhat, L, Lhat, P

    def update_costs(self, Q, Phat):
        Lhat, L, P = self.compute_gains(Q, Phat)
        A_c = self.A - self.B * L
        Qhat = - np.matmul(A_c.transpose(), Phat.transpose()) - np.matmul(Phat, A_c) + np.matmul(Phat, self.B) \
               * np.matmul(self.B.transpose(), Phat.transpose())
        return Qhat

    def compute_costs(self, x, u, Q):
        return np.matmul(np.matmul(x, Q), x.transpose()) + u**2

    def simulate(self, inputs):
        # Time entire operation
        print("Starting simulation")
        start = time.time()

        # Unpack dictionary
        N = inputs["simulation_steps"]
        h = inputs["step_size"]
        r = inputs["reference_signal"]
        Qh0 = inputs["human_weight"]
        alpha = inputs["alpha"]
        Gamma = inputs["Gamma"]
        Lh0 = np.array(inputs["virtual_human_gain"])
        C = inputs["sharing_rule"]
        T = np.array(range(N)) * h

        # State vector
        y = np.zeros((N + 1, 10))

        # Estimator vectors
        Qhhat = np.zeros((N + 1, 2, 2))
        Phhat = np.zeros((N + 1, 2, 2))
        Ph = np.zeros((N, 2, 2))
        Pr = np.zeros((N, 2, 2))
        Qh = np.zeros((N, 2, 2))
        Qr = np.zeros((N, 2, 2))
        Lhhat = np.zeros((N, 2))
        uhhat = np.zeros(N)

        # Real vectors
        Lh = np.zeros((N, 2))
        Lr = np.zeros((N, 2))
        ref = np.zeros((N, 2))
        e = np.zeros((N, 2))
        ur = np.zeros(N)
        uhbar = np.zeros(N)
        vh = np.zeros(N)
        uh = np.zeros(N)
        Jh = np.zeros(N)
        Jhhat = np.zeros(N)
        Jr = np.zeros(N)

        for i in range(N):
            # Human cost is fixed, Robot cost based on estimator
            Qr[i, :, :] = C - Qhhat[i, :, :]
            Qh[i, :, :] = Qh0

            # Calcuate derivative(s) of reference
            if i > 0:
                ref[i, :] = np.array([r[i], (r[i] - r[i - 1]) / h])
            else:
                ref[i, :] = np.array([r[i], (r[i+1] - r[i]) / h]) # Compensate strange bumps

            # Compute inputs
            e[i, :] = y[i, 0:2] - ref[i, :]
            ur[i], uhhat[i], Lr[i, :], Lhhat[i, :], Pr[i, :, :] = self.compute_inputs(Qr[i, :, :], Phhat[i, :, :], e[i, :])
            try:
                Lh[i, :] = Lh0[:, i]
                uhbar[i] = np.matmul(-Lh[i, :], e[i, :])
            except:
                Lh[i, :] = Lh0[:, i].flatten()
                uhbar[i] = np.matmul(-Lh[i, :], e[i, :])
            uh[i] = uhbar[i]

            # Integrate a time-step
            y[i + 1, :] = self.numerical_integration(ref[i, :], ur[i], uh[i], uhhat[i], y[i, :], h, alpha, Gamma)
            Phhat[i + 1, :, :] = np.array([[y[i + 1, 6], y[i + 1, 8]], [y[i + 1, 8], y[i + 1, 9]]])
            Jh[i] = self.compute_costs(e[i, :], uh[i], Qh[i, :, :])
            Jhhat[i] = self.compute_costs(e[i, :], uhhat[i], Qhhat[i, :, :])
            Jr[i] = self.compute_costs(e[i, :], ur[i], Qr[i, :, :])

            # Update cost matrices
            Qhhat[i + 1, :, :] = self.update_costs(Qr[i, :, :], Phhat[i + 1, :, :])
            Qhhat[i + 1, 0, 1] = 0
            Qhhat[i + 1, 1, 0] = 0

        outputs = {
            "states": y[:, 0:2],
            "time": T,
            "reference_signal": ref,
            "error_states": e,
            "human_input": uh,
            "human_estimated_input": uhhat,
            "human_estimated_Q": Qhhat,
            "human_Q": Qh,
            "robot_Q": Qr,
            "human_estimated_P": Phhat,
            "human_P": Ph,
            "robot_P": Pr,
            "human_estimated_gain": Lhhat,
            "human_gain": Lh,
            "robot_gain": Lr,
            "robot_input": ur,
            "human_costs": Jh,
            "human_estimated_costs": Jhhat,
            "robot_costs": Jr,
        }

        return outputs