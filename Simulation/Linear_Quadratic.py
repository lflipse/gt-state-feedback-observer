import numpy as np
import scipy.linalg as cp
import time

class ControllerLQ:
    def __init__(self, A, B, mu, sigma, nonlin):
        self.A = A
        self.B = B
        self.mu = mu
        self.sigma = sigma
        self.nonlin = nonlin

    def numerical_integration(self, r, u, y, h):
        k1 = h * self.ydot(r, u, y)
        k2 = h * self.ydot(r, u, y + 0.5 * k1)
        k3 = h * self.ydot(r, u, y + 0.5 * k2)
        k4 = h * self.ydot(r, u, y + k3)

        # Update next value of y
        y_new = y + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        #
        # y_new = y + h * self.ydot(r, ur, uh, y)

        return y_new.flatten()


    def ydot(self, r, u, y):
        x = y
        g = 9.81
        m = 0.206554
        dh = 0.14113349
        dl = 0.017985
        vt = 0.22541135
        vsp = 2 * vt
        tau_d = -0.0662
        tau_fric = -0.02547518

        # Gravity
        tau_g = - m * g * dh * np.sin(x[0]) - m * g * dl * np.cos(x[0])

        # Friction
        v = x[1]
        gv = v / vsp * np.exp(-(v / (np.sqrt(2) * vsp)) ** 2 + 1 / 2)
        fc = tau_d * np.tanh(v / vt)
        tau_f = gv * tau_fric + fc

        if self.nonlin:
            xdot = np.matmul(self.A, x) + self.B * (u + tau_f + tau_g)
        else:
            xdot = np.matmul(self.A, x) + self.B * (u)
        return xdot

    def compute_costs(self, x, u, Q):
        return np.matmul(np.matmul(x.transpose(), Q), x) + u**2

    def simulate(self, inputs):
        # Time entire operation
        print("Starting simulation")
        start = time.time()

        # Unpack dictionary
        # N = inputs["simulation_steps"]
        # h = inputs["step_size"] + 0.00098
        # Qh0 = inputs["human_weight"]
        # Qr0 = inputs["robot_weight"]
        x0 = inputs["initial_state"]
        u0 = inputs["u_initial"]
        e0 = inputs["e_initial"]
        Q0 = inputs["sharing_rule"]
        ref = np.array(inputs["reference"])
        T = np.array(inputs["time"])
        N = len(T)

        P = cp.solve_continuous_are(self.A, self.B, Q0, 1)
        # Ph = cp.solve_continuous_are(self.A, self.B, Qh0, 1)

        print("Cost matrix: ", Q0)

        # print("P matrices are computed as: P_r = ", Pr, " and P_h = ", Ph)
        try:
            L0 = inputs["virtual_human_gain"]
        except:
            L0 = np.matmul(self.B.transpose(), P)
        L0 = np.matmul(self.B.transpose(), P)
        # Lr0 = np.matmul(self.B.transpose(), Pr)

        print("Controller gains then are computed as: L = ", L0)

        x = np.zeros((N + 1, 2))
        x[0, :] = x0
        # ref = np.zeros((N, 2))
        e = np.zeros((N, 2))
        u = np.zeros(N)
        J = np.zeros(N)
        v = np.zeros(N)
        L = np.zeros((N, 2))
        Q = np.zeros((N, 2, 2))

        for l in range(N-1):
            # Derive inputs
            Q[l, :, :] = Q0
            e[l, :] = x[l, :] - ref[l, :]

            try:
                # print(Lh0[l, :])
                L[l, :] = L0
                u[l] = np.matmul(-L[l, :], e[l, :])
            except:
                L[l, :] = L0[:, l].flatten()
                u[l] = np.matmul(-L[l, :], e[l, :])

            u[l] = np.inner(-L[l, :], e[l, :])
            J[l] = self.compute_costs(e[l, :], u[l], Q0)

            x_vec = np.array([[x[l, 0]], [x[l, 1]]])

            h = T[l+1] - T[l]

            x[l + 1, :] = self.numerical_integration(ref[l, :], u[l], x_vec, h)
            v[l] = np.random.normal(self.mu, self.sigma, 1)
            x[l + 1, 1] += v[l]

        outputs = {
            "time": T,
            "states": x,
            "reference_signal": ref,
            "error_states": e,
            "human_input": None,
            "robot_input": u,
            "human_costs": None,
            "robot_costs": J,
            "human_gain": None,
            "robot_gain": L,
            "human_Q": None,
            "robot_Q": Q,
        }

        return outputs