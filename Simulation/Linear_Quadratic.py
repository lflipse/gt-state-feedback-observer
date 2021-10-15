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

    def numerical_integration(self, r, ur, uh, y, h):
        k1 = h * self.ydot(r, ur, uh, y)
        k2 = h * self.ydot(r, ur, uh, y + 0.5 * k1)
        k3 = h * self.ydot(r, ur, uh, y + 0.5 * k2)
        k4 = h * self.ydot(r, ur, uh, y + k3)

        # Update next value of y
        y_new = y + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        #
        # y_new = y + h * self.ydot(r, ur, uh, y)

        return y_new.flatten()


    def ydot(self, r, u, uh, y):
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
            xdot = np.matmul(self.A, x) + self.B * (u + uh + tau_f + tau_g)
        else:
            xdot = np.matmul(self.A, x) + self.B * (u + uh)
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
        Qh0 = inputs["human_weight"]
        Qr0 = inputs["robot_weight"]
        x0 = inputs["initial_state"]
        u0 = inputs["u_initial"]
        e0 = inputs["e_initial"]
        ref = np.array(inputs["reference"])
        T = np.array(inputs["time"])
        N = len(T)

        Pr = cp.solve_continuous_are(self.A, self.B, Qr0, 1)
        Ph = cp.solve_continuous_are(self.A, self.B, Qh0, 1)

        print("human and robot cost matrix: ", Qh0, Qr0)

        # print("P matrices are computed as: P_r = ", Pr, " and P_h = ", Ph)
        try:
            Lh0 = inputs["virtual_human_gain"]
        except:
            Lh0 = np.matmul(self.B.transpose(), Ph)
        Lr0 = np.matmul(self.B.transpose(), Pr)
        print(Lh0)

        print("Controller gains then are computed as: L_r = ", Lr0, " and L_h = ", Lh0)

        x = np.zeros((N + 1, 2))
        x[0, :] = x0
        # ref = np.zeros((N, 2))
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


        for l in range(N-1):
            # Derive inputs
            Qr[l, :, :] = Qr0
            Qh[l, :, :] = Qh0

            Lr[l, :] = Lr0
            e[l, :] = x[l, :] - ref[l, :]

            try:
                # print(Lh0[l, :])
                Lh[l, :] = Lh0
                uh[l] = np.matmul(-Lh[l, :], e[l, :])
            except:
                Lh[l, :] = Lh0[:, l].flatten()
                uh[l] = np.matmul(-Lh[l, :], e[l, :])


            ur[l] = np.matmul(-Lr0, e[l, :])
            Jh[l] = self.compute_costs(e[l, :], uh[l], Qh0)
            Jr[l] = self.compute_costs(e[l, :], ur[l], Qr0)

            x_vec = np.array([[x[l, 0]], [x[l, 1]]])

            h = T[l+1] - T[l]

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