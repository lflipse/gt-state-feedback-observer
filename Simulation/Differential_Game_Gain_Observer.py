import numpy as np
import scipy.linalg as cp
import time

class ControllerNG:
    def __init__(self, A, B, mu, sigma, nonlin):
        self.A = A
        self.B = B
        self.mu = mu
        self.sigma = sigma
        self.beta = B[1, 0]
        self.alpha_1 = A[1, 0]
        self.alpha_2 = A[1, 1]
        self.D = -self.alpha_2/self.beta
        self.nonlin = nonlin

    def numerical_integration(self, r, ur, uh, uhhat, y, h, Gamma, K, kappa):
        k1 = h * self.ydot(r, ur, uh, uhhat, y, Gamma, K, kappa)
        k2 = h * self.ydot(r, ur, uh, uhhat, y + 0.5 * k1, Gamma, K, kappa)
        k3 = h * self.ydot(r, ur, uh, uhhat, y + 0.5 * k2, Gamma, K, kappa)
        k4 = h * self.ydot(r, ur, uh, uhhat, y + k3, Gamma, K, kappa)

        pdot = self.ydot(r, ur, uh, uhhat, y, Gamma, K, kappa)
        ydot = pdot[0:2]

        # Update next value of y
        y_new = y + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        return y_new, ydot

    def nonlinear_term(self, x):
        # Parameters
        g = 9.81
        m = 0.47426441550449755
        dh = 0.04856599995887999
        dl = 0.007921400008340136
        vt = 0.2850153620588755
        vsp = 2 * vt
        tau_d = -0.09551579889787694
        tau_fric = 0.02622587046565122

        # Gravity
        tau_g = - m * g * (dh * np.sin(x[0, 0]) + dl * np.cos(x[0, 0]))

        # Friction
        v = x[1, 0]
        gv = v / vsp * np.exp(-(v / (np.sqrt(2) * vsp)) ** 2 + (1 / 2))
        tau_f = gv * tau_fric + tau_d * np.tanh(v / vt)
        if self.nonlin:
            f_nl = tau_g + tau_f
        else:
            f_nl = 0
        return f_nl

    def ydot(self, r, ur, uh, uhhat, y, Gamma, K, kappa):
        # Unpack y vector
        x = np.array([[y[0]], [y[1]]])
        xi = x - np.array([[r[0]], [r[1]]])
        xi_hat = np.array([[y[4]], [y[5]]])
        xi_tilde = xi_hat - xi

        # Calculate derivatives
        # Real response
        if self.nonlin:
            x_dot = np.matmul(self.A, x) + self.B * (ur + uh + self.nonlinear_term(x))
            xi_dot = np.matmul(self.A, xi) + self.B * (ur + uh + self.nonlinear_term(x))
            xi_hat_dot = np.matmul(self.A, xi_hat) + self.B * (ur + uhhat + self.nonlinear_term(x)) - np.matmul(Gamma, xi_tilde)
        else:
            x_dot = np.matmul(self.A, x) + self.B * (ur + uh)
            xi_dot = np.matmul(self.A, xi) + self.B * (ur + uh)
            xi_hat_dot = np.matmul(self.A, xi_hat) + self.B * (ur + uhhat) - np.matmul(Gamma, xi_tilde)

        # Estimation error
        xi_tilde_dot = xi_hat_dot - xi_dot

        pseudo_B = 1/(np.matmul(self.B.transpose(), self.B)) * self.B.transpose()
        u_h_tilde = np.matmul(pseudo_B, xi_tilde_dot - np.matmul((self.A - Gamma), xi_tilde))
        m_squared = 1 + kappa * np.matmul(xi.transpose(), xi)
        Lhhat_dot = u_h_tilde / m_squared * np.matmul(xi.transpose(), K)
        ydot = np.array([x_dot.transpose(), Lhhat_dot, xi_hat_dot.transpose()]).flatten()

        return ydot

    def compute_gains(self, Q, Lhat):
        A_c = self.A - self.B * Lhat
        try:
            P = cp.solve_continuous_are(A_c, self.B, Q, 1)
        except:
            print(Lhat, A_c, Q)
        L = np.matmul(self.B.transpose(), P)
        return L

    def compute_inputs(self, Q, Lhat, e, bias):
        xi = np.array([[e[0]], [e[1]]])
        L = self.compute_gains(Q, Lhat)
        u = np.matmul(-L, xi)
        uhat = np.matmul(-Lhat-bias, xi)
        return u, uhat, L

    def update_cost(self, Lr, Lhat):
        p = 1/self.beta * Lhat
        gamma_1 = self.alpha_1 - self.beta * Lr[0]
        gamma_2 = self.alpha_2 - self.beta * Lr[1]
        q_hhat1 = - 2 * gamma_1 * p[0] + Lhat[0] ** 2
        q_hhat2 = - 2 * p[0] - 2 * gamma_2 * p[1] + Lhat[1] ** 2
        Q_hhat = np.array([[q_hhat1, 0], [0, q_hhat2]])
        return Q_hhat

    def compute_costs(self, x, u, Q):
        return np.matmul(np.matmul(x, Q), x.transpose()) + u**2

    def simulate(self, inputs):
        # Time entire operation
        # Time entire operation
        print("Starting simulation")
        start = time.time()

        # Unpack dictionary
        Qh0 = inputs["human_weight"]
        Lh0 = np.array(inputs["virtual_human_gain"])
        try:
            vhg1 = inputs["virtual_human_gain_pos"]
            vhg2 = inputs["virtual_human_gain_vel"]
            Qh1 = inputs["virtual_human_cost_pos"]
            Qh2 = inputs["virtual_human_cost_vel"]
        except:
            vhg1 = None
            vhg2 = None
            Qh1 = None
            Qh2 = None

        x0 = inputs["initial_state"]
        u0 = inputs["u_initial"]
        e0 = inputs["e_initial"]
        kappa = inputs["kappa"]
        Gamma = inputs["Gamma"]
        K = inputs["K"]
        C = inputs["sharing_rule"]
        bias = inputs["gain_estimation_bias"]
        ref = inputs["reference"]
        T = np.array(inputs["time"])
        N = len(T)

        y = np.zeros((N + 1, 6))
        ydot = np.zeros((N, 2))
        y[0, 0:2] = x0

        # Estimator vectors
        Qhhat = np.zeros((N + 1, 2, 2))
        Lhhat = np.zeros((N + 1, 2))
        Ph = np.zeros((N, 2, 2))
        Pr = np.zeros((N, 2, 2))
        Qh = np.zeros((N, 2, 2))
        Qr = np.zeros((N, 2, 2))
        uhhat = np.zeros(N)

        # Real vectors
        Lh = np.zeros((N, 2))
        Lr = np.zeros((N, 2))
        e = np.zeros((N, 2))
        xhat = np.zeros((N, 2))
        x = np.zeros((N + 1, 2))
        ur = np.zeros(N)
        uhbar = np.zeros(N)
        vh = np.zeros(N)
        uh = np.zeros(N)
        Jh = np.zeros(N)
        Jhhat = np.zeros(N)
        Jr = np.zeros(N)
        e[0, :] = np.array(e0)
        ur[0] = u0

        for i in range(N):
            # Human cost is fixed, Robot cost based on estimator
            Qr[i, :, :] = C #- Qhhat[i, :, :]
            try:
                Qh[i, :, :] = np.array([[Qh1[i], 0], [0, Qh2[i]]])
            except:
                Qh[i, :, :] = Qh0

            # Calculate derivative(s) of reference
            # Compute inputs
            e[i, :] = y[i, 0:2] - ref[i, :]
            ur[i], uhhat[i], Lr[i, :] = self.compute_inputs(Qr[i, :, :], Lhhat[i, :], e[i, :], bias)

            # Actual human response
            if vhg1 != None:
                try:
                    Lh[i, :] = np.array([vhg1[i], vhg2[i]])
                    uhbar[i] = np.matmul(-Lh[i, :], e[i, :])
                except:
                    Lh[i, :] = Lh0[:, i].flatten()
                    uhbar[i] = np.matmul(-Lh[i, :], e[i, :])
            else:
                try:
                    Lh[i, :] = Lh0[:, i]
                    uhbar[i] = np.matmul(-Lh[i, :], e[i, :])
                except:
                    Lh[i, :] = Lh0[:, i].flatten()
                    uhbar[i] = np.matmul(-Lh[i, :], e[i, :])

            # uhbar[i], urhat, Lh[i, :], Lrhat, Ph[i, :, :] = self.compute_inputs(Qh[i, :, :],  Pr[i, :], e[i, :], bias)
            vh[i] = np.random.normal(self.mu, self.sigma, 1)
            uh[i] = uhbar[i] + 0*vh[i]

            try:
                h = T[i+1] - T[i]
            except:
                h = T[i] - T[i-1]

            # Integrate a time-step
            y[i + 1, :], ydot[i, :] = self.numerical_integration(ref[i, :], ur[i], uh[i], uhhat[i], y[i, :], h, Gamma, K, kappa)
            y[i + 1, 1] = y[i + 1, 1] + vh[i]
            Lhhat[i + 1, :] = y[i + 1, 2:4]
            Qhhat[i + 1, :, :] = self.update_cost(Lr[i, :], Lhhat[i, :])
            xhat[i, :] = ref[i, :] + y[i, 4:6]
            x[i, :] = y[i, 0:2]

            # Update P and Q
            Jh[i] = self.compute_costs(e[i, :], uh[i], Qh[i, :, :])
            Jhhat[i] = self.compute_costs(e[i, :], uhhat[i], Qhhat[i, :, :])
            Jr[i] = self.compute_costs(e[i, :], ur[i], Qr[i, :, :])


        outputs = {
            "states": x,
            "estimated_error": y[:, 4:6],
            "estimated_states": xhat,
            "time": T,
            "reference_signal": ref,
            "error_states": y[:, 0:2],
            "human_input": uh,
            "human_estimated_input": uhhat,
            "human_estimated_Q": Qhhat,
            "human_Q": Qh,
            "robot_Q": Qr,
            "human_P": Ph,
            "robot_P": Pr,
            "human_estimated_gain": Lhhat,
            "human_gain": Lh,
            "robot_gain": Lr,
            "robot_input": ur,
            "human_costs": Jh,
            "human_estimated_costs": Jhhat,
            "robot_costs": Jr,
            "ydot": ydot,
        }

        return outputs