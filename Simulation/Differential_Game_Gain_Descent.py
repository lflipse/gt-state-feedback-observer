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
        self.nonlin = nonlin

    def numerical_integration(self, r, ur, uh, uhhat, y, h, Gamma, kappa):
        k1 = h * self.ydot(r, ur, uh, uhhat, y, Gamma, kappa)
        k2 = h * self.ydot(r, ur, uh, uhhat, y + 0.5 * k1, Gamma, kappa)
        k3 = h * self.ydot(r, ur, uh, uhhat, y + 0.5 * k2, Gamma, kappa)
        k4 = h * self.ydot(r, ur, uh, uhhat, y + k3, Gamma, kappa)

        # Update next value of y
        y_new = y + (1.0 / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        # print(k1)
        return y_new, k1[0: 2]/h

    def nonlinear_term(self, x):
        # Parameters
        g = 9.81
        m = 0.4057759271653798
        dh = 0.06279095432769782
        dl = 0.009008157082978899
        vt = 0.5217807023268454
        vsp = 2 * vt
        tau_fric = 0.0
        tau_d = -0.08597506012132082

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

    def ydot(self, r, ur, uh, uhhat, y, Gamma, kappa):
        # Unpack y vector
        x = np.array([[y[0]], [y[1]]])

        # Calculate error vector
        e = x - np.array([[r[0]], [r[1]]])

        # Calculate derivatives
        # Real response
        x_dot = np.matmul(self.A, x) + self.B * (ur + uh + self.nonlinear_term(x))

        # Estimated responses
        x_hat_dot = np.matmul(self.A, x) + self.B * (ur + uhhat + self.nonlinear_term(x))

        # Estimation error
        x_tilde_dot = x_hat_dot - x_dot

        pseudo_B = 1/(np.matmul(self.B.transpose(), self.B)) * self.B.transpose()
        u_h_tilde = np.matmul(pseudo_B, x_tilde_dot)
        m_squared = 1 + kappa * np.matmul(e.transpose(), e)
        P_hhat_vec_dot = (1/m_squared) * np.matmul(np.matmul(Gamma, e), u_h_tilde)

        ydot = np.array([x_dot.transpose(), P_hhat_vec_dot.transpose()]).flatten()

        return ydot

    def compute_gains(self, Q, Phat, bias):
        Lhat = np.matmul(self.B.transpose(), Phat) + bias
        A_c = self.A - self.B * Lhat
        P = cp.solve_continuous_are(A_c, self.B, Q, 1)
        L = np.matmul(self.B.transpose(), P)
        return Lhat, L, P

    def compute_inputs(self, Q, Phat, e, bias):
        Lhat, L, P = self.compute_gains(Q, Phat, bias)
        u = np.inner(-L, e)
        uhat = np.inner(-Lhat, e)
        return u, uhat, L, Lhat, P

    def update_costs(self, Q, Phat, bias):
        Lhat, L, P = self.compute_gains(Q, Phat, bias)
        A_c = self.A - self.B * L
        Qhat = - np.matmul(A_c.transpose(), Phat.transpose()) - np.matmul(Phat, A_c) +  np.matmul(Phat, self.B) \
               * np.matmul(self.B.transpose(), Phat.transpose())
        return Qhat

    def update_parameters(self, Q, Phat, bias):
        Lhat, L, P = self.compute_gains(Q, Phat, bias)
        gamma_1 = self.alpha_1 - self.beta ** 2 * (P[0, 1] )
        gamma_2 = self.alpha_2 - self.beta ** 2 * (P[1, 1] )
        a_hhat = - gamma_1 * Phat[1, 1] - gamma_2 * Phat[0, 1] + self.beta**2*Phat[0,1]*Phat[1,1]
        q_hhat1 = - 2 * gamma_1 * Phat[0,1] + self.beta**2*Phat[0,1]**2
        q_hhat2 = - 2 * Phat[0,1] - 2 * gamma_2 * Phat[1,1] + self.beta**2*Phat[1,1]**2
        phi = np.array([q_hhat1, q_hhat2, a_hhat])
        return phi

    def compute_costs(self, x, u, Q):
        return np.matmul(np.matmul(x, Q), x.transpose()) + u**2

    def simulate(self, inputs):
        # Time entire operation
        # Time entire operation
        print("Starting simulation")
        start = time.time()

        # Unpack dictionary
        # Unpack dictionary
        # N = inputs["simulation_steps"]
        # h = inputs["step_size"] + 0.00098
        Qh0 = inputs["human_weight"]
        Qr0 = inputs["robot_weight"]


        x0 = inputs["initial_state"]
        u0 = inputs["u_initial"]
        e0 = inputs["e_initial"]
        kappa = inputs["kappa"]
        Gamma = inputs["Gamma"]
        C = inputs["sharing_rule"]
        bias = inputs["gain_estimation_bias"]
        ref = np.array(inputs["reference"])
        T = np.array(inputs["time"])
        N = len(T)
        try:
            Lh0 = np.array(inputs["virtual_human_gain"])
        except:
            Ph = cp.solve_continuous_are(self.A, self.B, Qh0, 1)
            Lh = np.matmul(self.B.transpose(), Ph)
            Lh0 = np.tile(Lh, (1, N))

        y = np.zeros((N + 1, 4))
        ydot = np.zeros((N, 2))
        y[0, 0:2] = x0

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
        e = np.zeros((N, 2))
        xhhat = np.zeros((N, 2))
        ur = np.zeros(N)
        uhbar = np.zeros(N)
        vh = np.zeros(N)
        uh = np.zeros(N)
        Jh = np.zeros(N)
        Jhhat = np.zeros(N)
        Jr = np.zeros(N)
        e[0, :] = e0
        ur[0] = u0



        for i in range(N-1):
            # Human cost is fixed, Robot cost based on estimator
            Qr[i, :, :] = Qr0
            Qh[i, :, :] = Qh0

            # Calcuate derivative(s) of reference

            # Compute inputs
            e[i, :] = (y[i, 0:2] - ref[i, :])
            ur[i], uhhat[i], Lr[i, :], Lhhat[i, :], Pr[i, :, :] = self.compute_inputs(Qr[i, :, :], Phhat[i, :], e[i, :], bias)
            # print(Lh0[:, i].flatten())
            # print(Lh[i,:])
            Lh[i, :] = Lh0[:, i].flatten()
            uhbar[i] = np.matmul(-Lh[i, :], e[i, :])

            # uhbar[i], urhat, Lh[i, :], Lrhat, Ph[i, :, :] = self.compute_inputs(Qh[i, :, :],  Pr[i, :], e[i, :], bias)
            vh[i] = np.random.normal(self.mu, self.sigma, 1)
            uh[i] = uhbar[i] + 0*vh[i]
            # uh[i] = uhbar[i]

            h = T[i+1] - T[i]

            # Integrate a time-step
            y[i + 1, :], ydot[i, :] = self.numerical_integration(ref[i, :], ur[i], uh[i], uhhat[i], y[i, :], h, Gamma, kappa)
            y[i + 1, 1] = y[i + 1, 1] + vh[i]
            Phhat[i + 1, :, :] = np.array([[0, y[i + 1, 2]], [y[i + 1, 2], y[i + 1, 3]]])

            # Update P and Q
            phi = self.update_parameters(Qr[i, :, :], Phhat[i + 1, :, :], bias)
            Qhhat[i + 1, 0, 0] = phi[0]
            Qhhat[i + 1, 1, 1] = phi[1]
            Phhat[i + 1, 0, 0] = phi[2]

            Jh[i] = self.compute_costs(e[i, :], uh[i], Qh[i, :, :])
            Jhhat[i] = self.compute_costs(e[i, :], uhhat[i], Qhhat[i, :, :])
            Jr[i] = self.compute_costs(e[i, :], ur[i], Qr[i, :, :])

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
            "ydot": ydot,
        }

        return outputs