import numpy as np
import scipy.linalg as cp
import time

class ControllerDG_GObsKal:
    def __init__(self, A, B, Gamma, Pi, kappa, Qr, Qh):
        self.A = A
        self.B = B
        self.Gamma = Gamma
        self.Pi = Pi
        self.kappa = kappa
        self.Qr = Qr
        self.Qh = Qh
        self.Lh_hat = np.array([0.0, 0.0])

    def compute_costs(self, x, u):
        return np.matmul(np.matmul(x.transpose(), self.Qr), x) + u**2

    def compute_control_input(self, states):
        xi = states["error_state"]
        x = states["state"]
        x_hat = states["state_estimate"]
        x_tilde = x_hat - x
        x_dot = states["state_derivative"]
        Lh_hat = states["estimated_human_gain"]
        Qr = states["robot_cost"]
        uhhat = np.matmul(-Lh_hat, xi)

        # Compute Controller gain
        Acl = self.A - self.B * Lh_hat
        try:
            Pr = cp.solve_continuous_are(Acl, self.B, Qr, 1)
        except:
            print("Debugs:")
            print("Qr = ", Qr)
            print("Lhhat = ", Lh_hat)
            print("failed to find finite solution")
            return -1
            # exit("failed to find finite solution")

        Lr = np.matmul(self.B.transpose(), Pr)
        ur = np.matmul(-Lr, xi)
        Jr = self.compute_costs(xi, ur)

        x_hat_dot = np.matmul(self.A, x_hat) + self.B * (ur + uhhat + self.nonlinear_term(x)) - np.matmul(self.Pi, x_tilde)
        xi_tilde_dot = x_hat_dot - x_dot
        uh_tilde = (1 / (np.matmul(self.B.transpose(), self.B))) * np.matmul(self.B.transpose(), xi_tilde_dot - np.matmul((self.A - self.Pi), x_tilde))
        m_squared = 1 + self.kappa * np.matmul(xi.transpose(), xi)
        xi_gamma = np.matmul(xi.transpose(), self.Gamma)
        Lhhat_dot = uh_tilde * xi_gamma / m_squared

        output = {
            "torque": ur,
            "estimated_human_torque": uhhat,
            "cost": Jr,
            "state_estimate_derivative": x_hat_dot,
            "estimated_human_gain_derivative": Lhhat_dot,
            "robot_gain": Lr,
            "input_estimation_error": uh_tilde / m_squared,
            "xi_gamma": xi_gamma,
        }

        return output

    def nonlinear_term(self, x):
        g = 9.81
        m = 0.406
        dh = 0.0628
        dl = 0.00901
        vt = 0.522
        vsp = 2 * vt
        tau_d = -0.086
        tau_fric = -0.0

        # Gravity
        tau_g = - m * g * dh * np.sin(x[0, 0]) - m * g * dl * np.cos(x[0, 0])

        # Friction
        v = x[1, 0]
        gv = v / vsp * np.exp(-(v / (np.sqrt(2) * vsp)) ** 2 + 1 / 2)
        fc = tau_d * np.tanh(v / vt)
        tau_f = gv * tau_fric + fc
        return tau_f + tau_g