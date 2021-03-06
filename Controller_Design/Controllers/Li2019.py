import numpy as np
import scipy.linalg as cp
import time

class ControllerDG_Li:
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
        uhhat = np.matmul(-Lh_hat, xi)

        # Compute Controller gain
        Acl = self.A - self.B * Lh_hat
        try:
            Pr = cp.solve_continuous_are(Acl, self.B, self.Qr, 1)
        except:
            print("Debugs:")
            print("Lhhat = ", Lh_hat)
            exit("failed to find finite solution")

        Lr = np.matmul(self.B.transpose(), Pr)
        ur = np.matmul(-Lr, xi)
        Jr = self.compute_costs(xi, ur)

        # Update estimated controller gains
        x_hat_dot = np.matmul(self.A, x_hat) + self.B * (ur + uhhat + self.nonlinear_term(x)) - np.matmul(self.Pi, x_tilde)
        Lhhat_dot = np.matmul(np.matmul(self.B.transpose(), 1 * self.Gamma), (x_tilde)) * xi.transpose()

        output = {
            "torque": ur,
            "estimated_human_torque": uhhat,
            "cost": Jr,
            "state_estimate_derivative": x_hat_dot,
            "estimated_human_gain_derivative": Lhhat_dot,
            "robot_gain": Lr,
            "input_estimation_error": np.array([[0]]),
            "xi_gamma": np.array([[0, 0]])
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