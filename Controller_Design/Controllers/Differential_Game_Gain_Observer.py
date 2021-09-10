import numpy as np
import scipy.linalg as cp

class ControllerDGObs:
    def __init__(self, A, B, K, Gamma, kappa):
        self.A = A
        self.B = B
        self.Gamma = Gamma
        self.K = K
        self.kappa = kappa

    def compute_control_input(self, states, manual):
        xi = states["error_state"]
        x = states["state"]
        x_hat = states["state_estimate"]
        x_tilde = x_hat - x

        x_dot = states["state_derivative"]
        Lh_hat = states["estimated_human_gain"]
        Qh = states["estimated_human_cost"]
        C = states["sharing_rule"]

        if manual:
            Qr = np.array([[0, 0], [0, 0]])
            Pr = Qr
            Lr = np.array([[0, 0]])
            ur = 0
        else:
            Qr = C - Qh
            if np.linalg.det(Qr) < 0:
                Qr = np.array([[0, 0], [0, 0]])
                Pr = np.array([[0, 0], [0, 0]])
                Lr = np.array([[0, 0]])
                print("Qr too low here")

            # Compute Controller gain
            else:
                Acl = self.A - self.B * Lh_hat
                Pr = cp.solve_continuous_are(Acl, self.B, Qr, 1)
                Lr = np.matmul(self.B.transpose(), Pr)

            ur = np.matmul(-Lr, xi)
        uhhat = np.matmul(-Lh_hat, xi)

        # Observer equations
        x_hat_dot = np.matmul(self.A, x_hat) + self.B * (ur + uhhat + self.nonlinear_term(x)) - np.matmul(self.Gamma, x_tilde)
        xi_tilde_dot = x_hat_dot - x_dot

        # Update law for human gain
        pseudo_B = 1 / np.matmul(self.B.transpose(), self.B) * self.B.transpose()
        uh_tilde = np.matmul(pseudo_B,  xi_tilde_dot - np.matmul(self.A - self.Gamma, x_tilde))
        m_squared = 1 + self.kappa * np.matmul(xi.transpose(), xi)
        Lhhat_dot = uh_tilde / m_squared * np.matmul(xi.transpose(), self.K)

        output = {
            "torque": ur,
            "estimated_human_torque": uhhat,
            "state_estimate_derivative": x_hat_dot,
            "estimated_human_gain_derivative": Lhhat_dot,
            "robot_gain": Lr,
            "robot_P": Pr,
            "input_estimation_error": uh_tilde[0, 0],
            "robot_cost": Qr,
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