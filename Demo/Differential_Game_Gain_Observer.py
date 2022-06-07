import numpy as np
import scipy.linalg as cp

class ControllerDGObs:
    def __init__(self, A, B, K, Gamma, kappa):
        self.A = A
        self.B = B
        self.Gamma = Gamma
        self.K = K
        self.kappa = kappa

    def compute_control_input(self, states, condition):
        # print(states)
        xi = states["error_state"]
        x = states["state"]
        x_dot = states["state_derivative"]
        xi_hat = states["estimated_error_state"]
        x_hat = states["estimated_state"]
        xi_tilde = xi_hat - xi
        xi_dot = states["error_state_derivative"]
        Lh_hat = states["estimated_human_gain"]
        Qh = states["estimated_human_cost"]
        C = states["sharing_rule"]

        if condition == "Manual Control":
            Qr = np.array([[0, 0], [0, 0]])
            Pr = Qr
            Lr = np.array([[0, 0]])
            ur = np.array([[0.0]])
            beta = 0
        else:
            alpha = np.array([[0.02, 0], [0, 1.0]])
            gamma = np.array([[1.0, 0], [0, 0.0]])
            zeta = np.array([[1.0, 0], [0, 0.0]])
            Qr1 = np.matmul(alpha, C) + np.matmul(gamma, Qh)
            Qr2 = C - np.matmul(zeta, Qh)
            if condition == "Positive Reinforcement":
                Qr = Qr1
                Qr[0, 0] = max(Qr[0, 0], 0)
            elif condition == "Negative Reinforcement":
                Qr = Qr2
                Qr[0, 0] = min(max(Qr[0, 0], 0), C[0, 0])
            else:
                Qr = C

            # Qr[0, 0] = max(Qr[0, 0], 0.1)
            Acl = self.A - self.B * Lh_hat

            try:
                Qr[0, 0] = max(Qr[0, 0], 0)
                Pr = cp.solve_continuous_are(Acl, self.B, Qr, 1)
                Lr = np.matmul(self.B.transpose(), Pr)
            except:
                Pr = np.array([[0, 0], [0, 0]])
                Lr = np.array([[0, 0]])
                print("Qr = ", Qr)
                print("Acl = ", Acl)
                exit()

            ur = np.matmul(-Lr, xi)
        uhhat = np.matmul(-Lh_hat, xi)

        # Forgetting factor
        try:
            Lh_pos = Lh_hat[0][0]
        except:
            Lh_pos = Lh_hat[0]
        if Lh_pos < 0:
            # beta = 0.0
            beta = 0.2
        else:
            beta = 0.00
        forget_factor = beta * np.array([[1, 1]])

        # Observer equations
        xi_hat_dot = np.matmul(self.A, xi_hat) + self.B * (ur + uhhat) - np.matmul(self.Gamma, xi_tilde)
        x_hat_dot = np.matmul(self.A, x_hat) + self.B * (ur + uhhat) - np.matmul(self.Gamma, xi_tilde)
        xi_tilde_dot = x_hat_dot - x_dot

        # Update law for human gain
        pseudo_B = 1 / (np.matmul(self.B.transpose(), self.B)) * self.B.transpose()
        u_h_tilde = np.matmul(pseudo_B, (xi_tilde_dot - (np.matmul((self.A - self.Gamma), xi_tilde))))
        m_squared = 1 + self.kappa * np.matmul(xi.transpose(), xi)
        Lhhat_dot = u_h_tilde / m_squared * np.matmul(xi.transpose(), self.K)

        ur_comp = ur - self.nonlinear_term(x)
        uh_meas = np.matmul(pseudo_B, xi_dot - np.matmul(self.A, xi) - np.matmul(self.B, ur))

        output = {
            "nonlins": self.nonlinear_term(x),
            "output_torque": ur_comp,
            "real_torque": ur,
            "estimated_human_torque": uhhat,
            "error_estimate_derivative": xi_hat_dot,
            "estimated_human_gain_derivative": Lhhat_dot,
            "robot_gain": Lr,
            "robot_P": Pr,
            "input_estimation_error": u_h_tilde[0, 0],
            "measured_human_input": uh_meas,
            "robot_cost": Qr,
        }

        return output

    def nonlinear_term(self, x):
        g = 9.81
        m = 0.47426441550449755
        dh = 0.04856599995887999
        dl = 0.007921400008340136
        vt = 0.2850153620588755
        vsp = 2 * vt
        tau_d = -0.09551579889787694
        tau_fric = 0.02622587046565122

        # Gravity
        tau_g = - m * g * dh * np.sin(x[0, 0]) - m * g * dl * np.cos(x[0, 0])

        # Friction
        v = x[1, 0]
        gv = v / vsp * np.exp(-(v / (np.sqrt(2) * vsp)) ** 2 + 1 / 2)
        fc = tau_d * np.tanh(v / vt)
        tau_f = gv * tau_fric + fc
        return tau_f + tau_g
