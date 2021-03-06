import numpy as np
import scipy.linalg as cp

class ControllerDGObs:
    def __init__(self, A, B, K, Gamma, kappa):
        self.A = A
        self.B = B
        self.Gamma = Gamma
        self.K = K
        self.kappa = kappa

    def compute_control_input(self, states, manual, static):
        xi = states["error_state"]
        x = states["state"]
        x_hat = states["state_estimate"]
        x_tilde = x_hat - x

        x_dot = states["state_derivative"]
        Lh_hat = states["estimated_human_gain"]
        Lr_old = states["robot_gain"]
        Qh = states["estimated_human_cost"]
        C = states["sharing_rule"]


        if manual:
            Qr = np.array([[0, 0], [0, 0]])
            Pr = Qr
            Lr = np.array([[0, 0]])
            ur = 0
            beta = 0
        else:
            if static:
                Qr = C
                Pr = cp.solve_continuous_are(self.A, self.B, Qr, 1)
                Lr = np.matmul(self.B.transpose(), Pr)
                beta = 0
            else:
                Qr = C - Qh
                # print("eigenvalue: ", np.linalg.det(Qr))
                # if np.linalg.det(Qr) < -0.2:
                #     # Pr = np.array([[0, 0], [0, 0]])
                #     # Lr = Lr_old
                #     Acl = self.A - self.B * Lh_hat
                #     Pr = cp.solve_continuous_are(Acl, self.B, Qr, 1)
                #     Lr = Lr_old
                #     # Lr = max(0, Lr[0, 0])
                #     # print(Lr)

                Acl = self.A - self.B * Lh_hat
                Pr = cp.solve_continuous_are(Acl, self.B, Qr, 1)
                Lr = np.matmul(self.B.transpose(), Pr)


            ur = np.matmul(-Lr, xi)
        uhhat = np.matmul(-Lh_hat, xi)

        # Observer equations
        x_hat_dot = np.matmul(self.A, x_hat) + self.B * (ur + uhhat) - np.matmul(self.Gamma, x_tilde)
        xi_tilde_dot = x_hat_dot - x_dot

        # Forgetting factor
        try:
            Lh_pos = Lh_hat[0][0]
        except:
            Lh_pos = Lh_hat[0]
        if Lh_pos < 0:
            beta = 0.04
        else:
            beta = 0.00
        forget_factor = beta * np.array([[1, 1]])

        # Update law for human gain
        pseudo_B = 1 / np.matmul(self.B.transpose(), self.B) * self.B.transpose()
        uh_tilde = np.matmul(pseudo_B,  xi_tilde_dot - np.matmul(self.A - self.Gamma, x_tilde))
        m_squared = 1 + self.kappa * np.matmul(xi.transpose(), xi)
        Lhhat_dot = np.matmul(uh_tilde / m_squared * xi.transpose() + forget_factor, self.K)
        # print(Lhhat_dot, forget_factor, Lhhat_dot + forget_factor)

        ur_comp = ur - self.nonlinear_term(x)

        output = {
            "nonlins": self.nonlinear_term(x),
            "torque": ur_comp,
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
