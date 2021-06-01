import numpy as np
import scipy.linalg as cp
import time

class ControllerDG_GObs:
    def __init__(self, A, B, Gamma, kappa, Qr, Qh):
        self.A = A
        self.B = B
        self.Gamma = Gamma
        self.kappa = kappa
        self.Qr = Qr
        self.Qh = Qh
        self.Lh_hat = np.array([0.0, 0.0])

    def compute_costs(self, x, u):
        return np.matmul(np.matmul(x.transpose(), self.Qr), x) + u**2

    def compute_control_input(self, xi, x, x_dot, Lhhat, time_step):
        xi_vec = xi
        x_vec = np.array([[x[0]], [x[1]]])
        x_dot_vec = np.array([[x[1]], [x_dot]])
        uhhat = np.matmul(-Lhhat, xi_vec)

        # Compute Controller gain
        Acl = self.A - self.B * Lhhat
        try:
            Pr = cp.solve_continuous_are(Acl, self.B, self.Qr, 1)
        except:
            print("Debugs:")
            print("Lhhat = ", Lhhat)
            exit("failed to find finite solution")
        Lr = np.matmul(self.B.transpose(), Pr)
        ur = np.matmul(-Lr, xi)

        # Estimated system response
        x_hat_dot = np.matmul(self.A, x_vec) + self.B * (ur + uhhat)
        xi_tilde_dot = x_hat_dot - x_dot_vec
        uh_tilde = 1/(np.matmul(self.B.transpose(), self.B)) * np.matmul(self.B.transpose(), xi_tilde_dot)
        m_squared = 1 + self.kappa * np.matmul(xi_vec.transpose(), xi_vec)
        Lhhat_dot = uh_tilde * np.matmul(xi_vec.transpose(), self.Gamma) / m_squared

        # Integrate to update value
        Lhhat_new = Lhhat + Lhhat_dot * time_step

        Jr = self.compute_costs(xi, ur)
        return ur.flatten(), uhhat, Lr, Lhhat_new, uh_tilde, Jr