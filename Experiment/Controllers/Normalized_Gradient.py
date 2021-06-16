import numpy as np
import scipy.linalg as cp
import time

class ControllerNG:
    def __init__(self, A, B, mu, sigma, kappa, Gamma):
        self.A = A
        self.B = B
        self.mu = mu
        self.sigma = sigma
        self.beta = B[1, 0]
        self.alpha_1 = A[1, 0]
        self.alpha_2 = A[1, 1]
        self.kappa = kappa
        self.Gamma = Gamma

    def numerical_integration(self, y, x, xdot, e, ur, uhhat, h):
        y_dot = h * self.ydot(x, xdot, e, ur, uhhat)

        # Update next value of y
        y_new = y + h * y_dot
        return y_new

    def ydot(self, x, xdot, e, ur, uhhat):
        # Estimated responses
        x_vec = np.array([[x[0]], [x[1]]])
        e_vec = np.array([[e[0]], [e[1]]])
        x_hat_dot = np.matmul(self.A, x_vec) + self.B * (ur + uhhat)
        # Estimation error
        x_tilde_dot = x_hat_dot - np.array([[xdot[0]], [xdot[1]]])

        # Compute P vector
        u_h_tilde = np.matmul(1/(np.matmul(self.B.transpose(), self.B)) * self.B.transpose(), x_tilde_dot)
        print(x_tilde_dot[1])
        m_squared = 1 + self.kappa * np.matmul(e_vec.transpose(), e_vec)
        P_hhat_vec_dot = (1/(m_squared*self.beta)) * np.matmul(np.matmul(self.Gamma, e_vec), u_h_tilde)

        return np.array([P_hhat_vec_dot.transpose(), x_hat_dot.transpose()]).flatten()

    def compute_gains(self, Qr, Phhat):
        Lhhat = np.matmul(self.B.transpose(), Phhat)
        A_c = self.A - self.B * Lhhat
        # Pr = cp.solve_continuous_are(A_c, self.B, Qr, 1)
        Pr = np.array([[0, 0], [0, 0]])
        Lr = np.matmul(self.B.transpose(), Pr)
        return Lhhat, Lr, Pr

    def compute_inputs(self, Qr, Phhat, e):
        Lhhat, Lr, Pr = self.compute_gains(Qr, Phhat)
        ur = np.inner(-Lr, e)
        uhhat = np.inner(-Lhhat, e)
        return ur, uhhat, Lr, Lhhat, Pr

    def update_parameters(self, Qr, Phhat):
        Lhat, Lr, Pr = self.compute_gains(Qr, Phhat)
        gamma_1 = self.alpha_1 - self.beta ** 2 * (Pr[0, 1])
        gamma_2 = self.alpha_2 - self.beta ** 2 * (Pr[1, 1])
        a_hhat = - gamma_1 * Phhat[1, 1] - gamma_2 * Phhat[0, 1] + self.beta**2*Phhat[0, 1]*Phhat[1, 1]
        q_hhat1 = - 2 * gamma_1 * Phhat[0, 1] + self.beta**2*Phhat[0, 1]**2
        q_hhat2 = - 2 * Phhat[0, 1] - 2 * gamma_2 * Phhat[1, 1] + self.beta**2*Phhat[1, 1]**2
        phi = np.array([q_hhat1, q_hhat2, a_hhat])
        return phi

    def update_gain(self, x, y, xdot, e, Phhat, Qhhat, C, h):
        Qr = C
        ur, uhhat, Lr, Lhhat, Pr = self.compute_inputs(Qr, Phhat, e)

        # Integrate a time-step
        y_new = self.numerical_integration(y, x, xdot, e, ur, uhhat, h)
        p_new = y_new[0:2]
        x_hat = y_new[2:4]
        Phhat_new = np.array([[0, p_new[0]], [p_new[0], p_new[1]]])

        # Update P and Q
        phi = self.update_parameters(Qr, Phhat)
        Qhhat_new = np.array([[phi[0], 0], [0, phi[1]]])
        Phhat_new[0, 0] = phi[2]

        # Compute new input
        Qr_new = C
        ur_new, uhhat_new, Lr, Lhhat, Pr_new = self.compute_inputs(Qr_new, Phhat_new, e)

        return ur, uhhat, y_new, Pr_new, Phhat_new, Qhhat_new, x_hat