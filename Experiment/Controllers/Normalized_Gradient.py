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
        x_hat_dot = np.matmul(self.A, x) + self.B * (ur + uhhat)

        # Estimation error
        x_tilde_dot = x_hat_dot - xdot

        # Compute P vector
        u_h_tilde = np.matmul(1/(np.matmul(self.B.transpose(), self.B)) * self.B.transpose(), x_tilde_dot)
        m_squared = 1 + self.kappa * np.matmul(e.transpose(), e)
        P_hhat_vec_dot = (1/(m_squared*self.beta)) * np.matmul(np.matmul(self.Gamma, e), u_h_tilde)

        return P_hhat_vec_dot.transpose()

    def compute_gains(self, Qr, Phhat):
        Lhhat = np.matmul(self.B.transpose(), Phhat)
        A_c = self.A - self.B * Lhhat
        P = cp.solve_continuous_are(A_c, self.B, Qr, 1)
        L = np.matmul(self.B.transpose(), P)
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

    def simulate(self, x, y, xdot, e, Phhat, Qhhat, C, h):
        Qr = C
        ur, uhhat, Lr, Lhhat, Pr = self.compute_inputs(Qr, Phhat, e)

        # Integrate a time-step
        y_new = self.numerical_integration(y, x, xdot, e, ur, uhhat, h)
        Phhat_new = np.array([[0, y_new[0]], [y_new[0], y_new[1]]])

        # Update P and Q
        phi = self.update_parameters(Qr, Phhat)
        Qhhat_new = np.array([[phi[0], 0], [0, phi[1]]])
        Phhat[0, 0] = phi[2]

        # Compute new input
        Qr_new = C
        ur, uhhat, Lr, Lhhat, Pr = self.compute_inputs(Qr_new, Phhat_new, e)

        # Human cost is fixed, Robot cost based on estimator
        Qr_new = C
        Qh[i, :, :] = Qh0

        # Calcuate derivative(s) of reference
        if i > 0:
            ref[i, :] = np.array([r[i], (r[i] - r[i - 1]) / h])
        else:
            ref[i, :] = np.array([r[i], (r[i]) / h])

        # Compute inputs
        e[i, :] = (y[i, 0:2] - ref[i, :])
        ur[i], uhhat[i], Lr[i, :], Lhhat[i, :], Pr[i, :, :] = self.compute_inputs(Qr[i, :, :], Phhat[i, :], e[i, :], bias)
        uhbar[i], urhat, Lh[i, :], Lrhat, Ph[i, :, :] = self.compute_inputs(Qh[i, :, :],  Pr[i, :], e[i, :], bias)
        vh = np.random.normal(self.mu, self.sigma, 1)
        uh_new = uhbar_new + vh



        return uh