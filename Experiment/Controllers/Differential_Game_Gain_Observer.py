import numpy as np
import scipy.linalg as cp
import time

class ControllerDG:
    def __init__(self, A, B, Qr, Qh):
        self.A = A
        self.B = B
        self.Qr = Qr
        self.Qh = Qh
        self.Pr, self.Ph = self.solve_coupled_riccati(10)
        self.Lr = - np.matmul(B.transpose(), self.Pr)

    def solve_coupled_riccati(self, it):
        Qr = self.Qr
        Qh = self.Qh
        S = self.B * self.B.transpose()
        P_it = np.zeros((it + 1, 2, 2))
        Ph_it = np.zeros((it + 1, 2, 2))
        P_it[0, :, :] = cp.solve_continuous_are(self.A, self.B, Qr, 1)
        Ph_it[0, :, :] = cp.solve_continuous_are(self.A, self.B, Qh, 1)

        for i in range(it):
            Acl = self.A - np.matmul(S, (Ph_it[i, :, :]))
            Aclh = self.A - np.matmul(S, (P_it[i, :, :]))
            P_it[i + 1, :, :] = cp.solve_continuous_are(Acl, self.B, Qr, 1)
            Ph_it[i + 1, :, :] = cp.solve_continuous_are(Aclh, self.B, Qh, 1)

        Ph = Ph_it[it, :, :]
        P = P_it[it, :, :]
        return P, Ph

    def compute_costs(self, x, u):
        return np.matmul(np.matmul(x.transpose(), self.Qr), x) + u**2

    def compute_control_input(self, x, xi):
        ur = np.matmul(self.Lr, xi)
        Jr = self.compute_costs(x, ur)
        return ur, Jr