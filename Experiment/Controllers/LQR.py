import numpy as np
import scipy.linalg as cp
import time

class ControllerLQ:
    def __init__(self, A, B, Qr):
        self.A = A
        self.B = B
        self.Qr = Qr
        self.Pr = cp.solve_continuous_are(A, B, Qr, 1)
        self.Lr = np.matmul(B.transpose(), self.Pr)

    def compute_costs(self, x, u):
        return np.matmul(np.matmul(x.transpose(), self.Qr), x) + u**2

    def compute_control_input(self, xi):
        ur = - np.matmul(self.Lr, xi)
        # print("L, xi, ur: ", self.Lr, xi, ur)
        Jr = self.compute_costs(xi, ur)
        return ur.flatten(), Jr