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
        print("real robot gain: ", self.Lr)

    def compute_costs(self, x, u):
        return np.matmul(np.matmul(x.transpose(), self.Qr), x) + u**2

    def compute_control_input(self, states, condition):
        # print("xi is", xi)
        xi = states["error_state"]
        x = states["state"]
        ur = - np.matmul(self.Lr, xi)
        ur_comp = ur - self.nonlinear_term(x)
        Jr = self.compute_costs(xi, ur)
        output = {
            "output_torque": ur_comp,
            "real_torque": ur,
            "cost": Jr
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
