import numpy as np
import scipy.optimize as cp

class HumanEstimator:
    def __init__(self, Jw, Bw, Kw):
        self.J = Jw
        self.B = Bw
        self.K = Kw
        self.phi = None
        self.phi_dot = None
        self.xi = None
        self.xidot = None

    def estimate(self, data):
        time = data["time"]
        self.phi = data["steering_angle"]
        self.phidot = data["steering_rate"]
        self.xi = data["angle_error"]
        self.xidot = data["rate_error"]
        self.h = data["execution_time"]
        self.ref = np.array(data["reference"])
        print(self.ref.shape)

        # Set-up estimation scheme
        gains0 = np.array([0, 0])

        # Optimize
        options = {
            'maxiter': 10000,
            'maxcor': 20,
            'ftol': 1e-10,
            'gtol': 1e-7,
            'eps': 1e-9,
        }
        gains_cs = cp.minimize(self.fun, gains0, method='L-BFGS-B', options=options)
        print("Results: ", gains_cs)

        estimated_gains = np.array(gains_cs["x"])

        return estimated_gains

    def simulate(self, gains):
        phi_hat = np.zeros(len(self.phi))
        phidot_hat = phi_hat
        phi_hat[0] = self.phi[0]
        phidot_hat[0] = self.phidot[0]

        for i in range(len(self.phi)-1):
            try:
                ref = self.ref[i, :]
            except:
                print(self.ref.shape)
            phi_hat[i + 1] = phi_hat[i] + self.h[i] * phidot_hat[i]
            phidot_hat[i + 1] = phidot_hat[i] + self.h[i] * self.derivative(phi_hat[i], phidot_hat[i], gains, ref)

        return phi_hat, phidot_hat

    def derivative(self, phi, phidot, gains, ref):
        xi = np.array([[phi - ref[0]], [phidot - ref[1]]])
        uh = - np.matmul(gains, xi)
        phiddot = (1 / self.J) * (-self.K * phi - self.B * phidot + uh + self.nonlinearities(phi, phidot))
        return phiddot

    def nonlinearities(self, phi, phidot):
        f_nl = 0
        return f_nl

    def fun(self, gains):
        phi_hat, phidot_hat = self.simulate(gains)
        dphi = phi_hat - self.phi
        dphidot = phidot_hat - self.phidot
        return np.inner(dphi, dphi) + np.inner(dphidot, dphidot)

