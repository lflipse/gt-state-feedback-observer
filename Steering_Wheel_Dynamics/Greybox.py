import numpy as np
import multiprocessing as mp
import time
import os
import matplotlib.pyplot as plt
import platform
import wres
import scipy.optimize as cp
import csv
import pandas as pd


import sys
# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '..')

from Steering_Wheel_Dynamics.SensoDriveTest import SensoDriveModule

class GreyBox:
    def __init__(self, parent_conn, child_conn, senso_drive_process):
        # Initial values
        self.Bw = 0
        self.Kw = 0

        self.n = 1  # No. settings
        self.duration = 10  # Seconds per experiment

        self.parent_conn = parent_conn
        self.child_conn = child_conn
        self.senso_drive_process = senso_drive_process

        self.send_dict = {}
        self.saved = {}
        self.saved_states = {}
        self.data_set = {}
        self.x = np.array([0, 0])
        self.phi_sim_init = None
        self.phidot_sim_init = None
        self.phi_sim_final = None
        self.phidot_sim_final = None

        self.file_csv = "file.csv"

    def do(self):
        # Check whether to create or load dataset
        if self.senso_drive_process is not None:
            self.generate_data_set()
        self.load_data_set()
        self.optimize()
        self.plot_data()

    def generate_data_set(self):
        for i in range(self.n):
            self.Bw = 0.2
            self.Kw = 0.5
            self.send_dict["stiffness"] = self.Kw
            self.send_dict["damping"] = self.Bw
            time.sleep(1.0)
            t0 = time.time()
            t_last = time.time()
            while time.time() - t0 < self.duration:
                self.send_dict["torque"] = self.compute_torque(t_last - t0)
                self.send_dict["exit"] = False
                self.send_dict["stiffness"] = self.Kw
                self.send_dict["damping"] = self.Bw

                if senso_drive_process.is_alive():
                    parent_conn.send(self.send_dict)  # Child is for sending
                    new_states = parent_conn.recv()  # Receive from child
                else:
                    exit("sensodrive process was killed")

                if new_states == None:
                    print("Double check with error")
                else:
                    states = new_states

                t_now = time.time()
                dt = t_now - t_last
                t_last = t_now

                if t_now - t0 > 0.4:

                    output = {
                        "steering_angle": states["steering_angle"],
                        "steering_rate": states["steering_rate"],
                        "torque": states["torque"],
                        "time": t_last - t0,
                        "execution_time": dt,
                        "stiffness": self.Kw,
                        "damping" : self.Bw,
                    }

                    self.store_variables(output)

        print("finished experiment")
        self.send_dict["exit"] = True
        if self.senso_drive_process.is_alive():
            self.parent_conn.send(self.send_dict)  # Child is for sending
        else:
            exit("sensodrive process was killed")

        self.to_csv()

    def load_data_set(self):
        df = pd.read_csv(self.file_csv, index_col=0)
        self.data_set = df.to_dict(orient='list')

    def optimize(self):
        # Initial guess
        J = 0.0447
        m = 0.5
        dh = 0.0
        dl = 0.0
        # tau_fric = -0.2
        # tau_kin = 0.1
        vt = 1
        tau_f = 0.0
        tau_d = 0.0

        p0 = np.array([J, m, dh, dl, vt, tau_f, tau_d])
        bounds_vec = np.array([(0.001, 0.07), (0.2, 0.8), (-0.15, 0.15), (-0.15, 0.15), (0.05, 3),
                               (-0.4, 0.0), (-0.5, 0.0)])

        self.simulate_experiment(p0)
        self.phi_sim_init = self.saved_states["steering_angle"]
        self.phidot_sim_init = self.saved_states["steering_rate"]

        # Optimize
        p = cp.minimize(self.fun, p0, method='SLSQP', bounds=bounds_vec)
        print("Results: ", p)
        p_opt = np.array(p["x"])

        self.show_nonlins(p_opt[5], p_opt[6], p_opt[4])

        self.simulate_experiment(p_opt)
        self.phi_sim_final = self.saved_states["steering_angle"]
        self.phidot_sim_final = self.saved_states["steering_rate"]

    def plot_data(self):
        csfont = {'fontname': 'Georgia'}
        hfont = {'fontname': 'Georgia'}

        # Colors
        tud_blue = "#0066A2"
        tud_black = "#000000"
        tud_grey = "#808080"
        tud_red = "#c3312f"
        tud_green = "#00A390"
        tud_yellow = "#F1BE3E"

        print("plotting stuff")
        t = self.data_set["time"]
        t_ex = self.data_set["execution_time"]
        u = self.data_set["torque"]
        phi = self.data_set["steering_angle"]
        phidot = self.data_set["steering_rate"]
        phi_sim = self.phi_sim_init
        phidot_sim = self.phidot_sim_init
        phi_sim = self.phi_sim_init
        phidot_sim = self.phidot_sim_init

        plt.figure()
        plt.plot(t, t_ex)

        plt.figure()
        plt.plot(t, u)

        plt.figure()
        plt.plot(t, phi, tud_blue, linewidth=2.5, label="Measured $\phi(t)$")
        plt.plot(t, self.phi_sim_init, tud_red, linewidth=2.5, label="Initial $\hat{\phi}(t)$")
        plt.plot(t, self.phi_sim_final, tud_green, linewidth=2.5, label="Final $\hat{\phi}(t)$")
        plt.title("Steering angle comparison", **csfont)
        plt.xlabel("Time (s)", **hfont)
        plt.ylabel("Steering angle (rad)", **hfont)
        plt.legend()

        plt.figure()
        plt.plot(t, phidot, tud_blue, linewidth=2.5, label="Measured $\dot{\phi}(t)$")
        plt.plot(t, self.phidot_sim_init, tud_red, linewidth=2.5, label="Initial $\dot{\hat{\phi}}(t)$")
        plt.plot(t, self.phidot_sim_final, tud_green, linewidth=2.5, label="Final $\dot{\hat{\phi}}(t)$")
        plt.title("Steering rate comparison", **csfont)
        plt.xlabel("Time (s)", **hfont)
        plt.ylabel("Steering rate (rad/s)", **hfont)
        plt.legend()

        plt.show()

    def to_csv(self):
        columns = []
        for key in self.saved.keys():
            columns.append(key)
        df = pd.DataFrame(data=self.saved)
        df.to_csv(self.file_csv)

    def compute_torque(self, t):
        fs = 4
        torque = 0.7 * np.sin(2 * np.pi * t / fs) + 0.4 * np.cos(2 * np.pi * t / (3.7*fs))
        return torque

    def fun(self, p):
        self.simulate_experiment(p)
        phi = self.data_set["steering_angle"]
        phi_sim = self.saved_states["steering_angle"]
        phidot = self.data_set["steering_rate"]
        phidot_sim = self.saved_states["steering_rate"]
        dphi = np.array(phi) - np.array(phi_sim)
        dphidot = np.array(phidot) - np.array(phidot_sim)
        cost = 3*np.inner(dphi, dphi) + np.inner(dphidot, dphidot)
        return cost

    def show_nonlins(self, tau_f, tau_d, vt):
        # Styles
        csfont = {'fontname': 'Georgia'}
        hfont = {'fontname': 'Georgia'}

        # Colors
        tud_blue = "#0066A2"
        tud_black = "#000000"
        tud_grey = "#808080"
        tud_red = "#c3312f"
        tud_green = "#00A390"
        tud_yellow = "#F1BE3E"
        colors = [tud_blue, tud_black, tud_red, tud_green, tud_yellow]

        # Simulations
        # gamma = [0.1, 0.5, 1, 5]
        gamma = 1

        n = 1

        plt.figure()
        labels = []
        vsp = 2 * vt
        vs = 3 * vsp
        v = np.linspace(-1.5*vs, 1.8*vs, 400)

        string = "$\gamma$ = " + str(gamma)
        labels.append(string)
        gv = v/vsp * np.exp(-(v/(np.sqrt(2) * vsp))**2 + 1/2)
        fc = tau_d * np.tanh(v/vt)
        f_fric = gv * tau_f + fc
        plt.plot(v, gv * tau_f, colors[0], linewidth=2)
        plt.plot(v, fc, colors[1], linewidth=2)
        plt.plot(v, f_fric, colors[2], linewidth=2)

        plt.xlabel("Steering rate $\dot{\phi}}(t)$ (rad/s)", **csfont)
        plt.ylabel("Friction torque $f_{fric}(t)$ (Nm)", **csfont)
        plt.title("The effect of parameter $\gamma$", **csfont)
        plt.xlim(v[0], v[-1])
        plt.tight_layout(pad=1)

        phi = np.linspace(-np.pi, np.pi, 200)
        tau_g = 0.3 * 9.81 * 0.05 * np.sin(phi)

        plt.figure()
        fac = 360 / (2 * np.pi)

        plt.plot(fac * phi, tau_g, colors[0], linewidth=2)

        plt.xlabel("Steering angle $\phi}(t)$ (degrees)", **csfont)
        plt.ylabel("Gravitational torque $f_{g}(t)$ (Nm)", **csfont)
        plt.title("Gravitational torque", **csfont)
        plt.xlim(fac * phi[0], fac * phi[-1])
        plt.tight_layout(pad=1)
        # plt.legend()

    def store_variables(self, output):
        for key in output.keys():
            self.saved.setdefault(key, []).append(output[key])

    def save_states(self, save_state):
        for key in save_state.keys():
            self.saved_states.setdefault(key, []).append(save_state[key])

    def dynamics(self, x, u, p, b, k):
        g = 9.81
        J = p[0]
        m = p[1]
        dh = p[2]
        dl = p[3]
        vt = p[4]
        vsp = 3*vt
        tau_d = p[5]
        tau_fric = p[6]
        tau_kin = 0

        # Gravity
        tau_g = - m * g * dh * np.sin(x[0]) - m * g * dl * np.cos(x[0])

        # Friction
        v = x[1]
        gv = v / vsp * np.exp(-(v / (np.sqrt(2) * vsp)) ** 2 + 1 / 2)
        fc = tau_d * np.tanh(v / vt)
        tau_f = gv * tau_fric + fc

        # Dynamic equations
        phidot = x[1]
        phiddot = 1/J * (-(b+tau_kin) * x[1] - k * x[0] + u + tau_g + tau_f)
        xdot = np.array([phidot, phiddot])
        return xdot

    def simulate_experiment(self, p):
        t_ex = self.data_set["execution_time"]
        u = self.data_set["torque"]
        b = self.data_set["damping"]
        k = self.data_set["stiffness"]
        N = len(t_ex)
        self.saved_states = {}

        self.x = np.array([self.data_set["steering_angle"][0], self.data_set["steering_rate"][0]])
        save_state = {"steering_angle": self.x[0], "steering_rate": self.x[1]}
        self.save_states(save_state)

        for i in range(N-1):
            h = t_ex[i]
            xdot = self.dynamics(self.x, u[i], p, b[i], k[i])

            x_new = h * xdot + self.x
            save_state = {"steering_angle": x_new[0], "steering_rate": x_new[1]}
            self.save_states(save_state)
            self.x = x_new


if __name__ == "__main__":

    if platform.system() == 'Windows':
        with wres.set_resolution(10000):

            set_data = input("Generate data set? 0: No, 1: Yes.    Your answer = ")

            # Activate sensodrive
            if int(set_data) == 1:
                print("starting sensodrive")
                parent_conn, child_conn = mp.Pipe(True)
                senso_drive_process = SensoDriveModule(parent_conn, child_conn)
                senso_drive_process.start()
            elif int(set_data) == 0:
                parent_conn, child_conn, senso_drive_process = None, None, None
            else:
                exit("That is no option")
            greybox = GreyBox(parent_conn, child_conn, senso_drive_process)
            greybox.do()

