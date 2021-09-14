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
import seaborn as sns
from matplotlib.backends.backend_pdf import PdfPages


import sys
# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '..')

from Steering_Wheel_Dynamics.SensoDriveTest import SensoDriveModule

class GreyBox:
    def __init__(self, parent_conn, child_conn, senso_drive_process, identification):
        # Initial values
        self.Bw = 0
        self.Kw = 0

        self.n = 3  # No. settings
        self.duration = 40  # Seconds per experiment
        self.identification = identification

        self.parent_conn = parent_conn
        self.child_conn = child_conn
        self.senso_drive_process = senso_drive_process

        self.send_dict = {}
        self.saved = {}
        self.saved_states = {}
        self.data_set_ID = {}
        self.data_set_ver = {}
        self.VAF = {}
        self.x = np.array([0, 0])
        self.phi_sim_init = None
        self.phidot_sim_init = None
        self.phi_sim_final = None
        self.phidot_sim_final = None
        self.phi_sim_lin = None
        self.phidot_sim_lin = None
        self.objs = None
        self.VAF = None
        self.abs = None
        self.angle_difference= None
        self.rate_difference = None
        self.cost_difference = None

        self.file_csv_ID = "identification_set.csv"
        self.file_csv_ver = "verification_set.csv"

        # self.colors
        tud_blue = "#0066A2"
        tud_blue2 = "#61A4B4"
        tud_blue3 = "#007188"
        tud_black = "#000000"
        tud_grey = "#808080"
        tud_red = "#c3312f"
        tud_orange = "#EB7245"
        tud_yellow = "#F1BE3E"
        tud_green = "#00A390"
        self.colors = [tud_blue, tud_red, tud_green, tud_orange, tud_blue2]

        size = None
        if size == "small":
            self.lw = 2.5
            title_size = 14
            label_size = 12
            self.legend_size = 8
        else:
            self.lw = 4
            title_size = 24
            label_size = 22
            self.legend_size = 15

        self.lw_box = 2.5
        self.boxw = 0.5

        self.csfont = {'fontname': 'Georgia', 'size': title_size}
        self.hfont = {'fontname': 'Georgia', 'size': label_size}

        # Forcing function
        bw = 15
        self.amp = 0.2
        self.load_data_reference()
        self.duration = (self.periods[10] * 2 * np.pi) / bw
        print(self.duration)
        frequencies = 2 * np.pi * self.periods / self.duration
        print("duration = ", self.duration)
        print("frequencies = ", frequencies)

        # ID set
        self.forcing_function = {
            'periods': self.periods,
            'phases_ID': self.phases_id,
            'phases_ver': self.phases_ver,
            'amplitudes': self.amp * self.amplitudes,
        }


        plt.figure()
        for i in range(len(self.periods)):
            plt.plot(frequencies[i], self.amplitudes[i], color=tud_blue, linewidth=self.lw, marker='o')
            plt.plot([frequencies[i], frequencies[i]], [0, self.amplitudes[i]], tud_blue, linewidth=self.lw, alpha=0.7)

        plt.title("Frequency domain reference", **self.csfont)
        plt.xlabel("Frequency (rad/s)", **self.hfont)
        plt.ylabel("Amplitude (-)", **self.hfont)
        plt.xlim(frequencies[0] - 0.1, frequencies[-1] + 10)
        plt.ylim(0.01, self.amplitudes[0] + 0.1)
        plt.yscale("log")
        plt.xscale("log")
        plt.tight_layout(pad=1)

        # Show forcing function:
        fs = 100
        n = int(fs * self.duration)
        t = np.array(range(n)) / fs
        r = np.zeros(n)
        for i in range(n):
            r[i] = self.compute_torque(t[i], True)

        plt.figure()
        plt.plot(t, r, tud_blue, linewidth=self.lw)
        plt.title("Time domain reference", **self.csfont)
        plt.xlabel("Time (s)", **self.hfont)
        plt.ylabel("Amplitude (-)", **self.hfont)
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

    def do(self):
        # Check whether to create or load dataset
        if self.senso_drive_process is not None:
            if self.identification == 0:
                self.generate_data_set(validation=False)
            else:
                self.generate_data_set(validation=True)
        self.load_data_set()
        self.optimize(reoptimize=False)  # Set to true if re-optimizing
        self.compute_metrics()
        self.plot_data()

    def generate_data_set(self, validation):
        if validation is False:
            # Identification set
            b = [0.1, 0.3]
        else:
            # Validation set
            b = [0.05, 0.2]

        time.sleep(1.0)
        t_start = time.time()

        print("warning, this will take ", len(b) * self.duration * 0.5, " seconds.")

        for i in range(len(b)):
            self.Bw = b[i]
            self.Kw = 0.0
            self.send_dict["stiffness"] = self.Kw
            self.send_dict["damping"] = self.Bw
            t0 = time.time()
            t_last = t0

            while t_last - t0 < self.duration:
                self.send_dict["torque"] = self.compute_torque(t_last - t_start, validation)
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

                if t_now - t_start > 0.4:

                    output = {
                        "steering_angle": states["steering_angle"],
                        "steering_rate": states["steering_rate"],
                        "torque": states["torque"],
                        "time": t_last - t_start,
                        "execution_time": dt,
                        "stiffness": self.Kw,
                        "damping": self.Bw,
                    }

                    self.store_variables(output)

        print("finished experiment")
        self.send_dict["exit"] = True
        if self.senso_drive_process.is_alive():
            self.parent_conn.send(self.send_dict)  # Child is for sending
        else:
            exit("sensodrive process was killed")

        if validation is False:
            self.to_csv(self.saved, file=self.file_csv_ID)
        else:
            self.to_csv(self.saved, file=self.file_csv_ver)

    def load_data_reference(self):
        try:
            df_data = pd.read_csv("ID_phases.csv", index_col=0)
            data = df_data.to_dict(orient='list')
            self.phases_id = np.array(data['phases'])
            self.periods = np.array(data['periods'])
            self.amplitudes = np.array(data['amplitudes'])
        except:
            exit("Missing data")

        try:
            df_data = pd.read_csv("Validation_phases.csv", index_col=0)
            data = df_data.to_dict(orient='list')
            self.phases_ver = np.array(data['phases'])
        except:
            exit("Missing data")

    def load_data_set(self):
        try:
            df_ID = pd.read_csv(self.file_csv_ID, index_col=0)
            self.data_set_ID = df_ID.to_dict(orient='list')
        except:
            exit("Missing identification dataset")
        try:
            df_ver = pd.read_csv(self.file_csv_ver, index_col=0)
            self.data_set_ver = df_ver.to_dict(orient='list')
        except:
            exit("Missing verification dataset")

    def optimize(self, reoptimize):
        # Initial guess
        J = 0.06
        m = 0.5
        dh = 0.0
        dl = 0.0
        vt = 0.3
        tau_f = 0.00
        tau_d = 0.00

        p0 = np.array([J, m, dh, dl, vt, tau_f, tau_d])
        bounds_vec = np.array([(0.001, 0.1), (0.2, 1.0), (-0.15, 0.15), (-0.15, 0.15), (0.05, 3),
                               (-0.2, 0.2), (-0.2, 0.2)])

        self.simulate_experiment(p0, linear=False, data_set=self.data_set_ver)
        self.phi_sim_init = self.saved_states["steering_angle"]
        self.phidot_sim_init = self.saved_states["steering_rate"]

        # Optimize
        # Cold-start
        if not reoptimize:
            df_ver = pd.read_csv("params.csv", index_col=0)
            p_cs = df_ver.to_dict(orient='list')
            p_opt = np.array(p_cs["params"])
        else:
            # Nonlinear fit

            options = {
                'maxiter': 10000,
                'maxcor': 20,
                'ftol': 1e-10,
                'gtol': 1e-7,
                'eps': 1e-9,
            }
            p_cs = cp.minimize(self.fun, p0, method='L-BFGS-B', bounds=bounds_vec, options=options)
            print("Results: ", p_cs)

            p_opt = np.array(p_cs["x"])
            names = ["J", "m", "dy", "dx", "vt", "tau_f", "tau_d"]
            param_dict = {"params": p_opt, "names": names}
            self.to_csv(param_dict, "params.csv")

        self.show_nonlins(p_opt[1], p_opt[2], p_opt[3], p_opt[5], p_opt[6], p_opt[4], False)
        p_lin = p_opt
        # p_lin[0] = 0.0447

        self.simulate_experiment(p_opt, linear=False, data_set=self.data_set_ver)
        self.phi_sim_final = self.saved_states["steering_angle"]
        self.phidot_sim_final = self.saved_states["steering_rate"]
        self.simulate_experiment(p_lin, linear=True, data_set=self.data_set_ver)
        self.phi_sim_lin = self.saved_states["steering_angle"]
        self.phidot_sim_lin = self.saved_states["steering_rate"]

    def to_csv(self, to_be_saved, file):
        columns = []
        for key in to_be_saved.keys():
            columns.append(key)
        df = pd.DataFrame(data=to_be_saved)
        df.to_csv(file)

    def compute_torque(self, t, verificate):
        period = self.forcing_function["periods"]
        phases_ID = self.forcing_function["phases_ID"]
        phases_ver = self.forcing_function["phases_ver"]
        amplitude = self.forcing_function["amplitudes"]
        torque = 0
        for i in range(10):
            wt = (2*period[i] / self.duration) * (2 * np.pi)
            if verificate is True:
                torque += amplitude[i] * np.sin(wt * t + phases_ver[i])
            else:
                torque += amplitude[i] * np.sin(wt * t + phases_ID[i])
        return torque

    def fun(self, p):
        self.simulate_experiment(p, linear=False, data_set=self.data_set_ID)
        phi = self.data_set_ID["steering_angle"]
        phi_sim = self.saved_states["steering_angle"]
        phidot = self.data_set_ID["steering_rate"]
        phidot_sim = self.saved_states["steering_rate"]
        dphi = np.array(phi) - np.array(phi_sim)
        dphidot = np.array(phidot) - np.array(phidot_sim)
        N = len(phi)
        cost = 1/N * (5*np.inner(dphi, dphi) + 30*np.inner(dphidot, dphidot))
        return cost

    def compute_metrics(self):
        phi =self.data_set_ver["steering_angle"]
        phidot = self.data_set_ver["steering_rate"]
        dphi_f = np.abs(np.array(phi) - np.array(self.phi_sim_final))
        dphidot_f = np.abs(np.array(phidot) - np.array(self.phidot_sim_final))
        dphi_i = np.abs(np.array(phi) - np.array(self.phi_sim_init))
        dphidot_i = np.abs(np.array(phidot) - np.array(self.phidot_sim_init))
        dphi_l = np.abs(np.array(phi) - np.array(self.phi_sim_lin))
        dphidot_l = np.abs(np.array(phidot) - np.array(self.phidot_sim_lin))
        cost_f = 5 * (dphi_f*dphi_f) + 2 * (dphidot_f*dphidot_f)
        cost_i = 5 * (dphi_i * dphi_i) + 2 * (dphidot_i * dphidot_i)
        cost_l = 5 * (dphi_l * dphi_l) + 2 * (dphidot_l * dphidot_l)

        var_y = np.var(phi)
        var_ydot = np.var(phidot)
        var_dyhat_i = np.var(dphi_i)
        var_dyhat_f = np.var(dphi_f)
        var_dyhat_l = np.var(dphi_l)
        var_dydothat_i = np.var(dphidot_i)
        var_dydothat_f = np.var(dphidot_f)
        var_dydothat_l = np.var(dphidot_l)
        vaf_phi_i = 100 * (1 - (var_dyhat_i / var_y))
        vaf_phi_f = 100 * (1 - (var_dyhat_f / var_y))
        vaf_phi_l = 100 * (1 - (var_dyhat_l / var_y))
        vaf_phidot_i = 100 * (1 - (var_dydothat_i / var_ydot))
        vaf_phidot_f = 100 * (1 - (var_dydothat_f / var_ydot))
        vaf_phidot_l = 100 * (1 - (var_dydothat_l / var_ydot))
        self.VAF = {"initial": [vaf_phi_i, vaf_phidot_i],
               "final": [vaf_phi_f, vaf_phidot_f],
               "linear": [vaf_phi_l, vaf_phidot_l],}

        N = len(phi)
        # cost_f = 1 / N * (5 * np.inner(dphi_f, dphi_f) + 2 * np.inner(dphidot_f, dphidot_f))
        # cost_i = 1 / N * (5 * np.inner(dphi_i, dphi_i) + 2 * np.inner(dphidot_i, dphidot_i))
        # cost_l = 1 / N * (5 * np.inner(dphi_l, dphi_l) + 2 * np.inner(dphidot_l, dphidot_l))

        print(cost_l.shape)
        d_f = np.mean(dphi_f)
        d_i = np.mean(dphi_i)
        d_l = np.mean(dphi_l)
        var_f = np.var(dphi_f)
        var_i = np.var(dphi_i)
        var_l = np.var(dphi_l)

        self.angle_difference = {
            "Initial": dphi_i,
            "Final": dphi_f,
            "Linear": dphi_l,
        }

        self.rate_difference = {
            "Initial": dphidot_i,
            "Final": dphidot_f,
            "Linear": dphidot_l,
        }

        self.cost_difference = {
            "Initial": cost_i,
            "Final": cost_f,
            "Linear": cost_l,
        }

    def show_nonlins(self, m, dh, dl, tau_f, tau_d, vt, multiple):

        # Simulations
        gamma = [0.1, 0.5, 1, 5]
        # gamma = 1

        n = 1

        plt.figure()
        labels = []
        vsp = 2 * vt
        vs = 3 * vsp
        v = np.linspace(-1.5*vs, 1.8*vs, 400)

        if not multiple:
            string = "$\gamma$ = " + str(gamma)
            labels.append(string)
            gv = v/vsp * np.exp(-(v/(np.sqrt(2) * vsp))**2 + 1/2)
            fc = tau_d * np.tanh(v/vt)
            f_fric = gv * tau_f + fc
            # plt.plot(v, gv * tau_f, self.colors[0], linewidth=2)
            # plt.plot(v, fc, self.colors[1], linewidth=2)
            plt.plot(v, f_fric, self.colors[0], linewidth=self.lw)

        if multiple:
            for i in range(len(gamma)):
                string = "$\\tau_c$ = " + str(round(gamma[i]*tau_d, 2)) + ", $\\tau_s = $" + str(round(gamma[i]*tau_f,2))
                gv = v / vsp * np.exp(-(v / (np.sqrt(2) * vsp)) ** 2 + 1 / 2)
                fc = gamma[i] * tau_d * np.tanh(v / vt)
                f_fric = gv * gamma[i] * tau_f + fc
                # plt.plot(v, gv * tau_f * gamma[i], self.colors[i], alpha=0.5, linestyle="--", linewidth=2)
                # plt.plot(v, fc, self.colors[i], alpha=0.5, linestyle="-.", linewidth=2)
                plt.plot(v, f_fric, self.colors[i], label=string, linewidth=self.lw,)

        plt.xlim(v[0], v[-1])
        plt.xlabel("Steering rate $\dot{\phi}}(t)$ (rad/s)", **self.hfont)
        plt.ylabel("Torque $f_{fric}(t)$ (Nm)", **self.hfont)
        plt.title("Friction torque", **self.csfont)

        plt.tight_layout(pad=1)
        if multiple:
            plt.legend(prop={"size": self.legend_size}, loc='upper right')

        phi = np.linspace(-np.pi, np.pi, 200)
        g = 9.81
        tau_g = - m * g * dh * np.sin(phi) - m * g * dl * np.cos(phi)


        fac = 360 / (2 * np.pi)

        offset = [0.05, 0.1, 0.05, 0.1]

        plt.figure()

        if not multiple:
            plt.plot(fac * phi, tau_g, self.colors[0], linewidth=self.lw)
        else:

            for i in range(len(offset)):
                dx = 0.00
                if i > 1:
                    dx = -0.05
                dy = dl - offset[i]
                tau_g = - m * g * dy * np.sin(phi) - m * g * dx * np.cos(phi)
                string = "$\\delta x = $" + str(round(dx, 2)) + "$, \\delta y = $" + str(round(dy, 2))
                plt.plot(fac * phi, tau_g, self.colors[i], label=string, linewidth=self.lw)


        plt.xlabel("Steering angle $\phi}(t)$ (Degrees)", **self.csfont)
        plt.ylabel("Torque $f_{g}(t)$ (Nm)", **self.csfont)
        plt.title("Gravitational torque", **self.csfont)
        plt.xlim(fac * phi[0], fac * phi[-1])
        plt.tight_layout(pad=1)
        if multiple:
            plt.legend(prop={"size": self.legend_size}, loc='upper left')

    def store_variables(self, output):
        for key in output.keys():
            self.saved.setdefault(key, []).append(output[key])

    def save_states(self, save_state):
        for key in save_state.keys():
            self.saved_states.setdefault(key, []).append(save_state[key])

    def dynamics(self, x, u, p, b, k, linear):
        # Parameters
        g = 9.81
        J = p[0]
        m = p[1]
        dh = p[2]
        dl = p[3]
        vt = p[4]
        vsp = 2 * vt
        tau_fric = p[5]
        tau_d = p[6]

        # Gravity
        tau_g = - m * g * (dh * np.sin(x[0]) + dl * np.cos(x[0]))
        # print(tau_g)

        # Friction
        v = x[1]
        # print(v)
        gv = v / vsp * np.exp(-(v / (np.sqrt(2) * vsp)) ** 2 + (1 / 2))
        # print(gv)
        tau_f = gv * tau_fric + tau_d * np.tanh(v / vt)
        # print(tau_f)

        # Dynamic equations
        phidot = x[1]
        if linear is not True:
            phiddot = 1 / J * (-b * x[1] - k * x[0] + u + tau_g + tau_f)
        else:
            phiddot = 1 / J * (-b * x[1] - k * x[0] + u)
        xdot = np.array([phidot, phiddot])

        return xdot

    def simulate_experiment(self, p, linear, data_set):
        t_ex = data_set["execution_time"]
        u = data_set["torque"]
        b = data_set["damping"]
        k = data_set["stiffness"]
        N = len(t_ex)
        self.saved_states = {}

        self.x = np.array([data_set["steering_angle"][0], data_set["steering_rate"][0]])
        save_state = {"steering_angle": self.x[0], "steering_rate": self.x[1]}
        self.save_states(save_state)

        for i in range(N-1):
            h = t_ex[i]
            xdot = self.dynamics(self.x, u[i], p, b[i], k[i], linear)
            x_new = h * xdot + self.x
            save_state = {"steering_angle": x_new[0], "steering_rate": x_new[1]}
            self.save_states(save_state)
            self.x = x_new

    def plot_data(self):

        print("plotting stuff")
        t = self.data_set_ver["time"]
        t_ex = self.data_set_ver["execution_time"]
        u = self.data_set_ver["torque"]
        phi = self.data_set_ver["steering_angle"]
        phidot = self.data_set_ver["steering_rate"]

        self.show_nonlins(0.2, 0.1, 0.05, -0.05, -0.05, 0.5, True)

        # plot metrics
        # print(self.metrics)
        angle_difference = pd.DataFrame.from_dict(self.angle_difference)
        rate_difference = pd.DataFrame.from_dict(self.rate_difference)
        cost_difference = pd.DataFrame.from_dict(self.cost_difference)


        color_palette = {"Initial": self.colors[0], "Final": self.colors[1], "Linear": self.colors[2]}


        fig1, ax1 = plt.subplots()
        ax1.set_title('Absolute steering angle difference', **self.csfont)
        res = sns.boxplot(linewidth=self.lw_box, width=self.boxw, data=angle_difference, palette=color_palette, ax=ax1, showfliers=False)
        ax1.set_ylabel("Steering angle (rad)", **self.hfont)
        res.set_xticklabels(res.get_xmajorticklabels(), **self.hfont)
        plt.tight_layout(pad=1)

        fig3, ax3 = plt.subplots()
        ax3.set_title('Absolute steering rate difference', **self.csfont)
        res3 = sns.boxplot(linewidth=self.lw_box, width=self.boxw, data=rate_difference, palette=color_palette, ax=ax3, showfliers=False)
        ax3.set_ylabel("Absolute steering rate (rad/s)", **self.hfont)
        res3.set_xticklabels(res3.get_xmajorticklabels(), **self.hfont)
        plt.tight_layout(pad=1)

        fig2, ax2 = plt.subplots()
        ax2.set_title('Cost function values', **self.csfont)
        res2 = sns.boxplot(linewidth=self.lw_box, width=self.boxw, data=cost_difference, palette=color_palette, ax=ax2, showfliers=False)
        ax2.set_ylabel("Cost function value (-)", **self.hfont)
        res2.set_xticklabels(res2.get_xmajorticklabels(), **self.hfont)
        plt.tight_layout(pad=1)

        print("Make this into a table! VAF values:")
        print("Initial guess: ", self.VAF["initial"])
        print("Finald: ", self.VAF["final"])
        print("Linear: ", self.VAF["linear"])

        print("means (init, final, linear), (angle, rate, cost):")
        print(np.mean(self.angle_difference["Initial"]), np.mean(self.angle_difference["Final"]),
              np.mean(self.angle_difference["Linear"]))
        print(np.mean(self.rate_difference["Initial"]), np.mean(self.rate_difference["Final"]),
              np.mean(self.rate_difference["Linear"]))
        print(np.mean(self.cost_difference["Initial"]), np.mean(self.cost_difference["Final"]),
              np.mean(self.cost_difference["Linear"]))

        print("variance (init, final, linear), (angle, rate, cost):")
        print(np.var(self.angle_difference["Initial"]), np.var(self.angle_difference["Final"]),
              np.var(self.angle_difference["Linear"]))
        print(np.var(self.rate_difference["Initial"]), np.var(self.rate_difference["Final"]),
              np.var(self.rate_difference["Linear"]))
        print(np.var(self.cost_difference["Initial"]), np.var(self.cost_difference["Final"]),
              np.var(self.cost_difference["Linear"]))

        # plot time signals
        plt.figure()
        plt.plot(t, phi, self.colors[0], linewidth=self.lw, label="Measured $\phi(t)$")
        plt.plot(t, self.phi_sim_final, self.colors[1], linewidth=self.lw, label="Estimated $\hat{\phi}(t)$")
        plt.plot(t, self.phi_sim_lin, self.colors[2], linewidth=self.lw, label="Linear $\hat{\phi}(t)$")
        plt.title("Steering angle comparison", **self.csfont)
        plt.xlabel("Time (s)", **self.hfont)
        plt.ylabel("Steering angle (rad)", **self.hfont)
        plt.xlim(t[0], 30)
        plt.legend(prop={"size": self.legend_size}, loc='upper right')
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(t, phidot, self.colors[0], linewidth=self.lw, label="Measured $\dot{\phi}(t)$")
        plt.plot(t, self.phidot_sim_final, self.colors[1], linewidth=self.lw, label="Estimated $\dot{\hat{\phi}}(t)$")
        plt.plot(t, self.phidot_sim_lin, self.colors[2], linewidth=self.lw, label="Linear $\dot{\hat{\phi}}(t)$")
        plt.title("Steering rate comparison", **self.csfont)
        plt.xlabel("Time (s)", **self.hfont)
        plt.ylabel("Steering rate (rad/s)", **self.hfont)
        plt.xlim(t[0], 30)
        plt.legend(prop={"size": self.legend_size}, loc='upper right')
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(t, u, self.colors[0], linewidth=self.lw, label="input $u(t)$")
        plt.plot(t, np.array(phidot) - np.array(self.phidot_sim_final), self.colors[1], linewidth=self.lw, label="$\Delta \dot{\hat{\phi}}(t)$")
        plt.title("Difference measured and simulated states", **self.csfont)
        plt.xlabel("Time (s)", **self.hfont)
        plt.ylabel("Difference in magnitude", **self.hfont)
        plt.legend(prop={"size": self.legend_size}, loc='upper right')
        plt.tight_layout(pad=1)

        self.save_all_figures()
        plt.show()

    def save_all_figures(self):
        a = input("Save in thesis folder? 0: No, 1: Yes   ")
        if int(a) == 1:
            location = "..\\..\\thesis\\img\\greybox.pdf"
            pp = PdfPages(location)
        else:
            pp = PdfPages('greybox.pdf')

        figs = None
        if figs is None:
            figs = [plt.figure(n) for n in plt.get_fignums()]
        for fig in figs:
            fig.savefig(pp, format='pdf')
        pp.close()


if __name__ == "__main__":

    if platform.system() == 'Windows':
        with wres.set_resolution(10000):
            set_data = input("Generate data set? 0: No, 1: Yes.    Your answer = ")

            # Activate sensodrive
            if int(set_data) == 1:
                print("starting sensodrive")
                identification = input("What set do you want to generate? 0: ID, 1: Verification.   Your answer = ")
                parent_conn, child_conn = mp.Pipe(True)
                senso_drive_process = SensoDriveModule(parent_conn, child_conn)
                senso_drive_process.start()
            elif int(set_data) == 0:
                parent_conn, child_conn, senso_drive_process = None, None, None
                identification = -1
            else:
                exit("That is no option")
            greybox = GreyBox(parent_conn, child_conn, senso_drive_process, int(identification))
            greybox.do()

