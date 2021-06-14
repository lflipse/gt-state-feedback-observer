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

        # Forcing function
        bw = 15
        period = np.array([5, 8, 11, 17, 26, 43, 71, 131, 233, 431])
        self.duration = (period[7] * 2 * np.pi) / bw
        frequencies = 2 * np.pi * period / self.duration
        phases_ID = [1.79893804,  1.00694642, -1.40575088,  0.56145111, -1.92977185, -1.1689079, -0.61034581,
                      -0.75180265, -0.71232366, -0.2509144]
        phases_ver = [-1.82153521, -1.03968042,  0.17035112, -0.90442899, -0.27152334, -1.05591208, -1.88837196,
                      -0.8715442, -0.3636802, 1.20421911]
        # print(phases)
        amp = 0.3
        amplitude = amp * np.array([1, 1, 1, 1, 1, 1, 1, 0.1, 0.1, 0.1])
        self.forcing_function = {
            'period': period,
            'phases_ID': phases_ID,
            'phases_ver': phases_ver,
            'amplitude': amplitude,
        }

        fig = plt.figure()
        for i in range(len(period)):
            plt.plot(frequencies[i], amplitude[i], color=tud_blue, marker='o')
            plt.plot([frequencies[i], frequencies[i]], [0, amplitude[i]], tud_blue, alpha=0.7, linewidth=2.5)

        plt.title("Forcing function in frequency domain", **csfont)
        plt.xlabel("Frequency (rad/s)", **hfont)
        plt.ylabel("Amplitude (-)", **hfont)
        plt.xlim(0.5, frequencies[-1] + 10, )
        plt.ylim(0.01, amplitude[0]+0.1)
        plt.yscale("log")
        plt.xscale("log")

        # Show forcing function:
        fs = 100
        n = int(fs * self.duration)
        t = np.array(range(n)) / fs
        u = np.zeros(n)
        for i in range(n):
            u[i] = self.compute_torque(t[i], True)

        plt.figure()
        plt.plot(t, u, tud_blue, linewidth=2.5)
        plt.title("Forcing function in time domain", **csfont)
        plt.xlabel("Time (s)", **hfont)
        plt.ylabel("Amplitude (-)", **hfont)
        plt.xlim(t[0], self.duration*0.2)

    def do(self):
        # Check whether to create or load dataset
        if self.senso_drive_process is not None:
            if self.identification == 0:
                self.generate_data_set(validation=False)
            else:
                self.generate_data_set(validation=True)
        self.load_data_set()
        self.optimize(False)
        self.compute_metrics()
        self.plot_data()

    def generate_data_set(self, validation):
        if validation is False:
            # Identification set
            b = [0.3, 0.5, 0.7, 0.9]
        else:
            # Validation set
            b = [0.2, 0.45, 0.65, 0.95]

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

            while t_last - t0 < 0.5 * self.duration:
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
        J = 0.01
        m = 0.4
        dh = 0.0
        dl = 0.0
        vt = 0.8
        tau_f = -0.05
        tau_d = -0.05

        p0 = np.array([J, m, dh, dl, vt, tau_f, tau_d])
        bounds_vec = np.array([(0.001, 0.1), (0.2, 1.0), (-0.15, 0.15), (-0.15, 0.15), (0.05, 3),
                               (-0.4, 0.0), (-0.5, 0.0)])

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

        self.simulate_experiment(p_opt, linear=False, data_set=self.data_set_ver)
        self.phi_sim_final = self.saved_states["steering_angle"]
        self.phidot_sim_final = self.saved_states["steering_rate"]
        self.simulate_experiment(p_opt, linear=True, data_set=self.data_set_ver)
        self.phi_sim_lin = self.saved_states["steering_angle"]
        self.phidot_sim_lin = self.saved_states["steering_rate"]

    def to_csv(self, to_be_saved, file):
        columns = []
        for key in to_be_saved.keys():
            columns.append(key)
        df = pd.DataFrame(data=to_be_saved)
        df.to_csv(file)

    def compute_torque(self, t, verificate):
        period = self.forcing_function["period"]
        phases_ID = self.forcing_function["phases_ID"]
        phases_ver = self.forcing_function["phases_ver"]
        amplitude = self.forcing_function["amplitude"]
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
        cost = 1/N * (5*np.inner(dphi, dphi) + 2*np.inner(dphidot, dphidot))
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
            "Initial estimate": dphi_i,
            "Final estimate": dphi_f,
            "Linear estimate": dphi_l,
        }

        self.rate_difference = {
            "Initial estimate": dphidot_i,
            "Final estimate": dphidot_f,
            "Linear estimate": dphidot_l,
        }

        self.cost_difference = {
            "Initial estimate": cost_i,
            "Final estimate": cost_f,
            "Linear estimate": cost_l,
        }

    def show_nonlins(self, m, dh, dl, tau_f, tau_d, vt, multiple):
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
            plt.plot(v, gv * tau_f, colors[0], linewidth=2)
            plt.plot(v, fc, colors[1], linewidth=2)
            plt.plot(v, f_fric, colors[2], linewidth=2)

        plt.xlabel("Steering rate $\dot{\phi}}(t)$ (rad/s)", **csfont)
        plt.ylabel("Friction torque $f_{fric}(t)$ (Nm)", **csfont)
        plt.title("Friction torque", **csfont)
        plt.xlim(v[0], v[-1])
        plt.tight_layout(pad=1)

        phi = np.linspace(-np.pi, np.pi, 200)
        g = 9.81
        tau_g = - m * g * dh * np.sin(phi) - m * g * dl * np.cos(phi)

        plt.figure()
        fac = 360 / (2 * np.pi)

        if not multiple:
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
        t = self.data_set_ver["time"]
        t_ex = self.data_set_ver["execution_time"]
        u = self.data_set_ver["torque"]
        phi = self.data_set_ver["steering_angle"]
        phidot = self.data_set_ver["steering_rate"]

        plt.figure()
        plt.plot(t, t_ex)

        plt.figure()
        plt.plot(t, u)

        # plot metrics
        # print(self.metrics)
        angle_difference = pd.DataFrame.from_dict(self.angle_difference)
        rate_difference = pd.DataFrame.from_dict(self.rate_difference)
        cost_difference = pd.DataFrame.from_dict(self.cost_difference)

        color_palette = {"Initial estimate": tud_yellow, "Final estimate": tud_blue, "Linear estimate": tud_green}

        fig1, ax1 = plt.subplots()
        ax1.set_title('Absolute steering angle difference')
        sns.boxplot(data=angle_difference, palette=color_palette, ax=ax1, showfliers = False)

        fig2, ax2 = plt.subplots()
        ax2.set_title('Cost function values')
        sns.boxplot(data=cost_difference, palette=color_palette, ax=ax2, showfliers=False)
        # ax2.set_yscale("log")


        # plot time signals

        plt.figure()
        plt.plot(t, phi, tud_blue, linewidth=2.5, label="Measured $\phi(t)$")
        plt.plot(t, self.phi_sim_final, tud_red, linewidth=2.5, label="Estimated $\hat{\phi}(t)$")
        plt.plot(t, self.phi_sim_lin, tud_green, linewidth=2.5, label="Linear $\hat{\phi}(t)$")
        plt.title("Steering angle comparison", **csfont)
        plt.xlabel("Time (s)", **hfont)
        plt.ylabel("Steering angle (rad)", **hfont)
        plt.xlim(t[0], 20)
        plt.legend()

        plt.figure()
        plt.plot(t, phidot, tud_blue, linewidth=2.5, label="Measured $\dot{\phi}(t)$")
        plt.plot(t, self.phidot_sim_final, tud_red, linewidth=2.5, label="Estimated $\dot{\hat{\phi}}(t)$")
        plt.plot(t, self.phidot_sim_lin, tud_green, linewidth=2.5, label="Linear $\dot{\hat{\phi}}(t)$")
        plt.title("Steering rate comparison", **csfont)
        plt.xlabel("Time (s)", **hfont)
        plt.ylabel("Steering rate (rad/s)", **hfont)
        plt.xlim(t[0], 20)
        plt.legend()

        plt.figure()
        plt.plot(t, u, tud_black, linewidth=1.5, label="input $u(t)$")
        plt.plot(t, np.array(phidot) - np.array(self.phidot_sim_final), tud_green, linewidth=2.5, label="$\Delta \dot{\hat{\phi}}(t)$")
        plt.title("Difference measured and simulated states", **csfont)
        plt.xlabel("Time (s)", **hfont)
        plt.ylabel("Difference in magnitude", **hfont)
        plt.legend()

        plt.show()

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
