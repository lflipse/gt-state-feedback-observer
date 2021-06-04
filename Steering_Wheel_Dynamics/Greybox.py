import numpy as np
import multiprocessing as mp
import time
import os
import matplotlib.pyplot as plt
import platform
import wres


import sys
# insert at 1, 0 is the script path (or '' in REPL)
sys.path.insert(1, '..')

from Steering_Wheel_Dynamics.SensoDriveTest import SensoDriveModule



class GreyBox:
    def __init__(self, parent_conn, child_conn, senso_drive_process):
        # Initial values
        self.Bw = 0
        self.Kw = 0

        self.n = 1  # No. experiments
        self.duration = 10  # Seconds per experiment

        self.parent_conn = parent_conn
        self.child_conn = child_conn
        self.senso_drive_process = senso_drive_process

        self.send_dict = {}
        self.saved = {}
        self.saved_states = {}
        self.x = np.array([0, 0])

    def do(self):
        # Loop for n experiments
        for i in range(self.n):
            self.Bw = 0.4
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

                output = {
                    "steering_angle": states["steering_angle"],
                    "steering_rate": states["steering_rate"],
                    "torque": states["torque"],
                    "time": t_last - t0,
                    "execution_time": dt
                }

                self.store_variables(output)


        print("finished experiment")
        self.send_dict["exit"] = True
        if senso_drive_process.is_alive():
            parent_conn.send(self.send_dict)  # Child is for sending
        else:
            exit("sensodrive process was killed")

        self.simulate_experiment()

        self.plot_data()

    def compute_torque(self, t):
        fs = 3
        torque = 0.4 * np.sin(2 * np.pi * t / fs)
        return torque

    def store_variables(self, output):
        for key in output.keys():
            self.saved.setdefault(key, []).append(output[key])

    def save_states(self, save_state):
        for key in save_state.keys():
            self.saved_states.setdefault(key, []).append(save_state[key])

    def dynamics(self, x, u, p):
        # p = [J, m, dh, tau_fric, alpha]
        g = 9.81
        J = p[0]
        m = p[1]
        dh = p[2]
        tau_fric = p[3]
        alpha = p[4]
        # tau_g = - m * g * dh * np.sin(x[0])
        tau_g = 0
        tau_f = tau_fric * (1/(1+np.exp(-alpha*x[1])) - 1/(1+np.exp(alpha*x[1])))
        # tau_f = 0
        phidot = x[1]
        phiddot = 1/J * (-self.Bw * x[1] - self.Kw * x[0] + u + tau_g + tau_f)
        xdot = np.array([phidot, phiddot])
        return xdot

    def simulate_experiment(self):
        t_ex = self.saved["execution_time"]
        u = self.saved["torque"]
        N = len(t_ex)
        J = 0.0447
        m = 0.2
        dh = 0.1
        tau_fric = -0.1
        alpha = 3
        p = [J, m, dh, tau_fric, alpha]
        self.x = np.array([self.saved["steering_angle"][0], self.saved["steering_rate"][0]])
        save_state = {"steering_angle": self.x[0], "steering_rate": self.x[1]}
        self.save_states(save_state)

        for i in range(N-1):
            h = t_ex[i]
            xdot = self.dynamics(self.x, u[i], p)

            x_new = h * xdot + self.x
            save_state = {"steering_angle": x_new[0], "steering_rate": x_new[1]}
            self.save_states(save_state)
            self.x = x_new


    def plot_data(self):
        print("plotting stuff")
        t = self.saved["time"]
        t_ex = self.saved["execution_time"]
        u = self.saved["torque"]
        phi = self.saved["steering_angle"]
        phi_sim = self.saved_states["steering_angle"]
        phidot = self.saved["steering_rate"]
        phidot_sim = self.saved_states["steering_rate"]

        plt.figure()
        plt.plot(t, t_ex)

        plt.figure()
        plt.plot(t, u)

        plt.figure()
        plt.plot(t, phi)
        plt.plot(t, phi_sim)
        plt.figure()
        plt.plot(t, phidot)
        plt.plot(t, phidot_sim)

        plt.show()

if __name__ == "__main__":

    if platform.system() == 'Windows':
        with wres.set_resolution(10000):
            # Activate sensodrive
            parent_conn, child_conn = mp.Pipe(True)
            senso_drive_process = SensoDriveModule(parent_conn, child_conn)
            senso_drive_process.start()
            greybox = GreyBox(parent_conn, child_conn, senso_drive_process)
            greybox.do()

