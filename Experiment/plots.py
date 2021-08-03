import matplotlib.pyplot as plt
import os
from datetime import datetime
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages


class PlotStuff:
    def __init__(self):
        print("Plotting stuff")

    def plot(self, exp_data):
        csfont = {'fontname': 'Georgia'}
        hfont = {'fontname': 'Georgia'}

        # Colors
        tud_blue = "#0066A2"
        tud_black = "#000000"
        tud_red = "#c3312f"
        tud_green = "#00A390"
        tud_yellow = "#F1BE3E"

        # UNPACK DATA
        # General
        t = exp_data["time"]
        ur = exp_data["torque"]
        x = exp_data["steering_angle"]
        r = exp_data["reference_angle"]
        xdot = exp_data["steering_rate"]
        rdot = exp_data["reference_rate"]
        e = exp_data["angle_error"]
        edot = exp_data["rate_error"]
        t_ex = exp_data["execution_time"]

        x_hat = exp_data["state_estimate_pos"]
        Lhhat_pos = exp_data["estimated_human_gain_pos"]
        Lhhat_vel = exp_data["estimated_human_gain_vel"]
        Lr_pos = exp_data["robot_gain_pos"]
        Lr_vel = exp_data["robot_gain_vel"]
        uhhat = exp_data["estimated_human_input"]
        uhtilde = exp_data["input_estimation_error"]

        # Conditions
        conditions = exp_data["condition"]
        human_noise = exp_data["human_noise"]
        robot_noise = exp_data["robot_noise"]



        xddot = exp_data["acceleration"]
        xhatdot = exp_data["state_estimate_vel"]

        self.check_corelation(ur, uhhat)

        q_h_1 = exp_data["estimated_human_cost_1"]
        q_h_2 = exp_data["estimated_human_cost_2"]
        try:
            q_r_1 = exp_data["robot_cost_pos"]
            q_r_2 = exp_data["robot_cost_vel"]
        except:
            print("that did not work")

        # RMSE
        rmse = self.root_mean_squared(e)
        # print(rmse)
        rmsedot = self.root_mean_squared(edot)

        plt.figure()
        plt.title("Performance (RMSE)", **csfont)
        plt.plot(t, rmse, tud_red, linewidth=2.5, linestyle="--", alpha=1, label="Angle error")
        plt.plot(t, rmsedot, tud_blue, linewidth=2.5, linestyle="--", alpha=1, label="Rate error")
        plt.ylabel('Root mean squared error (rad)', **hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, t[-1])
        plt.tight_layout(pad=1)

        # RMSU
        rmsur = self.root_mean_squared(ur)
        rmsuhhat = self.root_mean_squared(uhhat)

        plt.figure()
        plt.title("Effort (RMSU)", **csfont)
        plt.plot(t, rmsur, tud_blue, linewidth=2.5, linestyle="--", alpha=1, label="Robot input")
        plt.plot(t, rmsuhhat, tud_red, linewidth=2.5, linestyle="--", alpha=1, label="Human input")
        self.draw_regions(t, conditions, rmsur, rmsuhhat)
        self.limit_y(rmsur, rmsuhhat)
        plt.ylabel('Root mean squared input (torque)', **hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, t[-1])
        plt.tight_layout(pad=1)

        dominance = (rmsur - rmsuhhat)/(rmsur + rmsuhhat)

        plt.figure()
        plt.title("Dominance", **csfont)
        plt.plot(t, dominance, tud_blue, linewidth=2.5, linestyle="--", alpha=1, label="Robot input")
        plt.ylabel('Root mean squared input (torque)', **hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, t[-1])
        plt.tight_layout(pad=1)

        # Steering angle
        plt.figure()
        plt.title("Measured and estimated steering angle", **csfont)
        plt.plot(t, r, tud_black, linewidth=2.5, linestyle="-", alpha=0.7, label="Reference $\phi_r(t)$")
        plt.plot(t, x, tud_blue, linestyle="--", linewidth=2.5, label="Steering angle $\phi(t)$")
        plt.plot(t, x_hat, tud_red, linewidth=2.5, linestyle="--", label="Estimated $\hat{\phi}(t)$")
        plt.xlabel('Time (s)', **hfont)
        plt.ylabel('Steering angle (rad)', **hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        # Steering angle
        fig, axs = plt.subplots(2)
        fig.suptitle('Vertically stacked subplots')
        axs[0].plot(t, human_noise)
        axs[1].plot(t, robot_noise)



        # Steering rate
        plt.figure()
        plt.title("Measured and estimated steering rate", **csfont)
        plt.plot(t, rdot, tud_black, linewidth=2.5, linestyle="-", alpha=0.7, label="Reference $\dot{\phi}_r(t)$")
        plt.plot(t, xdot, tud_blue, linestyle="--", linewidth=2.5, label="Steering rate $\dot{\phi}(t)$")
        plt.plot(t, xhatdot, tud_red, linewidth=2.5, linestyle="--", label="Estimated $\dot{\hat{\phi}}(t)$")
        plt.xlabel('Time (s)', **hfont)
        plt.ylabel('Steering rate (rad/s)', **hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        # Input torque (robot)
        plt.figure()
        plt.title("Input torque", **csfont)
        plt.plot(t, np.array(ur), tud_blue, linestyle="--", linewidth=2, label="Input torque (robot) $u_r(t)$")
        plt.plot(t, np.array(uhhat), tud_red, linestyle="--", linewidth=2, alpha=1, label="Estimated (human) $\hat{u}_h(t)$")
        plt.xlabel('Time (s)', **hfont)
        plt.ylabel('Torque (Nm)', **hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.title("Gains position", **csfont)
        plt.plot(t, Lr_pos, tud_blue, linestyle="--", linewidth=3, label="Robot gain $L_r(t)$")
        plt.plot(t, Lhhat_pos, tud_red, linestyle="--", linewidth=3, label="Estimated human gain $\hat{L}_h(t)$")
        # plt.plot(t, uh_vir, tud_black, linestyle="--", linewidth=2, alpha=1, label="Virtual (human) $u_{h,vir}(t)$")
        self.draw_regions(t, conditions, Lr_pos, Lhhat_pos)
        self.limit_y(Lr_pos, Lhhat_pos)
        plt.xlabel('Time (s)', **hfont)
        plt.ylabel('Gain (Nm)', **hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, t[-1])
        plt.tight_layout(pad=1)

        plt.figure()
        plt.title("Computational times", **csfont)
        plt.plot(t, t_ex)

        plt.figure()
        plt.title("Cost function weights", **csfont)
        plt.plot(t, q_r_1, tud_blue, linestyle="--", linewidth=3, label="Robot cost $Q_{r,1}(t)$")
        plt.plot(t, q_h_1, tud_red, linestyle="--", linewidth=3, label="Estimated human cost $\hat{Q}_{h,1}(t)$")
        self.draw_regions(t, conditions, q_r_1, q_h_1)
        self.limit_y(q_r_1, q_h_1)
        plt.xlabel('Time (s)', **hfont)
        plt.ylabel('Gain (Nm)', **hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, t[-1])
        plt.tight_layout(pad=1)

        plt.figure()
        plt.title("Cost function weights", **csfont)
        plt.plot(t, q_r_2, tud_blue, linestyle="--", linewidth=3, label="Robot cost $Q_{r,2}(t)$")
        plt.plot(t, q_h_2, tud_red, linestyle="--", linewidth=3, label="Estimated human cost $\hat{Q}_{h,2}(t)$")
        # plt.plot(t, uh_vir, tud_black, linestyle="--", linewidth=2, alpha=1, label="Virtual (human) $u_{h,vir}(t)$")
        self.draw_regions(t, conditions, q_r_2, q_h_2)
        self.limit_y(q_r_2, q_h_2)
        plt.xlabel('Time (s)', **hfont)
        plt.ylabel('Gain (Nm)', **hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, t[-1])
        plt.tight_layout(pad=1)

        self.save_all_figures()

        plt.show()

    def draw_regions(self, t, conditions, var1, var2):
        c = 0
        count = 0
        dy = max(max(var1), max(var2)) - min(min(var1), min(var2))
        mid = (max(max(var1), max(var2)) + min(min(var1), min(var2))) / 2
        for i in range(len(t)):
            if c == 1:
                text = "Both noiseless"
            elif c == 2:
                text = "Noisy robot"
            elif c == 3:
                text = "Noisy human"
            else:
                text = "Both noisy"

            if conditions[i] != c:
                plt.plot([t[i], t[i]], [-100, 100], 'k--', linewidth=3)

                count += 1
                if count > 2:
                    plt.text(t[i + 200], mid - 0.6 * dy, text, color='black', bbox=dict(facecolor='white', edgecolor='black',
                                                                                  boxstyle='round,pad=1'))
                else:
                    plt.text(t[i + 200], mid + 0.6 * dy, text, color='black', bbox=dict(facecolor='white', edgecolor='black',
                                                                                  boxstyle='round,pad=1'))

            c = conditions[i]

        plt.ylim(mid - 0.7 * dy, mid + 0.7 * dy)

    def limit_y(self, var1, var2):
        test = 1

    def check_corelation(self, ur, uh):
        # TODO: need to fix the differences in sample times
        Rxx = np.correlate(ur, uh, "same")
        # print(Rxx)
        # plt.figure()
        # plt.plot(Rxx)

    def root_mean_squared(self, var):
        l = len(var)
        rms = np.zeros(l)
        for i in range(l):
            rms[i] = np.sqrt(1 / l * var[i] ** 2)

        return rms

    def save_figure_dump(self, type):
        working_dir = os.getcwd()
        dateTimeObj = datetime.now()
        location1 = working_dir + "\\fig_dump\\" + str(dateTimeObj.month)
        location2 = location1 + "\\" + str(dateTimeObj.day)

        if not os.path.exists(location1):
            os.makedirs(location1)
            print("made directory: ", location1)

        if not os.path.exists(location2):
            os.makedirs(location2)
            print("made directory: ", location2)

        string = location2 + "\\" + str(dateTimeObj.hour) + "_" + str(dateTimeObj.minute) + "_" + \
                 str(dateTimeObj.second) + "_" + str(type) + ".pdf"
        plt.savefig(string)

    def save_all_figures(self):
        pp = PdfPages('experiment.pdf')
        figs = None
        if figs is None:
            figs = [plt.figure(n) for n in plt.get_fignums()]
        for fig in figs:
            fig.savefig(pp, format='pdf')
        pp.close()
