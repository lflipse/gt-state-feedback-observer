import matplotlib.pyplot as plt
import os
from datetime import datetime
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages
import nfft
import scipy as cp
import pandas as pd
import seaborn as sns

class PlotStuff:
    def __init__(self):
        print("About to be plotting stuff")
        self.csfont = {'fontname': 'Georgia'}
        self.hfont = {'fontname': 'Georgia'}

        # Colors
        self.tud_black = "#000000"
        self.tud_blue = "#0066A2"
        self.tud_red = "#c3312f"
        self.tud_green = "#00A390"
        self.tud_yellow = "#F1BE3E"
        self.tud_orange = "#EB7245"
        self.tud_lightblue = "#00B7D3"

        colors = [self.tud_blue, self.tud_red, self.tud_green, self.tud_yellow, self.tud_orange, self.tud_lightblue]
        sns.set(style="whitegrid")
        sns.set_palette(sns.color_palette(colors))

        self.fig1 = plt.figure()
        self.fig2 = plt.figure()
        self.fig3 = plt.figure()
        self.fig4 = plt.figure()
        self.fig5 = plt.figure()


    def plot_experiment(self, data, compare):
        pd_metric = pd.DataFrame.from_dict(data)
        solos = pd_metric["condition"] < 7
        pd_metric_no_solo = pd_metric[solos]
        print(pd_metric)
        if compare:
            # plot box-plots for all metrics
            Y = ["rmse", "rmsu", "cost"]
            T = ["rmse", "rmsu", "cost"]
        else:
            Y = ["rms_angle_error", "rms_rate_error", "rms_human_torque",
                 "rms_robot_torque", "human_angle_cost", "robot_angle_cost"]
            T = ["Controller Performance (Steering angle)", "Controller Performance (Steering rate)", "Control Effort (Estimated human)",
                 "Control Effort (Robot)", "Cost Function Weight (Estimated Human)", "Cost Function Weight (Robot)"]
            unit = ["Root-mean-square steering angle error (rad)", "Root-mean-square steering rate error (rad/s)",
                    "Root-mean-square steering torque (Nm)", "Root-mean-square steering torque (Nm)",
                    "Steering angle cost function weight (-)", "Steering angle cost function weight (-)",]
        for i in range(len(Y)):
            y = Y[i]
            title = T[i]
            if compare:
                hue = y + "_index"
            else:
                hue = None
            # print(y)
            # if y == "rms_human_torque" or y == "human_angle_cost" or y == "robot_angle_cost":
            #     metric = pd_metric_no_solo
            # else:
            #     metric = pd_metric

            metric = pd_metric

            plt.figure()
            sns.swarmplot(linewidth=2.5, data=metric, x="condition", y=y, hue=hue, alpha=0.7)
            ax = sns.boxplot(linewidth=2.5, data=metric, x="condition", y=y, hue=hue, width=0.7)
            ax.set_xlabel("Condition", **self.csfont)
            ax.set_ylabel(unit[i], **self.csfont)
            ax.set_title(title, **self.csfont)

    def plot_trial(self, data):
        # UNPACK DATA
        t = data["time"]
        ur = data["torque"]
        x = data["steering_angle"]
        r = data["reference_angle"]
        xdot = data["steering_rate"]
        rdot = data["reference_rate"]
        t_ex = data["execution_time"]

        x_hat = data["state_estimate_pos"]
        Lhhat_pos = data["estimated_human_gain_pos"]
        Lhhat_vel = data["estimated_human_gain_vel"]
        Lr_pos = data["robot_gain_pos"]
        Lr_vel = data["robot_gain_vel"]
        uhhat = data["estimated_human_input"]
        uhtilde = data["input_estimation_error"]

        # Conditions
        conditions = data["condition"]
        human_noise = data["human_noise"]
        role = data["role"]
        xddot = data["acceleration"]
        xhatdot = data["state_estimate_vel"]

        q_h_1 = data["estimated_human_cost_1"]
        q_h_2 = data["estimated_human_cost_2"]
        q_r_1 = data["robot_cost_pos"]
        q_r_2 = data["robot_cost_vel"]

        # Steering angle
        plt.figure(self.fig1)
        plt.title("Measured and estimated steering angle", **self.csfont)
        plt.plot(t, r, self.tud_black, linewidth=2.5, linestyle="-", alpha=0.7, label="Reference $\phi_r(t)$")
        plt.plot(t, x, self.tud_blue, linestyle="--", linewidth=2.5, label="Steering angle $\phi(t)$")
        plt.plot(t, x_hat, self.tud_red, linewidth=2.5, linestyle="--", label="Estimated $\hat{\phi}(t)$")
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Steering angle (rad)', **self.hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        # Steering rate
        plt.figure(self.fig2)
        plt.title("Measured and estimated steering rate", **self.csfont)
        plt.plot(t, rdot, self.tud_black, linewidth=2.5, linestyle="-", alpha=0.7, label="Reference $\dot{\phi}_r(t)$")
        plt.plot(t, xdot, self.tud_blue, linestyle="--", linewidth=2.5, label="Steering rate $\dot{\phi}(t)$")
        plt.plot(t, xhatdot, self.tud_red, linewidth=2.5, linestyle="--", label="Estimated $\dot{\hat{\phi}}(t)$")
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Steering rate (rad/s)', **self.hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        # Input torque (robot)
        plt.figure()
        plt.title("Input torque", **self.csfont)
        plt.plot(t, np.array(ur), self.tud_blue, linestyle="--", linewidth=2, label="Input torque (robot) $u_r(t)$")
        plt.plot(t, np.array(uhhat), self.tud_red, linestyle="--", linewidth=2, alpha=1, label="Estimated (human) $\hat{u}_h(t)$")
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Torque (Nm)', **self.hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        plt.figure(self.fig3)
        plt.title("Gains position", **self.csfont)
        plt.plot(t, Lr_pos, self.tud_blue, linestyle="--", linewidth=3, label="Robot gain $L_r(t)$")
        plt.plot(t, Lhhat_pos, self.tud_red, linestyle="--", linewidth=3, label="Estimated human gain $\hat{L}_h(t)$")
        # self.draw_regions(t, conditions, Lr_pos, Lhhat_pos)
        self.limit_y(Lr_pos, Lhhat_pos)
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Gain (Nm)', **self.hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, t[-1])
        plt.tight_layout(pad=1)

        plt.figure()
        plt.title("Computational times", **self.csfont)
        plt.plot(t, t_ex)

        plt.figure(self.fig4)
        plt.title("Cost function weights", **self.csfont)
        plt.plot(t, q_r_1, self.tud_blue, linestyle="--", linewidth=3, label="Robot cost $Q_{r,1}(t)$")
        plt.plot(t, q_h_1, self.tud_red, linestyle="--", linewidth=3, label="Estimated human cost $\hat{Q}_{h,1}(t)$")
        # self.draw_regions(t, conditions, q_r_1, q_h_1)
        self.limit_y(q_r_1, q_h_1)
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Gain (Nm)', **self.hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, t[-1])
        plt.tight_layout(pad=1)

        plt.figure(self.fig5)
        plt.title("Cost function weights", **self.csfont)
        plt.plot(t, q_r_2, self.tud_blue, linestyle="--", linewidth=3, label="Robot cost $Q_{r,2}(t)$")
        plt.plot(t, q_h_2, self.tud_red, linestyle="--", linewidth=3, label="Estimated human cost $\hat{Q}_{h,2}(t)$")
        # self.draw_regions(t, conditions, q_r_2, q_h_2)
        self.limit_y(q_r_2, q_h_2)
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Gain (Nm)', **self.hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, t[-1])
        plt.tight_layout(pad=1)

        # plt.show()

        # self.save_all_figures()



    def limit_y(self, var1, var2):
        test = 1

    def save_all_figures(self):
        pp = PdfPages('figures\\experiment.pdf')
        figs = None
        if figs is None:
            figs = [plt.figure(n) for n in plt.get_fignums()]
        for fig in figs:
            fig.savefig(pp, format='pdf')
        pp.close()
