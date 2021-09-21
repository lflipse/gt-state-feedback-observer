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



    def plot_experiment(self, data, averaged_data, compare):
        pd_metric = pd.DataFrame.from_dict(data)
        pd_averaged = pd.DataFrame.from_dict(averaged_data)

        print(pd_averaged)
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
                hue = 'settings'
            # print(y)
            # if y == "rms_human_torque" or y == "human_angle_cost" or y == "robot_angle_cost":
            #     metric = pd_metric_no_solo
            # else:
            #     metric = pd_metric

            metric = pd_metric

            plt.figure()
            sns.swarmplot(linewidth=2.5, data=metric, x="condition", y=y, hue='participant', alpha=0.7)
            ax = sns.boxplot(linewidth=2.5, data=metric, x="condition", y=y, hue=hue, width=0.7)
            ax.set_xlabel("Condition", **self.csfont)
            ax.set_ylabel(unit[i], **self.csfont)
            ax.set_title(title, **self.csfont)

        plt.figure()
        sns.swarmplot(linewidth=2.5, data=pd_averaged, x="conditions", y="rms_angle_error", hue='participants', alpha=0.7)
        ax = sns.boxplot(linewidth=2.5, data=pd_averaged, x="conditions", y="rms_angle_error", hue='participants', width=0.7)
        ax.set_xlabel("Condition", **self.csfont)
        ax.set_ylabel("Root-mean-squared error", **self.csfont)
        ax.set_title("Performance", **self.csfont)

    def plot_data(self, raw_data, trials, participant):

        for i in range(trials-1):
            data = raw_data[participant, i+1]
            condition = data["condition"][0]
            setting = data["setting"][0]

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


            q_h_1 = data["estimated_human_cost_1"]
            q_h_2 = data["estimated_human_cost_2"]
            q_r_1 = data["robot_cost_pos"]
            q_r_2 = data["robot_cost_vel"]

            if condition == "Manual Control":
                ls = '-.'
                label_human = 'Manual control estimated $\hat{L}_h(t)$'
            elif condition == "Static Shared Control":
                ls = '--'
                label_robot = 'Static controller gain $L_r(t)$'
                label_human = 'Static controller estimated $\hat{L}_h(t)$'
            else:
                ls = '-'
                label_robot = 'Adaptive controller gain $L_r(t)$'
                label_human = 'Adaptive controller estimated $\hat{L}_h(t)$'

            if setting == "Good Visuals":
                plt.figure(self.fig1)
                plt.title("Steering error gain for Good Visuals", **self.csfont)
            else:
                plt.figure(self.fig2)
                plt.title("Steering error gain for Bad Visuals", **self.csfont)


            if condition != "Manual Control":
                plt.plot(t, Lr_pos, self.tud_blue, linestyle=ls, linewidth=3, label=label_robot)
            plt.plot(t, Lhhat_pos, self.tud_red, linestyle=ls, linewidth=3, label=label_human)
            # self.draw_regions(t, conditions, Lr_pos, Lhhat_pos)
            self.limit_y(Lr_pos, Lhhat_pos)
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
