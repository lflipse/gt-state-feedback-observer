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
        self.colors= {"Manual Control": self.tud_orange, "Positive Reinforcement": self.tud_red,
               "Negative Reinforcement": self.tud_blue, "Mixed Reinforcement": self.tud_green}

        self.gains = {"human_gain": [],
                      "robot_gain": [],
                      "Condition": []}

        # colors = [self.tud_blue, self.tud_red, self.tud_green, self.tud_yellow, self.tud_orange, self.tud_lightblue]
        sns.set(style="whitegrid")
        # sns.set_palette(sns.color_palette(self.colors))

        left, width = 0.1, 0.65
        bottom, height = 0.1, 0.65
        spacing = 0.005
        rect_scatter = [left, bottom, width, height]
        rect_histx = [left, bottom + height + spacing, width, 0.2]
        rect_histy = [left + width + spacing, bottom, 0.2, height]

        self.fig1 = plt.figure()
        self.fig2 = plt.figure()
        self.fig3 = plt.figure()
        self.fig4 = plt.figure()
        self.fig5 = plt.figure()
        self.fig6 = plt.figure()

    def plot_experiment(self, data, averaged_data, compare):
        pd_metric = pd.DataFrame.from_dict(data)
        pd_averaged = pd.DataFrame.from_dict(averaged_data)


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

            metric = pd_metric

            plt.figure()
            sns.swarmplot(linewidth=2.5, data=metric, x="condition", y=y, hue='participant', alpha=0.4)
            ax = sns.boxplot(linewidth=2.5, data=metric, palette=self.colors, x="condition", y=y, width=0.4)
            ax.set_xlabel("Condition", **self.csfont)
            ax.set_ylabel(unit[i], **self.csfont)
            ax.set_title(title, **self.csfont)

        plt.figure()
        sns.swarmplot(linewidth=2.5, data=pd_averaged, x="conditions", y="rms_angle_error", hue='participants', alpha=0.7)
        ax = sns.boxplot(linewidth=2.5, data=pd_averaged, x="conditions", y="rms_angle_error", hue='participants', width=0.7)
        ax.set_xlabel("Condition", **self.csfont)
        ax.set_ylabel("Root-mean-squared error", **self.csfont)
        ax.set_title("Performance", **self.csfont)

        print(pd_metric)

        options = {"alpha": 1, "s": 25}
        # h = sns.jointplot(data=pd_metric, x="human_angle_gain", y="rms_angle_error", hue="condition", joint_kws=options)
        h = sns.jointplot(data=pd_metric, x="human_angle_gain", y="rms_angle_error", hue="condition", joint_kws=options)
        # h.plot_joint(sns.kdeplot, data=pd_metric, x="human_angle_gain", y="rms_angle_error", color="r", zorder=0, levels=1, hue="participant")
        sns.kdeplot(data=pd_metric, x="human_angle_gain", y="rms_angle_error", hue="participant", ax=h.ax_joint, zorder=5, levels=1, legend=False, )
        # h.plot_marginals(sns.rugplot, color="r", height=-.15, clip_on=False)
        # JointGrid has a convenience function
        # or set labels via the axes objects

        h.ax_joint.set_xlabel('Average Human Controller Gain (Nm)', **self.hfont)
        h.ax_joint.set_ylabel('Average Joint Performance', **self.hfont)
        # h.ax_joint.set_xlim(-1.2, 1.2)
        # h.ax_joint.set_ylim(-1, 15)
        plt.tight_layout(pad=1)



    def plot_data(self, raw_data, trials, participant):
        # trials = 4
        self.t_end = 20
        for i in range(trials):
            data = raw_data[participant, i]
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
            error = np.array(x) - np.array(r)

            x_hat = data["state_estimate_pos"]
            Lhhat_pos = data["estimated_human_gain_pos"]
            Lhhat_vel = data["estimated_human_gain_vel"]
            Lr_pos = data["robot_gain_pos"]
            Lr_vel = data["robot_gain_vel"]
            uhhat = data["estimated_human_input"]
            uhtilde = data["input_estimation_error"]
            # t_off = data["turn_off_time"][0]
            # print(t_off)

            # Conditions
            q_h_1 = data["estimated_human_cost_1"]
            q_h_2 = data["estimated_human_cost_2"]
            q_r_1 = data["robot_cost_pos"]
            q_r_2 = data["robot_cost_vel"]

            label_robot = condition + " Robot"
            label_human = condition + " Human"

            if condition == "Manual Control":
                ls = '-'
                label_human = 'Manual control estimated $\hat{L}_h(t)$'
                line_color = self.tud_orange
            elif condition == "Positive Reinforcement":
                ls = '-'
                line_color = self.tud_red
            elif condition == "Negative Reinforcement":
                ls = '-'
                line_color = self.tud_blue
            else:
                ls = '-'
                line_color = self.tud_green



            plt.figure(self.fig1)
            plt.title("Steering error gain", **self.csfont)
            plt.plot(t, Lhhat_pos, line_color, linestyle=ls, linewidth=4, label=label_human)
            # self.draw_regions(t, conditions, Lr_pos, Lhhat_pos)
            self.limit_y(Lr_pos, Lhhat_pos)
            plt.xlabel('Time (s)', **self.hfont)
            plt.ylabel('Gain (Nm)', **self.hfont)
            plt.legend(prop={"size": 14}, loc='upper left')
            plt.xlim(0, t[-1]-self.t_end)
            plt.tight_layout(pad=1)

            plt.figure(self.fig2)
            plt.title("Velocity error gain", **self.csfont)
            plt.plot(t, Lhhat_vel, line_color, linestyle=ls, linewidth=4, label=label_human)
            # self.draw_regions(t, conditions, Lr_pos, Lhhat_pos)
            self.limit_y(Lr_vel, Lhhat_vel)
            plt.xlabel('Time (s)', **self.hfont)
            plt.ylabel('Gain (Nm/s)', **self.hfont)
            plt.legend(prop={"size": 14}, loc='upper left')
            plt.xlim(0, t[-1]-self.t_end)
            plt.tight_layout(pad=1)

            plt.figure(self.fig3)
            plt.title("Steering error cost", **self.csfont)
            plt.plot(t, q_h_1, line_color, linestyle=ls, linewidth=4, label=label_human)
            # self.draw_regions(t, conditions, Lr_pos, Lhhat_pos)
            self.limit_y(Lr_pos, Lhhat_pos)
            plt.xlabel('Time (s)', **self.hfont)
            plt.ylabel('Gain (Nm)', **self.hfont)
            plt.legend(prop={"size": 14}, loc='upper left')
            plt.xlim(0, t[-1]-self.t_end)
            plt.tight_layout(pad=1)

            plt.figure(self.fig4)
            plt.title("Velocity error cost", **self.csfont)
            plt.plot(t, q_h_2, line_color, linestyle=ls, linewidth=4, label=label_human)
            # self.draw_regions(t, conditions, Lr_pos, Lhhat_pos)
            self.limit_y(Lr_vel, Lhhat_vel)
            plt.xlabel('Time (s)', **self.hfont)
            plt.ylabel('Gain (Nm/s)', **self.hfont)
            plt.legend(prop={"size": 14}, loc='upper left')
            plt.xlim(0, t[-1]-self.t_end)
            plt.tight_layout(pad=1)

            # self.plot_gains(t, Lhhat_pos, Lr_pos, line_color, ls, label_human)
            self.save_gains(Lhhat_pos, Lr_pos, condition)


            # Control authority

            C = self.compute_authority(Lr_pos, Lhhat_pos)

            options = {"arrowstyle": '->', }

            plt.figure(self.fig5)
            plt.title("Control authority", **self.csfont)

            plt.plot(t, C, line_color, linestyle=ls, linewidth=4, label=label_human)
            plt.plot([0, t[-1]], [1, 1], self.tud_blue, alpha=0.7, linestyle='--')
            plt.plot([0, t[-1]], [-1, -1], self.tud_blue, alpha=0.7, linestyle='--')

            plt.annotate("Robot has control authority", (10, 1.05), arrowprops=options)
            plt.annotate("Human has control authority", (10, -0.95), arrowprops=options)
            plt.annotate("Human in competition", (20, 1.8), arrowprops=options)
            plt.annotate("Robot in competition", (20, -1.8), arrowprops=options)

            plt.xlabel('Time (s)', **self.hfont)
            plt.ylabel('Authority (-)', **self.hfont)
            plt.legend(prop={"size": 8}, loc='upper right')
            plt.xlim(0, t[-1])
            plt.ylim(-2, 2)
            plt.tight_layout(pad=1)

        # self.plot_gains()
        gains_pd = pd.DataFrame(self.gains)

        options = {"alpha": 0.005, "s": 12}
        h = sns.jointplot(data=gains_pd, x="human_gain", y="robot_gain", hue="Condition",
                      palette=self.colors, joint_kws=options)
        # JointGrid has a convenience function
        # or set labels via the axes objects

        h.ax_joint.set_xlabel('Human Gain (Nm)', **self.hfont)
        h.ax_joint.set_ylabel('Robot Gain (Nm)', **self.hfont)
        # h.ax_joint.set_xlim(-1, 5.5)
        # h.ax_joint.set_ylim(-1, 15)
        plt.tight_layout(pad=1)

        # plt.show()

        # self.save_all_figures()

    def plot_gains(self, t, Lh, Lr, line_color, ls, label_human):
        n = len(Lh)
        plt.figure(self.fig6)
        self.ax.set_title("Steering error gain", **self.csfont)

        Lhpd = pd.DataFrame(Lh)
        Lrpd = pd.DataFrame(Lr)


        # the scatter plot:
        self.ax.scatter(Lh, Lr, c=line_color, alpha=0.005, s=5, label=label_human)

        # now determine nice limits by hand:
        binwidth = 0.25
        xymax = max(np.max(np.abs(Lh)), np.max(np.abs(Lr)))
        lim = (int(xymax / binwidth) + 1) * binwidth

        bins = np.arange(-lim, lim + binwidth, binwidth)
        # self.ax_histx.hist(Lh, bins=bins, color=line_color)
        # self.ax_histy.hist(Lr, bins=bins, orientation='horizontal', color=line_color)

        # self.ax_histx()
        sns.kdeplot(data=Lhpd, ax=self.ax_histx, color=line_color)
        # self.ax_histy()
        # sns.kdeplot(data=Lrpd, ax=self.ax_histy, color=line_color, orientation)


        self.ax_histx.tick_params(axis="x", labelbottom=False)
        self.ax_histy.tick_params(axis="y", labelleft=False)

        self.ax.set_xlabel('Human Gain (Nm)', **self.hfont)
        self.ax.set_ylabel('Robot Gain (Nm)', **self.hfont)
        leg = self.ax.legend(prop={"size": 10}, loc='upper left')
        for lh in leg.legendHandles:
            lh.set_alpha(1)

        self.ax.set_xlim(-2, 4)
        self.ax.set_ylim(0.5, 14)
        plt.figure(self.fig6)
        plt.tight_layout(pad=1)

        # plt.tight_layout(pad=1)


    def limit_y(self, var1, var2):
        test = 1

    def save_gains(self, Lh, Lr, condition):
        condition_list = [condition] * len(Lh)
        self.gains["human_gain"].extend(Lh)
        self.gains["robot_gain"].extend(Lr)
        self.gains["Condition"].extend(condition_list)

    def compute_authority(self, Lr, Lh):
        n = len(Lh)
        C = np.zeros(n)
        for i in range(n):
            try:
                C[i] = min(max((Lr[i] - Lh[i])/(Lr[i] + Lh[i]), -2), 2)
            except:
                C[i] = min(max((Lr[i] - Lh[i]) / (Lr[i] + Lh[i] + 0.001), -2), 2)
        return C

    def save_all_figures(self):
        pp = PdfPages('figures\\experiment.pdf')
        figs = None
        if figs is None:
            figs = [plt.figure(n) for n in plt.get_fignums()]
        for fig in figs:
            fig.savefig(pp, format='pdf')
        pp.close()
