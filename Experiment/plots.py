import matplotlib.pyplot as plt
import os
from datetime import datetime
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages
# import nfft
import scipy as cp
import pandas as pd
import seaborn as sns

class PlotStuff:
    def __init__(self):
        print("About to be plotting stuff")
        title_size = 14
        label_size = 10
        self.csfont = {'fontname': 'Georgia', 'size': title_size}
        self.hfont = {'fontname': 'Georgia', 'size': label_size}

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

        sns.set(style="whitegrid")


    def plot_experiment(self, data, averaged_data, compare):
        # Human torque vs robot torque
        data_pd = pd.DataFrame(data)
        options = {"alpha": 1, "s": 25}
        h = sns.jointplot(data=data_pd, palette=self.colors, x="rms_human_torque", y="rms_robot_torque",
                          hue="condition", joint_kws=options)
        h.ax_joint.set_xlabel('RMS human torque (Nm)', **self.hfont)
        h.ax_joint.set_ylabel('RMS robot torque (Nm)', **self.hfont)
        plt.tight_layout(pad=1)
        h.ax_joint.legend(title='Condition', prop={"size": 12}, loc='upper left')

        # Conflict
        # h2 = sns.jointplot(data=data_pd, palette=self.colors, x="rms_human_torque", y="conflict",
        #                   hue="condition", joint_kws=options)
        # h2.ax_joint.set_xlabel('RMS human torque', **self.hfont)
        # h2.ax_joint.set_ylabel('Conflict', **self.hfont)
        # plt.tight_layout(pad=1)
        # h2.ax_joint.legend(title='Condition', prop={"size": 12}, loc='upper left')
        plt.figure()
        h2 = sns.kdeplot(data=data_pd, x="conflict", hue="condition", fill=True, alpha=0.5)
        h2.set_xlabel("Conflict (-)")
        # h2.legend(title='Condition', prop={"size": 12}, loc='upper right')

        # Sanity check
        h3 = sns.jointplot(data=data_pd, palette=self.colors, y="rms_human_torque", x="human_angle_gain",
                           hue="condition", joint_kws=options)
        h3.ax_joint.set_ylabel('RMS Human Torque (Nm)', **self.hfont)
        h3.ax_joint.set_xlabel('Estimated Human Error Gain (Nm/rad)', **self.hfont)
        plt.tight_layout(pad=1)
        h3.ax_joint.legend(title='Condition', prop={"size": 12}, loc='upper left')
        self.save_all_figures()

        # Sanity check
        h4 = sns.jointplot(data=data_pd, palette=self.colors, y="rms_angle_error", x="human_angle_gain",
                           hue="condition", joint_kws=options)
        h4.ax_joint.set_ylabel('RMS Steering Angle Error (rad)', **self.hfont)
        h4.ax_joint.set_xlabel('Estimated Human Error Gain (Nm/rad)', **self.hfont)
        plt.tight_layout(pad=1)
        h4.ax_joint.legend(title='Condition', prop={"size": 12}, loc='upper left')
        self.save_all_figures()

    def plot_data(self, raw_data, trials, participant):
        figb, axs = plt.subplots(4, 3)
        figc, axsb = plt.subplots(4, 3)

        for i in range(trials):
            data = raw_data[participant, i]
            condition = data["condition"][0]
            repetition = data["repetition"][0]

            # UNPACK DATA
            t = data["time"]
            ur = data["torque"]
            uhhat = data["estimated_human_input"]
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

            uhtilde = data["input_estimation_error"]
            uh_rec = np.array(uhhat) - np.array(uhtilde)

            Pr = np.array(ur) * np.array(xdot)
            Ph = np.array(uh_rec) * np.array(xdot)
            auth = 1 - np.abs(Ph)/(np.abs(Pr) + np.abs(Ph) + 0.001)


            # Conditions
            q_h_1 = data["estimated_human_cost_1"]
            q_h_2 = data["estimated_human_cost_2"]
            q_r_1 = data["robot_cost_pos"]
            q_r_2 = data["robot_cost_vel"]

            label_robot = "Robot"
            label_human = "Estimated Human"

            if condition == "Manual Control":
                ls = '-'
                # label_human = 'Manual control'
                line_color = self.tud_orange
                title = "Manual \n Control"
                c = 0
            elif condition == "Positive Reinforcement":
                ls = '-'
                line_color = self.tud_red
                title = "Positive Re."
                c = 1
            elif condition == "Negative Reinforcement":
                ls = '-'
                line_color = self.tud_blue
                title = "Negative Re."
                c = 2
            elif condition == "Mixed Reinforcement":
                ls = '-'
                line_color = self.tud_green
                title = "Mixed Re."
                c = 3
            else:
                ls = '.'
                line_color = self.tud_yellow

            labels = [label_human, label_robot]
            colors = [self.tud_red, self.tud_blue]

            # if repetition == 0:
            #     plt.figure(fig1)
            # elif repetition == 1:
            #     plt.figure(fig2)
            # if repetition == 2:
            #     plt.figure(fig3)
            # elif repetition == 3:
            #     plt.figure(fig4)

            t_start = 0
            t_example = 25
            t_end = t[-1]

            if repetition == 0:
                fig, (ax2, ax1, ax3) = plt.subplots(3)

                fig.suptitle(condition, **self.csfont)
                ax1.stackplot(t, Lhhat_pos, Lr_pos, colors=colors, labels=labels, edgecolor='black', linewidth=0.8)
                ax1.set_ylabel('Gain (Nm/rad)', **self.hfont)
                ax1.legend(prop={"size": 12}, loc='upper left')
                ax1.set_xticks([])
                ax1.set_xlim(t_start, t_example)
                ax1.set_ylim(-1, 12)

                # plt.ylim(-3, 6)

                ax2.plot(t, error, self.tud_blue)
                ax2.set_ylabel('Error (rad)', **self.hfont)
                ax2.set_xticks([])
                ax2.set_xlim(t_start, t_example)

                ax3.stackplot(t, uhhat, ur, -np.array(uhtilde), colors=[self.tud_red, self.tud_blue, self.tud_orange],
                              labels=['Estimated Human', 'Robot', 'Estimation Error'], edgecolor='black', linewidth=0.2)
                # ax3.set_xticks([])
                ax3.set_ylabel('Input torque (Nm)', **self.hfont)
                ax3.legend(prop={"size": 8}, loc='upper left')
                ax3.set_xlim(t_start, t_example)
                ax3.set_xlabel("Time (s)")

                plt.tight_layout(pad=1)

            auth_est = (np.array(Lr_pos) - np.array(Lhhat_pos)) / (np.array(Lr_pos) + np.array(Lhhat_pos) + 0.001)
            auth = (ur - uh_rec) / (ur + uh_rec + 0.001)

            figb.suptitle("Estimated Control Authority", **self.csfont)
            if c > 0:
                axs[repetition, c-1].plot(t, auth_est, label='Estimated authority')
                axs[repetition, c - 1].plot(t, auth, alpha=0.3, label='Measured authority')
                axs[repetition, c-1].set_ylim(-1.5, 1.5)
                axs[repetition, c - 1].set_xlim(t_start, t_end)
                if repetition == 0:
                    axs[repetition, c - 1].set_title(title)
                    axs[repetition, c - 1].set_xticks([])
                    axs[repetition, c-1].legend(prop={"size": 8}, loc='lower right')
                elif repetition == 3:
                    axs[repetition, c - 1].set_xlabel('Time (s)')
                else:
                    axs[repetition, c - 1].set_xticks([])
            # plt.tight_layout(pad=1)

            figc.suptitle("Gains", **self.csfont)
            if c > 0:
                axsb[repetition, c - 1].stackplot(t, Lhhat_pos, Lr_pos, baseline='zero', colors=colors,
                                                  edgecolor='black', linewidth=0.8, labels=labels)
                axsb[repetition, c - 1].set_ylim(-1, 12)
                axsb[repetition, c - 1].set_xlim(t_start, t_end)
                if repetition == 0:
                    axsb[repetition, c - 1].set_title(title)
                    axsb[repetition, c - 1].set_xticks([])
                    axsb[repetition, c - 1].legend(prop={"size": 6}, loc='upper left')
                if repetition == 3:
                    axsb[repetition, c - 1].set_xlabel('Time (s)')
                else:
                    axsb[repetition, c - 1].set_xticks([])
            # plt.tight_layout(pad=1)

    def save_all_figures(self):
        pp = PdfPages('..\\Experiment\\figures\\pilot.pdf')
        figs = None
        if figs is None:
            figs = [plt.figure(n) for n in plt.get_fignums()]
        for fig in figs:
            fig.savefig(pp, format='pdf')
        pp.close()
