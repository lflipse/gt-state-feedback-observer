import matplotlib.pyplot as plt
import os
from datetime import datetime
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages
# import nfft
import scipy as cp
import pandas as pd
import seaborn as sns
from Demo.strategy import Strategy

class PlotStuff:
    def __init__(self):
        print("About to be plotting stuff")
        title_size = 14
        label_size = 10
        self.linewidth = 3
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


    def plot_experiment(self, data, averaged_data, participants, conditions):
        # Human torque vs robot torque
        data_pd = pd.DataFrame(data)
        mean_data_pd = pd.DataFrame(averaged_data)
        options = {"alpha": 1, "s": 25}
        # h = sns.jointplot(data=data_pd, palette=self.colors, x="rms_human_torque", y="rms_robot_torque",
        #                   hue="condition", joint_kws=options)
        # h.ax_joint.set_xlabel('RMS human torque (Nm)', **self.hfont)
        # h.ax_joint.set_ylabel('RMS robot torque (Nm)', **self.hfont)
        # plt.tight_layout(pad=1)
        # h.ax_joint.legend(title='Condition', prop={"size": 12}, loc='upper left')
        #
        # plt.figure()
        # h2 = sns.kdeplot(data=data_pd, x="conflict", hue="condition", fill=True, alpha=0.5)
        # h2.set_xlabel("Conflict (-)")
        #
        # # Sanity check
        # h3 = sns.jointplot(data=data_pd, palette=self.colors, y="rms_human_torque", x="human_angle_gain",
        #                    hue="condition", joint_kws=options)
        # h3.ax_joint.set_ylabel('RMS Human Torque (Nm)', **self.hfont)
        # h3.ax_joint.set_xlabel('Estimated Human Error Gain (Nm/rad)', **self.hfont)
        # plt.tight_layout(pad=1)
        # h3.ax_joint.legend(title='Condition', prop={"size": 12}, loc='upper left')
        # self.save_all_figures()
        #
        # # Sanity check
        # h4 = sns.jointplot(data=data_pd, palette=self.colors, y="rms_angle_error", x="human_angle_gain",
        #                    hue="condition", joint_kws=options)
        # h4.ax_joint.set_ylabel('RMS Steering Angle Error (rad)', **self.hfont)
        # h4.ax_joint.set_xlabel('Estimated Human Error Gain (Nm/rad)', **self.hfont)
        # plt.tight_layout(pad=1)
        # h4.ax_joint.legend(title='Condition', prop={"size": 12}, loc='upper left')

        strategy = Strategy()
        inputs = strategy.run()
        Qh = inputs["cost_human"]
        Qr1 = inputs["cost_robot_pos"]
        Lr1 = inputs["gain_robot_pos"]
        Lh1 = inputs["gain_human_pos"]
        Qr2 = inputs["cost_robot_neg"]
        Lr2 = inputs["gain_robot_neg"]
        Lh2 = inputs["gain_human_neg"]
        C1 = inputs["auth_pos"]
        C2 = inputs["auth_neg"]


        # Let's go chronologically
        # 1. Costs
        # 1a. Human cost vs robot cost
        fig1 = plt.figure()
        fig2 = plt.figure()
        fig3 = plt.figure()
        axs1 = [fig1.gca(), fig2.gca(), fig3.gca()]
        # f1, axs1 = plt.subplots(1, 3)

        # 1b. System cost vs Performance
        fig4 = plt.figure()
        fig5 = plt.figure()
        fig6 = plt.figure()
        axs2 = [fig4.gca(), fig5.gca(), fig6.gca()]

        # 2a. Human gains vs robot gains
        fig7 = plt.figure()
        fig8 = plt.figure()
        fig9 = plt.figure()
        axs3 = [fig7.gca(), fig8.gca(), fig9.gca()]

        # 2b. System gain vs performance
        fig10 = plt.figure()
        fig11 = plt.figure()
        fig12 = plt.figure()
        axs4 = [fig10.gca(), fig11.gca(), fig12.gca()]

        # 3a. System gain vs performance
        fig13 = plt.figure()
        fig14 = plt.figure()
        fig15 = plt.figure()
        axs5 = [fig13.gca(), fig14.gca(), fig15.gca()]


        for i in range(participants):
            participant_data = mean_data_pd.loc[mean_data_pd["participant"] == i]
            data_manual = participant_data.loc[participant_data["condition"] == "Manual Control"]
            data_negative = participant_data.loc[participant_data["condition"] == "Negative Reinforcement"]
            data_positive = participant_data.loc[participant_data["condition"] == "Positive Reinforcement"]
            performance_dict_manual = data_manual["performance"]
            key = list(performance_dict_manual.keys())[0]
            performance_manual = performance_dict_manual[key]
            color = self.tud_blue

            for j in range(conditions):
                if j == 0:
                    data_now = data_manual.to_dict()
                elif j == 1:
                    data_now = data_negative.to_dict()
                    if i == 0:
                        axs1[j].plot(Qh[:, 0, 0], Qr1[:, 0, 0], color, label="Design", linewidth=self.linewidth, alpha=0.5)
                        axs3[j].plot(Lh1[:, 0], Lr1[:, 0], color, label="Design", linewidth=self.linewidth, alpha=0.5)
                else:
                    data_now = data_positive.to_dict()
                    if i == 0:
                        axs1[j].plot(Qh[:, 0, 0], Qr2[:, 0, 0], color, label="Design", linewidth=self.linewidth, alpha=0.5)
                        axs3[j].plot(Lh2[:, 0], Lr2[:, 0], color, label="Design", linewidth=self.linewidth, alpha=0.5)

                performance_dict = data_now["performance"]
                key = list(performance_dict.keys())[0]
                performance = performance_dict[key]
                standard = 100
                size = int(round(standard + 10/performance, 1))
                print("size = ", size)
                label = "Participant " + str(i)

                # Figure 1a.
                axs1[j].scatter(data_now["cost_human"][key], data_now["cost_robot"][key], s=size, label=label)
                axs1[j].set_xlim(-10, 85)
                axs1[j].set_ylim(-10, 85)
                axs1[j].legend()
                axs1[j].set_xlabel("Averaged Estimated Human Cost Weight (Nm/rad)", **self.hfont)
                axs1[j].set_ylabel("Averaged Robot Cost Weight (-)", **self.hfont)
                axs1[j].set_title(data_now['condition'][key], **self.csfont)

                # Figure 1b.
                axs2[j].scatter(data_now["cost_system"][key], data_now["performance"][key], s=size, label=label)
                # axs2[j].set_xlim(-10, 85)
                # axs2[j].set_ylim(-10, 85)
                axs2[j].legend()
                axs2[j].set_xlabel("Total Estimated System Cost Weight (-)", **self.hfont)
                axs2[j].set_ylabel("RMS Error (rad)", **self.hfont)
                axs2[j].set_title(data_now['condition'][key], **self.csfont)

                # Figure 2a.
                axs3[j].scatter(data_now["gain_human"][key], data_now["gain_robot"][key], s=size, label=label)
                axs3[j].set_xlim(-0.5, 10)
                axs3[j].set_ylim(-0.5, 10)
                axs3[j].legend()
                axs3[j].set_xlabel("Averaged Estimated Human Gain (Nm/rad)", **self.hfont)
                axs3[j].set_ylabel("Averaged Robot Gain (Nm/rad)", **self.hfont)
                axs3[j].set_title(data_now['condition'][key], **self.csfont)

                # Figure 2b.
                axs4[j].scatter(data_now["gain_system"][key], data_now["performance"][key], s=size, label=label)
                # axs4[j].set_xlim(-10, 85)
                # axs4[j].set_ylim(-10, 85)
                axs4[j].legend()
                axs4[j].set_xlabel("Total Estimated System Gain (Nm/rad)", **self.hfont)
                axs4[j].set_ylabel("RMS Error (rad)", **self.hfont)
                axs4[j].set_title(data_now['condition'][key], **self.csfont)

                # Figure 3a.
                axs5[j].scatter(data_now["inputs_human"][key], data_now["inputs_robot"][key], s=size, label=label)
                axs5[j].set_xlim(0, 0.2)
                axs5[j].set_ylim(0, 0.2)
                axs5[j].legend()
                axs5[j].set_xlabel("RMS Human Torque (Nm)", **self.hfont)
                axs5[j].set_ylabel("RMS Robot Torque (Nm)", **self.hfont)
                axs5[j].set_title(data_now['condition'][key], **self.csfont)



        # sns.swarmplot(data=mean_data_manual, x="cost_human", y="cost_robot", hue="participant", ax=ax1)
        # ax1.set_xlim(0, 80)
        # ax1.set_ylim(0, 80)
        # sns.swarmplot(data=mean_data_negative, x="cost_human", y="cost_robot", hue="participant", ax=ax2)
        # sns.swarmplot(data=mean_data_positive, x="cost_human", y="cost_robot", hue="participant", ax=ax3)



        # self.save_all_figures()




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
