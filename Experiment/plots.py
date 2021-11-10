import matplotlib.pyplot as plt
import os
from datetime import datetime
import numpy as np

# import nfft
import scipy as cp
import pandas as pd
import seaborn as sns
from Demo.strategy import Strategy
from matplotlib.ticker import FormatStrFormatter
import matplotlib

class PlotStuff:
    def __init__(self):
        print("About to be plotting stuff")
        title_size = 18
        label_size_small = 10
        label_size_big = 14
        self.linewidth = 4
        self.csfont = {'fontname': 'Georgia', 'size': title_size}
        self.hfont = {'fontname': 'Georgia', 'size': label_size_big}
        self.hfont_small = {'fontname': 'Georgia', 'size': label_size_small}

        # Colors
        self.tud_black = "#000000"
        self.tud_blue = "#0066A2"
        self.tud_red = "#c3312f"
        self.tud_green = "#00A390"
        self.tud_yellow = "#F1BE3E"
        self.tud_orange = "#EB7245"
        self.tud_lightblue = "#00B7D3"
        self.tud_otherblue = "#61A4B4"

        self.colors = {"Manual Control": self.tud_orange, "Positive Reinforcement": self.tud_green,
                       "Negative Reinforcement": self.tud_otherblue}

        self.order = ["Manual Control", "Negative Reinforcement", "Positive Reinforcement"]

        self.colormap = [self.tud_orange, self.tud_green, self.tud_otherblue]

        sns.set_palette(sns.color_palette(self.colormap))

        self.gains = {"human_gain": [],
                      "robot_gain": [],
                      "Condition": []}

    def annotate_significance(self, no_boxes, ax, significance):
        if no_boxes == 2:
            lines = 1
        elif no_boxes == 3:
            lines = 3

        x = [0, 1, 1, 2, 0, 2]

        for i in range(lines):
            ylims = ax.get_ylim()
            ax.plot([x[2*i], x[2*i], x[2*i+1], x[2*i+1]], [ylims[1] + 0.1 * abs(ylims[1]), ylims[1] + 0.3 * abs(ylims[1]),
                                       ylims[1] + 0.3 * abs(ylims[1]), ylims[1] + 0.1 * abs(ylims[1])], self.tud_black)
            ax.text((x[2*i] + x[2*i+1]) / 2, ylims[1] + 0.35 * abs(ylims[1]), significance[i])

    def plot_metrics(self, metrics, participant, conditions):
        # mainly used to show consitency
        metrics_pd = pd.DataFrame(metrics)
        metrics_pd_no_manual = metrics_pd.loc[metrics_pd["condition"] != "Manual Control"]
        metrics_part = metrics_pd.loc[metrics_pd["participant"] == participant]
        metrics_no_manual = metrics_part.loc[metrics_part["condition"] != "Manual Control"]
        metrics_positive = metrics_pd.loc[metrics_pd["condition"] == "Positive Reinforcement"]
        metrics_negative = metrics_pd.loc[metrics_pd["condition"] == "Negative Reinforcement"]
        metrics_manual = metrics_pd.loc[metrics_pd["condition"] == "Manual Control"]

        labels = ["Manual \n Control", "Negative \n Reinforcement", "Positive \n Reinforcement"]

        # Control authority
        figa, axa = plt.subplots(1, 2, gridspec_kw={'width_ratios': [5, 1]})
        figa.suptitle("Control Authority", **self.csfont)
        sns.boxplot(data=metrics_pd, x="condition", y="authority", palette=self.colors, ax=axa[0], order=self.order)
        sns.swarmplot(data=metrics_pd, x="condition", y="authority", palette=self.colors, ax=axa[0], order=self.order,
                      alpha=0.6, size=6, linewidth=1)
        axa[0].set_xticklabels(labels)
        axa[0].set_xlabel("")
        # self.annotate_significance(2, axa[0], ["***"])
        ylims = axa[0].get_ylim()
        axa[0].set_ylabel("Authority (-)", **self.hfont)
        delta_y = ylims[1] - ylims[0]
        width = delta_y / 40
        sns.histplot(data=metrics_pd_no_manual, y="authority", hue="condition", ax=axa[1],
                     palette=self.colors, binwidth=width)
        axa[1].get_legend().remove()
        axa[1].axes.get_yaxis().set_visible(False)
        plt.tight_layout(pad=1)

        # Performance
        figb, axb = plt.subplots(1, 2, gridspec_kw={'width_ratios': [5, 1]})
        figb.suptitle("Steering Angle Error", **self.csfont)
        sns.boxplot(data=metrics_pd, x="condition", y="rms_angle_error", palette=self.colors, ax=axb[0], order=self.order,)
        sns.swarmplot(data=metrics_pd, x="condition", y="rms_angle_error", palette=self.colors, ax=axb[0], order=self.order,
                      alpha=0.6, size=6, linewidth=1)
        axb[0].set_xticklabels(labels)
        axb[0].set_xlabel("")
        # self.annotate_significance(3, axb[0], ["***", "***", "***"])
        ylims = axb[0].get_ylim()
        axb[0].set_ylabel("RMS error ($^{\circ}$)", **self.hfont)
        delta_y = ylims[1] - ylims[0]
        width = delta_y / 40
        sns.histplot(data=metrics_pd, y="rms_angle_error", hue="condition", ax=axb[1], kde=False,
                     palette=self.colors, binwidth=width)
        # sns.histplot(data=metrics_pd, y="rms_angle_error", hue="condition", ax=axb[1], kde=True,
        #              palette=self.colors, binwidth=width)
        axb[1].get_legend().remove()
        axb[1].axes.get_yaxis().set_visible(False)
        plt.tight_layout(pad=1)

        # Work
        figc, axc = plt.subplots(1, 2, gridspec_kw={'width_ratios': [5, 1]})
        figc.suptitle("Human Input Power", **self.csfont)
        sns.boxplot(data=metrics_pd, x="condition", y="rms_human_power", palette=self.colors, ax=axc[0], order=self.order,)
        sns.swarmplot(data=metrics_pd, x="condition", y="rms_human_power", palette=self.colors, ax=axc[0], order=self.order,
                      alpha=0.6, size=6, linewidth=1)
        axc[0].set_xticklabels(labels)
        axc[0].set_xlabel("")
        axc[0].set_ylabel("RMS power (W)", **self.hfont)
        ylims = axc[0].get_ylim()
        delta_y = ylims[1] - ylims[0]
        width = delta_y / 40
        # sns.histplot(data=metrics_pd, y="rms_human_power", hue="condition", ax=axc[1], kde=True,
        #              palette=self.colors, binwidth=width)
        sns.histplot(data=metrics_pd, y="rms_human_power", hue="condition", ax=axc[1], kde=False,
                     palette=self.colors, binwidth=width)
        axc[1].get_legend().remove()
        axc[1].axes.get_yaxis().set_visible(False)
        plt.tight_layout(pad=1)

        # Variability
        # Work
        figd, axd = plt.subplots(1, 2, gridspec_kw={'width_ratios': [5, 1]})
        figd.suptitle("Cost function variability", **self.csfont)
        sns.boxplot(data=metrics_pd, x="condition", y="cost_variability", palette=self.colors, ax=axd[0])
        sns.swarmplot(data=metrics_pd, x="condition", y="cost_variability", palette=self.colors, ax=axd[0],
                      alpha=0.6, size=6, linewidth=1)
        axd[0].set_xticklabels(labels)
        axd[0].set_xlabel("")
        axd[0].set_ylabel("Variance (-)", **self.hfont)
        ylims = axd[0].get_ylim()
        delta_y = ylims[1] - ylims[0]
        width = delta_y / 40
        # sns.histplot(data=metrics_pd, y="cost_variability", hue="condition", ax=axd[1], kde=True,
        #              palette=self.colors, binwidth=width)
        sns.histplot(data=metrics_pd, y="cost_variability", hue="condition", ax=axd[1], kde=False,
                     palette=self.colors, binwidth=width)
        axd[1].get_legend().remove()
        axd[1].axes.get_yaxis().set_visible(False)
        plt.tight_layout(pad=1)

        # Differences in control strategy
        plt.figure()
        sns.boxplot(data=metrics_pd, x="participant", y="human_angle_cost", hue="condition", palette=self.colors, hue_order=self.order,)
        plt.title("Human control strategy", **self.csfont)
        plt.tight_layout(pad=1)

        # self.save_all_figures()


    def plot_experiment(self, averaged_metrics, data_robot, participants, conditions):
        # Human torque vs robot torque
        # data_pd = pd.DataFrame(data)
        mean_metrics_pd = pd.DataFrame(averaged_metrics)

        strategy = Strategy()
        inputs = strategy.run()
        Qh1 = inputs["cost_human_pos"]
        Qh2 = inputs["cost_human_neg"]
        Qr1 = inputs["cost_robot_pos"]
        Lr1 = inputs["gain_robot_pos"]
        Lh = inputs["gain_human"]
        Qr2 = inputs["cost_robot_neg"]
        Lr2 = inputs["gain_robot_neg"]
        C1 = inputs["auth_pos"]
        C2 = inputs["auth_neg"]


        # Let's go chronologically
        # 1. Costs
        # 1a. Human cost vs robot cost
        fig1 = plt.figure()
        fig2 = plt.figure()
        fig3 = plt.figure()
        figa = plt.figure()
        axs1 = [fig1.gca(), fig2.gca(), fig3.gca(), figa.gca()]
        # f1, axs1 = plt.subplots(1, 3)

        # 1b. System cost vs Performance
        fig4 = plt.figure()
        fig5 = plt.figure()
        fig6 = plt.figure()
        figb = plt.figure()
        axs2 = [fig4.gca(), fig5.gca(), fig6.gca(), figb.gca()]

        # 2a. Human gains vs robot gains
        fig7 = plt.figure()
        fig8 = plt.figure()
        fig9 = plt.figure()
        figc = plt.figure()
        axs3 = [fig7.gca(), fig8.gca(), fig9.gca(), figc.gca()]

        # 2b. System gain vs performance
        fig10 = plt.figure()
        fig11 = plt.figure()
        fig12 = plt.figure()
        figd = plt.figure()
        axs4 = [fig10.gca(), fig11.gca(), fig12.gca(), figd.gca()]

        # 3a. System gain vs performance
        fig13 = plt.figure()
        fig14 = plt.figure()
        fig15 = plt.figure()
        fige = plt.figure()
        axs5 = [fig13.gca(), fig14.gca(), fig15.gca(), fige.gca()]

        for i in range(participants):
            participant_data = mean_metrics_pd.loc[mean_metrics_pd["participant"] == i]
            data_manual = participant_data.loc[participant_data["condition"] == "Manual Control"]
            data_negative = participant_data.loc[participant_data["condition"] == "Negative Reinforcement"]
            data_positive = participant_data.loc[participant_data["condition"] == "Positive Reinforcement"]
            performance_dict_manual = data_manual["performance"]
            key = list(performance_dict_manual.keys())[0]
            performance_manual = performance_dict_manual[key]
            color = self.tud_blue
            condition = ""

            for j in range(conditions):
                if j == 0:
                    data_now = data_manual.to_dict()
                    if i % 4 == 0:
                        condition = "Manual Control"
                elif j == 1:
                    data_now = data_negative.to_dict()
                    if i % 4 == 0:
                        condition = "Negative Reinforcement"
                    if i == 0:
                        axs1[j].plot(Qh1[:, 0, 0], Qr1[:, 0, 0], color, label="Design", linewidth=self.linewidth, alpha=0.5)
                        axs3[j].plot(Lh[:, 0], Lr1[:, 0], color, label="Design", linewidth=self.linewidth, alpha=0.5)
                else:
                    data_now = data_positive.to_dict()
                    if i % 4 == 0:
                        condition = "Positive Reinforcement"
                    if i == 0:
                        axs1[j].plot(Qh2[:, 0, 0], Qr2[:, 0, 0], color, label="Design", linewidth=self.linewidth, alpha=0.5)
                        axs3[j].plot(Lh[:, 0], Lr2[:, 0], color, label="Design", linewidth=self.linewidth, alpha=0.5)

                performance_dict = data_now["performance"]
                key = list(performance_dict.keys())[0]
                # print(key)
                performance = performance_dict[key]
                standard = 100
                size = int(round(standard + 1000/performance, 1))
                print("size = ", size)
                label = "Participant " + str(i)

                # Figure 1a.
                axs1[j].scatter(data_now["cost_human"][key], data_now["cost_robot"][key], s=size, label=label)
                axs1[j].set_xlim(0, 20)
                axs1[j].set_ylim(0, 20)
                axs1[j].legend()
                # axs1[j].axis('equal')
                axs1[j].set_xlabel("Averaged Estimated Human Cost Weight (Nm/rad)", **self.hfont)
                axs1[j].set_ylabel("Averaged Robot Cost Weight (-)", **self.hfont)
                axs1[j].set_title(data_now['condition'][key], **self.csfont)

                # print(self.colormap[j], axs1)

                axs1[3].scatter(data_now["cost_human"][key], data_now["cost_robot"][key], color=self.colormap[j], s=size, label=condition)
                axs1[3].set_xlim(0, 20)
                axs1[3].set_ylim(0, 20)
                if j == 0:
                    axs1[3].legend()
                # axs1[j].axis('equal')
                axs1[3].set_xlabel("Averaged Estimated Human Cost Weight (Nm/rad)", **self.hfont)
                axs1[3].set_ylabel("Averaged Robot Cost Weight (-)", **self.hfont)
                axs1[3].set_title("Comparison", **self.csfont)

                # Figure 1b.
                axs2[j].scatter(data_now["cost_system"][key], data_now["performance"][key], s=size, label=label)
                axs2[j].set_xlim(0, 30)
                axs2[j].set_ylim(0, 10)
                axs2[j].legend()
                # axs2[j].axis('equal')
                axs2[j].set_xlabel("Total Estimated System Cost Weight (-)", **self.hfont)
                axs2[j].set_ylabel("RMS Error ($^{\circ}$)", **self.hfont)
                axs2[j].set_title(data_now['condition'][key], **self.csfont)

                axs2[3].scatter(data_now["cost_system"][key], data_now["performance"][key], color=self.colormap[j], s=size, label=condition)
                axs2[3].set_xlim(0, 30)
                axs2[3].set_ylim(0, 10)
                if j == 0:
                    axs2[3].legend()
                # axs2[j].axis('equal')
                axs2[3].set_xlabel("Total Estimated System Cost Weight (-)", **self.hfont)
                axs2[3].set_ylabel("RMS Error ($^{\circ}$)", **self.hfont)
                axs2[3].set_title("Comparison", **self.csfont)

                # Figure 2a.
                axs3[j].scatter(data_now["gain_human"][key], data_now["gain_robot"][key], s=size, label=label)
                axs3[j].set_ylim(0, 6)
                axs3[j].set_xlim(-2, 4)
                axs3[j].legend()
                axs3[j].set_xlabel("Averaged Estimated Human Gain (Nm/rad)", **self.hfont)
                axs3[j].set_ylabel("Averaged Robot Gain (Nm/rad)", **self.hfont)
                axs3[j].set_title(data_now['condition'][key], **self.csfont)

                axs3[3].scatter(data_now["gain_human"][key], data_now["gain_robot"][key], color=self.colormap[j], s=size, label=condition)
                axs3[3].set_ylim(0, 6)
                axs3[3].set_xlim(-2, 4)
                if j == 0:
                    axs3[3].legend()
                axs3[3].set_xlabel("Averaged Estimated Human Gain (Nm/rad)", **self.hfont)
                axs3[3].set_ylabel("Averaged Robot Gain (Nm/rad)", **self.hfont)
                axs3[3].set_title("Comparison", **self.csfont)

                # Figure 2b.
                axs4[j].scatter(data_now["gain_system"][key], data_now["performance"][key], marker="o", s=size, label=label)
                robot_label = ""
                human_label = ""
                if i == 0:
                    robot_label = "Robot gain"
                    human_label = "Human gain"
                    axs4[j].plot(data_robot["robot_angle_gain"], data_robot["rms_angle_error"], self.tud_blue,
                                 linewidth=self.linewidth, alpha=0.5, label="Optimal Control")
                axs4[j].set_xlim(0, 10)
                axs4[j].set_ylim(0, 10)
                axs4[j].legend()
                axs4[j].set_xlabel("Total Estimated System Gain (Nm/rad)", **self.hfont)
                axs4[j].set_ylabel("RMS Error ($^{\circ}$)", **self.hfont)
                axs4[j].set_title(data_now['condition'][key], **self.csfont)

                axs4[3].scatter(data_now["gain_system"][key], data_now["performance"][key], marker="o", color=self.colormap[j], s=size, label=condition)
                axs4[3].scatter(data_now["gain_human"][key], data_now["performance"][key], marker="p", color=self.colormap[j], edgecolor='black', s=size,
                                label=human_label)
                axs4[3].scatter(data_now["gain_robot"][key], data_now["performance"][key], marker="v",
                                color=self.colormap[j], edgecolor='black', s=size,
                                label=robot_label)
                if j == 0 and i == 0:
                    axs4[3].plot(data_robot["robot_angle_gain"], data_robot["rms_angle_error"], self.tud_blue,
                                 linewidth=self.linewidth, alpha=0.5, label="Optimal Control")
                axs4[3].set_xlim(0, 10)
                axs4[3].set_ylim(0, 10)
                if j == 0:
                    axs4[3].legend()
                axs4[3].set_xlabel("Total Estimated System Gain (Nm/rad)", **self.hfont)
                axs4[3].set_ylabel("RMS Error ($^{\circ}$)", **self.hfont)
                axs4[3].set_title("Comparison", **self.csfont)

                # Figure 3a.
                axs5[j].scatter(data_now["human_power"][key], data_now["robot_power"][key], s=size, label=label)
                axs5[j].set_xlim(0, 0.7)
                axs5[j].set_ylim(0, 0.7)
                axs5[j].legend()
                axs5[j].set_xlabel("RMS Estimated Human Power (W)", **self.hfont)
                axs5[j].set_ylabel("RMS Robot Power (W)", **self.hfont)
                axs5[j].set_title(data_now['condition'][key], **self.csfont)

                axs5[3].scatter(data_now["human_power"][key], data_now["robot_power"][key], color=self.colormap[j], s=size, label=condition)
                axs5[3].set_xlim(0, 0.7)
                axs5[3].set_ylim(0,0.7)
                if j == 0:
                    axs5[3].legend()
                axs5[3].set_xlabel("RMS Estimated Human Power (W)", **self.hfont)
                axs5[3].set_ylabel("RMS Robot Power (W)", **self.hfont)
                axs5[3].set_title("Comparison", **self.csfont)

    def plot_participant(self, raw_data, trials, participant):
        figb, axs = plt.subplots(4, 2)
        figc, axsb = plt.subplots(4, 3)

        for i in range(trials):
            data = raw_data[participant][i]
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
            error = (np.array(x) - np.array(r)) * 180 / np.pi

            x_hat = data["state_estimate_pos"]
            xdot = data["steering_rate"]
            Lhhat_pos = data["estimated_human_gain_pos"]
            Lhhat_vel = data["estimated_human_gain_vel"]
            Lr_pos = data["robot_gain_pos"]
            Lr_vel = data["robot_gain_vel"]
            Qhhat_pos = data["estimated_human_cost_1"]
            Qr_pos = data["robot_cost_pos"]

            Phhat = np.array(uhhat) * np.array(xdot)

            uhtilde = data["input_estimation_error"]
            uh_rec = np.array(uhhat) - np.array(uhtilde)

            Pr = np.array(ur) * np.array(xdot)
            Ph = np.array(uhhat) * np.array(xdot)
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

            t_start = 0
            t_end = t[-1]
            t_example = t_end

            # Plot time data
            if repetition == 0:
                # fig, axa = plt.subplots(3, 4)
                fig, axa = plt.subplots(4, 2, gridspec_kw={'width_ratios': [5, 1]})

                fig.suptitle(condition, **self.csfont)
                axa[0, 0].stackplot(t, Lhhat_pos, Lr_pos, colors=colors, labels=labels, edgecolor='black', linewidth=0.8)
                axa[0, 0].set_ylabel('Gain (Nm/rad)', **self.hfont_small)
                axa[0, 0].legend(prop={"size": 8}, loc='upper left')
                axa[0, 0].set_xticks([])
                axa[0, 0].set_xlim(t_start, t_example)
                Lt = np.array(Lhhat_pos) + np.array(Lr_pos)
                ymin_t = min(Lhhat_pos)
                ymax_t = max(Lt)
                axa[0, 0].set_ylim(ymin_t, ymax_t)

                data_gains = pd.DataFrame({"gains": Lhhat_pos})
                sns.histplot(data=data_gains, y="gains", ax=axa[0, 1], kde=True, color=self.tud_red, binwidth=0.1)
                axa[0, 1].set_xticks([])
                axa[0, 1].set_ylabel("")
                axa[0, 1].set_xlabel("")
                axa[0, 1].set_title("Distribution", **self.hfont)
                axa[0, 1].set_ylim(ymin_t, ymax_t)
                med = np.median(Lhhat_pos)
                ymin = min(Lhhat_pos)
                ymax = max(Lhhat_pos)
                axa[0, 1].set_yticks([ymin, med,  ymax])
                axa[0, 1].yaxis.set_major_formatter(FormatStrFormatter('%.1f'))


                axa[1, 0].plot(t, error, self.tud_blue)
                axa[1, 0].set_ylabel('Error ($^{\circ}$)', **self.hfont_small)
                axa[1, 0].set_xticks([])
                axa[1, 0].set_xlim(t_start, t_example)

                data_error = pd.DataFrame({"RMSE": error})
                sns.histplot(data=data_error, y="RMSE", ax=axa[1, 1], kde=True, color=self.tud_blue)
                axa[1, 1].set_yticks([])
                axa[1, 1].set_xticks([])
                axa[1, 1].set_ylabel("")
                axa[1, 1].set_xlabel("")
                ymin = min(error)
                ymax = max(error)
                med = np.median(error)
                axa[1, 1].set_yticks([ymin, med, ymax])
                axa[1, 1].yaxis.set_major_formatter(FormatStrFormatter('%.1f'))


                axa[2, 0].stackplot(t, uhhat, ur, -np.array(uhtilde), colors=[self.tud_red, self.tud_blue, self.tud_orange],
                              labels=['Estimated Human', 'Robot', 'Estimation Error'], edgecolor='black', linewidth=0.8)
                axa[2, 0].set_ylabel('Torque (Nm)', **self.hfont_small)
                axa[2, 0].legend(prop={"size": 8}, loc='upper left')
                axa[2, 0].set_xlim(t_start, t_example)
                axa[2, 0].set_xticks([])

                plt.figure()
                plt.stackplot(t, uhhat, ur, -np.array(uhtilde),
                                    colors=[self.tud_red, self.tud_blue, self.tud_orange],
                                    labels=['Estimated Human', 'Robot', 'Estimation Error'], edgecolor='black',
                                    linewidth=0.2)
                plt.ylabel('Torque (Nm)', **self.hfont_small)
                plt.legend(prop={"size": 8}, loc='upper left')
                plt.xlim(t_start, 30)
                plt.xlabel("Time (s)")

                data_inputs = pd.DataFrame({"Input torque": uhhat})
                sns.histplot(data=data_inputs, y=uhhat, ax=axa[2, 1], kde=True, color=self.tud_red)
                axa[2, 1].set_yticks([])
                axa[2, 1].set_xticks([])
                axa[2, 1].set_ylabel("")
                axa[2, 1].set_xlabel("")
                ymin = min(uhhat)
                ymax = max(uhhat)
                med = np.median(error)
                axa[2, 1].set_yticks([ymin, med, ymax])
                axa[2, 1].yaxis.set_major_formatter(FormatStrFormatter('%.1f'))

                Qt = np.array(Qhhat_pos) + np.array(Qr_pos)
                ymin_t = min(Qhhat_pos)
                ymax_t = max(Qt)

                axa[3, 0].stackplot(t, Qhhat_pos, Qr_pos, colors=colors, labels=labels, edgecolor='black',
                                    linewidth=0.8)
                axa[3, 0].set_ylabel('Cost Weight (-)', **self.hfont_small)
                axa[3, 0].legend(prop={"size": 8}, loc='upper left')
                axa[3, 0].set_ylim(ymin_t, ymax_t)
                axa[3, 0].set_xlim(t_start, t_example)
                axa[3, 0].set_xlabel("Time (s)")

                data_gains = pd.DataFrame({"costs": Qhhat_pos})
                sns.histplot(data=data_gains, y="costs", ax=axa[3, 1], kde=True, color=self.tud_red)
                axa[3, 1].set_yticks([])
                axa[3, 1].set_xticks([])
                axa[3, 1].set_ylim(ymin_t, ymax_t)
                axa[3, 1].set_ylabel("")
                axa[3, 1].set_xlabel("")
                ymin = min(Qhhat_pos)
                ymax = max(Qhhat_pos)
                med = np.median(data_gains)
                axa[3, 1].set_yticks([ymin, med, ymax])
                axa[3, 1].yaxis.set_major_formatter(FormatStrFormatter('%.1f'))

                plt.tight_layout(pad=1)

            auth_est = (np.array(Lr_pos) - np.array(Lhhat_pos)) / (np.array(Lr_pos) + np.array(Lhhat_pos) + 0.001)
            figb.suptitle("Estimated Control Authority", **self.csfont)
            if c > 0:
                axs[repetition, c - 1].plot(t, auth_est, label='Estimated authority')
                # axs[repetition, c - 1].plot(t, auth, alpha=0.3, label='Measured authority')
                axs[repetition, c - 1].set_ylim(-1.5, 1.5)
                axs[repetition, c - 1].set_xlim(t_start, t_end)
                if repetition == 0:
                    axs[repetition, c - 1].set_title(title)
                    axs[repetition, c - 1].set_xticks([])
                    # axs[repetition, c - 1].legend(prop={"size": 8}, loc='lower right')
                elif repetition == 3:
                    axs[repetition, c - 1].set_xlabel('Time (s)', **self.hfont)
                else:
                    axs[repetition, c - 1].set_xticks([])
            # plt.tight_layout(pad=1)

            figc.suptitle("Gains", **self.csfont)
            axsb[repetition, c].stackplot(t, Lhhat_pos, Lr_pos, baseline='zero', colors=colors,
                                              edgecolor='black', linewidth=0.8, labels=labels)
            axsb[repetition, c].set_ylim(-1, 12)
            axsb[repetition, c].set_xlim(t_start, t_end)
            if repetition == 0:
                axsb[repetition, c].set_title(title)
                axsb[repetition, c].set_xticks([])
                axsb[repetition, c].legend(prop={"size": 6}, loc='upper left')
            if repetition == 3:
                axsb[repetition, c].set_xlabel('Time (s)', **self.hfont)
            else:
                axsb[repetition, c].set_xticks([])
            # plt.tight_layout(pad=1)


