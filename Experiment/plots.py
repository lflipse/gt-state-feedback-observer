import matplotlib.pyplot as plt
from matplotlib.markers import MarkerStyle
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
        self.legend_size = 15
        self.linewidth = 4
        self.legend_size_small = 8
        self.legend_size_large = 12
        self.csfont = {'fontname': 'Georgia', 'size': title_size}
        self.hfont = {'fontname': 'Georgia', 'size': label_size_big}
        self.hfont_small = {'fontname': 'Georgia', 'size': label_size_small}
        self.legend_font_small = {'family': 'Georgia', 'size': self.legend_size_small}
        self.legend_font_large = {'family': 'Georgia', 'size': self.legend_size_large}

        # Colors
        self.tud_black = "#000000"
        self.tud_blue = "#0066A2"
        self.tud_red = "#c3312f"
        self.tud_green = "#99D28C"
        self.tud_yellow = "#F1BE3E"
        self.tud_orange = "#EB7245"
        self.tud_lightblue = "#61A4B4"
        self.tud_otherblue = "#007188"
        self.tud_green2 = "#00A390"

        self.colors = {"Manual Control": self.tud_orange, "Negative Reinforcement": self.tud_otherblue,
                       "Positive Reinforcement": self.tud_green,}
        self.colors2 = {"Negative Reinforcement": self.tud_otherblue,
                       "Positive Reinforcement": self.tud_green, }

        self.order = ["Manual Control", "Negative Reinforcement", "Positive Reinforcement"]

        self.colormap = [self.tud_orange, self.tud_otherblue, self.tud_green, self.tud_blue, self.tud_red, self.tud_lightblue, self.tud_yellow]
        self.colormap2 = [self.tud_lightblue, self.tud_red, self.tud_green]

        # sns.set_palette(sns.color_palette(self.colormap))
        sns.color_palette("Spectral", as_cmap=True)

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
        # print(metrics_pd)
        metrics_pd_no_manual = metrics_pd.loc[metrics_pd["condition"] != "Manual Control"]
        # print(metrics_pd_no_manual["condition"] == "Manual Control")
        metrics_part = metrics_pd.loc[metrics_pd["participant"] == participant]
        metrics_no_manual = metrics_part.loc[metrics_part["condition"] != "Manual Control"]
        metrics_positive_uns = metrics_pd.loc[metrics_pd["condition"] == "Positive Reinforcement"]
        metrics_negative_uns = metrics_pd.loc[metrics_pd["condition"] == "Negative Reinforcement"]
        metrics_manual_uns = metrics_pd.loc[metrics_pd["condition"] == "Manual Control"]
        metrics_positive = metrics_positive_uns.sort_values(by=['trial'])
        metrics_negative = metrics_negative_uns.sort_values(by=['trial'])
        metrics_manual = metrics_manual_uns.sort_values(by=['trial'])

        labels = ["Manual \n Control", "Negative \n Reinforcement", "Positive \n Reinforcement"]

        # Control authority
        figa, axa = plt.subplots(1, 2, gridspec_kw={'width_ratios': [5, 1]})
        figa.suptitle("Estimated control Share", **self.csfont)
        sns.boxplot(data=metrics_pd_no_manual, x="condition", y="authority", palette=self.colors2, ax=axa[0], order=self.order[1:3])
        sns.swarmplot(data=metrics_pd_no_manual, x="condition", y="authority", palette=self.colors2, ax=axa[0], order=self.order[1:3],
                      alpha=0.6, size=6, linewidth=1)
        axa[0].set_xticklabels(labels[1:3])
        axa[0].set_xlabel("")
        # self.annotate_significance(2, axa[0], ["***"])
        ylims = axa[0].get_ylim()
        axa[0].set_ylabel("Estimated control share (-)", **self.hfont)
        delta_y = ylims[1] - ylims[0]
        width = delta_y / 40
        sns.histplot(data=metrics_pd_no_manual, y="authority", hue="condition", ax=axa[1],
                     palette=self.colors2, binwidth=width)
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
        axc[0].set_ylabel("RMS power ($W$)", **self.hfont)
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

        # sns.set_palette(sns.color_palette(self.colormap))
        sns.set_palette("Spectral")

        # Differences in control strategy
        plt.figure()
        sns.boxplot(data=metrics_pd, x="participant", y="human_angle_cost", hue="condition", palette=self.colors, hue_order=self.order,)
        plt.title("Human control strategy", **self.csfont)
        plt.tight_layout(pad=1)


        fig1 = plt.figure().gca()
        try:
            fa = sns.scatterplot(data=metrics_positive, x="repetition", y="human_angle_gain", hue="label", ax=fig1,palette=self.colormap2[0:3],
                              s=60)
            fb = sns.lineplot(data=metrics_positive, x="repetition", y="human_angle_gain", hue="label", ax=fig1, estimator=None, lw=2.5, units="participant", palette=self.colormap2[0:3],)
        except:
            fa = sns.scatterplot(data=metrics_positive, x="repetition", y="human_angle_gain", hue="label", ax=fig1,
                                 palette=self.colormap2[0:2],
                                 s=60)
            fb = sns.lineplot(data=metrics_positive, x="repetition", y="human_angle_gain", hue="label", ax=fig1,
                              estimator=None, lw=2.5, units="participant", palette=self.colormap2[0:2], )

        plt.title("Positive adaptation strategy", **self.csfont)
        fig1.set_ylabel("Human steering angle gain ($Nm/^\circ$)", **self.hfont)
        fig1.set_xlabel("Trial number", **self.hfont)
        fig1.set_xticks([1, 2, 3, 4])
        # fig1.get_legend().remove()
        plt.tight_layout(pad=1)

        fig2 = plt.figure().gca()
        try:
            ga = sns.scatterplot(data=metrics_negative, x="repetition", y="human_angle_gain", hue="label", ax=fig2,palette=self.colormap2[0:3],
                              s=60)
            gb = sns.lineplot(data=metrics_negative, x="repetition", y="human_angle_gain", hue="label", ax=fig2, estimator=None, lw=2.5, units="participant",palette=self.colormap2[0:3],)
        except:
            ga = sns.scatterplot(data=metrics_negative, x="repetition", y="human_angle_gain", hue="label", ax=fig2,
                             palette=self.colormap2[0:2],
                             s=60)
            gb = sns.lineplot(data=metrics_negative, x="repetition", y="human_angle_gain", hue="label", ax=fig2,
                          estimator=None, lw=2.5, units="participant", palette=self.colormap2[0:2], )

        plt.title("Negative adaptation strategy", **self.csfont)
        fig2.set_ylabel("Human steering angle gain ($Nm/^\circ$)", **self.hfont)
        fig2.set_xlabel("Trial number", **self.hfont)
        fig2.set_xticks([1, 2, 3, 4])
        # fig2.get_legend().remove()
        plt.tight_layout(pad=1)

        fig3 = plt.figure().gca()
        try:
            ha = sns.scatterplot(data=metrics_manual, x="repetition", y="human_angle_gain", hue="label", ax=fig3,palette=self.colormap2[0:3],
                              s=60)
            hb = sns.lineplot(data=metrics_manual, x="repetition", y="human_angle_gain", hue="label", ax=fig3, estimator=None, lw=2.5, units="participant",palette=self.colormap2[0:3],)
        except:
            ha = sns.scatterplot(data=metrics_manual, x="repetition", y="human_angle_gain", hue="label", ax=fig3,
                                 palette=self.colormap2[0:2],
                                 s=60)
            hb = sns.lineplot(data=metrics_manual, x="repetition", y="human_angle_gain", hue="label", ax=fig3,
                              estimator=None, lw=2.5, units="participant", palette=self.colormap2[0:2], )

        plt.title("Manual control", **self.csfont)
        fig3.set_ylabel("Human steering angle gain ($Nm/^\circ$)", **self.hfont)
        fig3.set_xlabel("Trial number", **self.hfont)
        fig3.set_xticks([1, 2, 3, 4])
        # fig3.get_legend().remove()
        plt.tight_layout(pad=1)

        # metrics_pd.loc[metrics_pd["condition"] == "Positive Reinforcement"]

        # Positive reinforcement
        fig4 = plt.figure().gca()
        plt.title("Positive reinforcement", **self.csfont)
        try:
            ha = sns.scatterplot(data=metrics_positive, x="gains_normalized", y="error_normalized", hue="label", ax=fig4,
                                 palette=self.colormap2[0:3], size="repetition", sizes=(20, 80))
            hc = sns.scatterplot(data=metrics_manual, x="gains_normalized", y="error_normalized", hue="label",
                                 ax=fig4,
                                 palette=self.colormap2[0:3], size="repetition", sizes=(20, 80))
            hb = sns.lineplot(data=metrics_positive, x="gains_normalized", y="error_normalized", hue="label", ax=fig4,
                              estimator=None, lw=1, units="participant", palette=self.colormap2[0:3], sort=False)
        except:
            ha = sns.scatterplot(data=metrics_positive, x="gains_normalized", y="error_normalized", hue="label", ax=fig4,
                                 palette=self.colormap2[0:2], size="repetition", sizes=(10, 40))
            hc = sns.scatterplot(data=metrics_manual, x="gains_normalized", y="error_normalized", hue="label",
                                 ax=fig4,
                                 palette=self.colormap2[0:2], size="repetition", sizes=(20, 80))
            hb = sns.lineplot(data=metrics_positive, x="gains_normalized", y="error_normalized", hue="label", ax=fig4,
                              estimator=None, lw=1, units="participant", palette=self.colormap2[0:2], sort=False)
        # Positive reinforcement
        fig5 = plt.figure().gca()
        plt.title("Negative reinforcement", **self.csfont)
        try:
            ha = sns.scatterplot(data=metrics_negative, x="gains_normalized", y="error_normalized", hue="label", ax=fig5,
                                 palette=self.colormap2[0:3], size="repetition", sizes=(10, 40))
            hb = sns.lineplot(data=metrics_negative, x="gains_normalized", y="error_normalized", hue="label", ax=fig5,
                              estimator=None, lw=1, units="participant", palette=self.colormap2[0:3], sort=False)
        except:
            ha = sns.scatterplot(data=metrics_negative, x="gains_normalized", y="error_normalized", hue="label", ax=fig5,
                                 palette=self.colormap2[0:2], size="repetition", sizes=(10, 40))
            hb = sns.lineplot(data=metrics_negative, x="gains_normalized", y="error_normalized", hue="label", ax=fig5,
                              estimator=None, lw=1, units="participant", palette=self.colormap2[0:2], sort=False)

        # Positive reinforcement
        fig6 = plt.figure().gca()
        plt.title("Negative reinforcement", **self.csfont)
        try:
            ha = sns.scatterplot(data=metrics_negative, x="system_angle_gain", y="rms_angle_error", hue="label",
                                 ax=fig6,
                                 palette=self.colormap2[0:3], size="repetition", sizes=(10, 40))
            hb = sns.lineplot(data=metrics_negative, x="system_angle_gain", y="rms_angle_error", hue="label",
                              ax=fig6,
                              estimator=None, lw=1, units="participant", palette=self.colormap2[0:3], sort=False)
        except:
            ha = sns.scatterplot(data=metrics_negative, x="system_angle_gain", y="rms_angle_error", hue="label",
                                 ax=fig6,
                                 palette=self.colormap2[0:2], size="repetition", sizes=(10, 40))
            hb = sns.lineplot(data=metrics_negative, x="system_angle_gain", y="rms_angle_error", hue="label",
                              ax=fig6,
                              estimator=None, lw=1, units="participant", palette=self.colormap2[0:2], sort=False)

        fig7 = plt.figure().gca()
        plt.title("Effect of human cost function on team performance", **self.csfont)
        ha = sns.scatterplot(data=metrics_pd, x="human_angle_cost", y="rms_angle_error", hue="condition",
                             ax=fig7,
                             palette=self.colors, s=40) #size="repetition",
        # hb = sns.lineplot(data=metrics_pd, x="human_angle_cost", y="rms_angle_error", hue="condition", ax=fig7,
        #                   estimator=None, lw=0.5, units="participant", palette=self.colormap2[0:3], sort=False)
        fig7.set_ylabel("RMS steering error ($^\circ$)", **self.hfont)
        fig7.set_xlabel("Estimated human cost function weight (-)", **self.hfont)
        fig7.legend(title="Condition")
        plt.tight_layout(pad=1)


        fig4.set_ylabel("delta error", **self.hfont)
        fig4.set_xlabel("delta gain", **self.hfont)
        # fig3.get_legend().remove()
        plt.tight_layout(pad=1)


        fig5.set_ylabel("delta error", **self.hfont)
        fig5.set_xlabel("delta gain", **self.hfont)
        # fig3.get_legend().remove()
        plt.tight_layout(pad=1)

        # self.save_all_figures()


    def plot_experiment(self, averaged_metrics, data_robot, participants, conditions):
        # Human torque vs robot torque
        # data_pd = pd.DataFrame(data)
        mean_metrics_pd = pd.DataFrame(averaged_metrics)

        # strategy = Strategy()
        # inputs = strategy.run()
        # Qh1 = inputs["cost_human_pos"]
        # Qh2 = inputs["cost_human_neg"]
        # Qr1 = inputs["cost_robot_pos"]
        # Lr1 = inputs["gain_robot_pos"]
        # Lh1 = inputs["gain_human_pos"]
        # Qr2 = inputs["cost_robot_neg"]
        # Lr2 = inputs["gain_robot_neg"]
        # C1 = inputs["auth_pos"]
        # C2 = inputs["auth_neg"]


        # Let's go chronologically
        # # 1. Costs
        # # 1a. Human cost vs robot cost
        # fig1 = plt.figure()
        # fig2 = plt.figure()
        # fig3 = plt.figure()
        # figa = plt.figure()
        # axs1 = [fig1.gca(), fig2.gca(), fig3.gca(), figa.gca()]
        # # f1, axs1 = plt.subplots(1, 3)
        #
        # # 1b. System cost vs Performance
        # fig4 = plt.figure()
        # fig5 = plt.figure()
        # fig6 = plt.figure()
        # figb = plt.figure()
        # axs2 = [fig4.gca(), fig5.gca(), fig6.gca(), figb.gca()]
        #
        # # 2a. Human gains vs robot gains
        # fig7 = plt.figure()
        # fig8 = plt.figure()
        # fig9 = plt.figure()
        # figc = plt.figure().gca()
        # axs3 = [fig7.gca(), fig8.gca(), fig9.gca(), figc.gca()]

        # 2b. System gain vs performance
        # fig10 = plt.figure()
        # fig11 = plt.figure()
        # fig12 = plt.figure()
        figd = plt.figure().gca()
        # axs4 = [fig10.gca(), fig11.gca(), fig12.gca(), figd.gca()]

        # 3a. System gain vs performance
        # fig13 = plt.figure()
        # fig14 = plt.figure()
        # fig15 = plt.figure()
        fige = plt.figure().gca()
        # axs5 = [fig13.gca(), fig14.gca(), fig15.gca(), fige.gca()]

        data_manual_pd = mean_metrics_pd.loc[mean_metrics_pd["condition"] == "Manual Control"]
        data_negative_pd = mean_metrics_pd.loc[mean_metrics_pd["condition"] == "Negative Reinforcement"]
        data_positive_pd = mean_metrics_pd.loc[mean_metrics_pd["condition"] == "Positive Reinforcement"]
        metrics_pd_no_manual = mean_metrics_pd.loc[mean_metrics_pd["condition"] != "Manual Control"]

        labels = ["Manual \n Control", "Negative \n Reinforcement", "Positive \n Reinforcement"]

        # Control authority
        figa, axa = plt.subplots(1, 2, gridspec_kw={'width_ratios': [5, 1]})
        figa.suptitle("Trial-by-trial variability", **self.csfont)
        sns.boxplot(data=mean_metrics_pd, x="condition", y="cost_human_var", palette=self.colors, ax=axa[0], order=self.order)
        sns.swarmplot(data=mean_metrics_pd, x="condition", y="cost_human_var", palette=self.colors, ax=axa[0], order=self.order,
                      alpha=0.6, size=6, linewidth=1)
        axa[0].set_xticklabels(labels)
        axa[0].set_xlabel("")
        # self.annotate_significance(2, axa[0], ["***"])
        ylims = axa[0].get_ylim()
        axa[0].set_ylabel("Cost function weight (-)", **self.hfont)
        delta_y = ylims[1] - ylims[0]
        width = delta_y / 40
        sns.histplot(data=mean_metrics_pd, y="cost_human_var", hue="condition", ax=axa[1],
                     palette=self.colors, binwidth=width)
        axa[1].get_legend().remove()
        axa[1].axes.get_yaxis().set_visible(False)
        plt.tight_layout(pad=1)
        axa[0].set_ylim(0, 40)
        axa[1].set_ylim(0, 40)

        # Subjective control authority
        figb, axb = plt.subplots(1, 2, gridspec_kw={'width_ratios': [5, 1]})
        figb.suptitle("Perceived control share", **self.csfont)
        sns.boxplot(data=metrics_pd_no_manual, x="condition", y="subjective_authority", palette=self.colors2, ax=axb[0],
                    order=self.order[1:3])
        sns.swarmplot(data=metrics_pd_no_manual, x="condition", y="subjective_authority", palette=self.colors2, ax=axb[0],
                      order=self.order[1:3],
                      alpha=0.6, size=6, linewidth=1)
        axb[0].set_xticklabels(labels[1:3])
        axb[0].set_xlabel("")
        # self.annotate_significance(2, axa[0], ["***"])
        ylims = axb[0].get_ylim()
        axb[0].set_ylabel("Perceived control share (-)", **self.hfont)

        delta_y = ylims[1] - ylims[0]
        width = delta_y / 40
        sns.histplot(data=metrics_pd_no_manual, y="subjective_authority", hue="condition", ax=axb[1],
                     palette=self.colors2, binwidth=width)
        axb[1].get_legend().remove()
        axb[1].axes.get_yaxis().set_visible(False)
        plt.tight_layout(pad=1)


        h = sns.jointplot(data=mean_metrics_pd, x="authority", y="subjective_authority", hue="condition",
                      palette=self.colors, hue_order=self.order, kind="scatter", joint_kws={"s":100})
        h.fig.suptitle("The interaction from two perspectives", **self.csfont)
        # jp.fig.legend(fontsize=self.legend_size, title=[])
        h.ax_joint.set_xlabel("Robot: Estimated control share", **self.hfont)
        h.ax_joint.set_ylabel("Human: Perceived control share", **self.hfont)
        h.ax_joint.legend(loc="lower right", title=[])
        ticks = [-2, -1, 0, 1]
        labels = ["Competition", "Robot Only", "Cooperation", "Human Only"]
        h.ax_joint.set_xticks(ticks)
        h.ax_joint.set_xticklabels(labels)
        h.ax_joint.set_yticks(ticks)
        h.ax_joint.set_yticklabels(labels)
        h.ax_joint.set_xlim(-2.2, 1.7)
        h.ax_joint.set_ylim(-2.2, 1.7)
        h.ax_joint.legend(title="Condition")
        h.fig.tight_layout()



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
                    # if i == 0:
                        # axs1[j].plot(Qh1[:, 0, 0], Qr1[:, 0, 0], color, label="Design", linewidth=self.linewidth, alpha=0.5)
                        # axs3[j].plot(Lh[:, 0], Lr1[:, 0], color, label="Design", linewidth=self.linewidth, alpha=0.5)
                else:
                    data_now = data_positive.to_dict()
                    if i % 4 == 0:
                        condition = "Positive Reinforcement"
                    # if i == 0:
                        # axs1[j].plot(Qh2[:, 0, 0], Qr2[:, 0, 0], color, label="Design", linewidth=self.linewidth, alpha=0.5)
                        # axs3[j].plot(Lh[:, 0], Lr2[:, 0], color, label="Design", linewidth=self.linewidth, alpha=0.5)

                performance_dict = data_now["performance"]
                key = list(performance_dict.keys())[0]
                # print(key)
                performance = performance_dict[key]
                standard = 50
                size = int(round(standard + 500/performance, 1))
                # print("size = ", size)
                label = "Participant " + str(i)



                marker_style = dict(color=self.tud_black, linestyle=':', marker='o',
                                    markersize=size, markerfacecoloralt='tab:white')
                fill_style = 'top'
                # print(data_now)
                auth = data_now["authority"][key]
                # lr = data_now["gain_robot"][key]

                markers = ["o", "v", "p"]

                if i == 0:
                    figd.scatter(-100, -100, s=size,
                                 edgecolor=self.tud_black,
                                 marker=markers[j], linewidths=0.2, facecolors=self.tud_black, label=condition)
                    figd.legend(prop=self.legend_font_large)

                im1 = figd.scatter(data_now["gain_system"][key], data_now["performance"][key], s=size, c=auth, edgecolor=self.tud_black,
                                 marker=markers[j], cmap="seismic", linewidths=0.2, vmin=-2, vmax=2)
                if j == 0 and i == 0:
                    cbar1 = plt.colorbar(im1, ax=figd)
                    # cbar1.set_label("Control Share", rotation=90, **self.hfont)
                    ticks = [-2, -1, 0, 1, 2]
                    labels = ["Competition", "Robot Only", "Cooperation", "Human Only", "Competition"]
                    cbar1.set_ticks(ticks)
                    cbar1.set_ticklabels(labels)
                    # cbar2 = plt.colorbar(im2, ax=figd)
                    # cbar2.set_label("Robot Gain", rotation=90, **self.hfont)

                plt.tight_layout()

                if j == 0 and i == 0:
                    figd.plot(np.array(data_robot["robot_angle_gain"]) *180/np.pi, data_robot["rms_angle_error"], self.tud_blue,
                                 linewidth=self.linewidth, alpha=0.7, label="Optimal Control",)

                figd.set_xlim(1*180/np.pi, 6*180/np.pi)
                figd.set_ylim(0, 13)
                if j == 0:
                    figd.legend(prop=self.legend_font_large)
                figd.set_title("Interaction mode, system gain and performance", **self.csfont)
                figd.set_xlabel("Total estimated system gain ($Nm/^\circ$)", **self.hfont)
                figd.set_ylabel("Mean RMS steering error ($^{\circ}$)", **self.hfont)


                # Figure 3a.
                # axs5[j].scatter(data_now["human_power"][key], data_now["robot_power"][key], s=size, label=label)
                # axs5[j].set_xlim(0, 0.7)
                # axs5[j].set_ylim(0, 0.7)
                # axs5[j].legend()
                # axs5[j].set_xlabel("RMS Estimated Human Power (W)", **self.hfont)
                # axs5[j].set_ylabel("RMS Robot Power (W)", **self.hfont)
                # axs5[j].set_title(data_now['condition'][key], **self.csfont)

                fige.scatter(data_now["human_power"][key], data_now["robot_power"][key], marker=markers[j], color=self.colormap[j], s=size, label=condition)
                fige.set_xlim(-0.05, 1.2)
                fige.set_ylim(-0.05, 1.2)
                if i == 0 and j == 0:
                    fige.plot([0.3, -0.3], [-0.3, 0.3], color=self.tud_blue, alpha=0.2, label="Constant System Power", linewidth=3)
                    fige.plot([0.6, -0.3], [-0.3, 0.6], color=self.tud_blue, alpha=0.2, linewidth=3)
                    fige.plot([0.9, -0.3], [-0.3, 0.9], color=self.tud_blue, alpha=0.2, linewidth=3)
                    fige.plot([1.2, -0.3], [-0.3, 1.2], color=self.tud_blue, alpha=0.2, linewidth=3)
                    fige.plot([1.5, -0.3], [-0.3, 1.5], color=self.tud_blue, alpha=0.2, linewidth=3)
                    fige.plot([1.8, -0.3], [-0.3, 1.8], color=self.tud_blue, alpha=0.2, linewidth=3)
                if i == 0:
                    fige.legend(prop=self.legend_font_large)
                fige.set_xlabel("RMS estimated human power ($W$)", **self.hfont)
                fige.set_ylabel("RMS robot power ($W$)", **self.hfont)
                fige.set_title("Relation between human and robot power", **self.csfont)



    def plot_participant(self, raw_data, trials, participant):
        print("plotting for participant ", participant)
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
            Lhhat_pos = np.array(data["estimated_human_gain_pos"]) *180/np.pi
            Lhhat_vel = np.array(data["estimated_human_gain_vel"]) * 180/np.pi
            Lr_pos = np.array(data["robot_gain_pos"]) *180/np.pi
            Lr_vel = np.array(data["robot_gain_vel"]) *180/np.pi
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
                title = "Manual"
                c = 0
            elif condition == "Positive Reinforcement":
                ls = '-'
                line_color = self.tud_red
                title = "Positive re."
                c = 1
            elif condition == "Negative Reinforcement":
                ls = '-'
                line_color = self.tud_blue
                title = "Negative re."
                c = 2
            elif condition == "Mixed Reinforcement":
                ls = '-'
                line_color = self.tud_green
                title = "Mixed re."
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
                stacks = axa[0, 0].stackplot(t, Lhhat_pos, Lr_pos, colors=colors, labels=labels, edgecolor='black', linewidth=0.8)
                hatches = ["\\\\", "//"]
                for stack, hatch in zip(stacks, hatches):
                    stack.set_hatch(hatch)

                axa[0, 0].set_ylabel('Gain ($Nm/^\circ$)', **self.hfont_small)
                axa[0, 0].legend(prop=self.legend_font_small, loc='upper left')
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


                stacks = axa[2, 0].stackplot(t, uhhat, ur, -np.array(uhtilde), colors=[self.tud_red, self.tud_blue, self.tud_orange],
                              labels=['Estimated Human', 'Robot', 'Estimation Error'], edgecolor='black', linewidth=0.2)

                hatches = ["\\\\", "//"]
                for stack, hatch in zip(stacks, hatches):
                    stack.set_hatch(hatch)
                axa[2, 0].set_ylabel('Torque ($Nm$)', **self.hfont_small)
                axa[2, 0].legend(prop=self.legend_font_small, loc='upper left')
                axa[2, 0].set_xlim(t_start, t_example)
                axa[2, 0].set_xticks([])

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

                stacks = axa[3, 0].stackplot(t, Qhhat_pos, Qr_pos, colors=colors, labels=labels, edgecolor='black',
                                    linewidth=0.8)
                hatches = ["\\\\", "//"]
                for stack, hatch in zip(stacks, hatches):
                    stack.set_hatch(hatch)
                axa[3, 0].set_ylabel('Cost weight (-)', **self.hfont_small)
                axa[3, 0].legend(prop=self.legend_font_small, loc='upper left')
                axa[3, 0].set_ylim(ymin_t, ymax_t)
                axa[3, 0].set_xlim(t_start, t_example)
                axa[3, 0].set_xlabel("Time ($s$)")

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

            auth_est = (np.array(Lhhat_pos) - np.array(Lr_pos)) / (np.array(Lr_pos) + np.array(Lhhat_pos) + 0.001)
            figb.suptitle("Estimated control share", **self.csfont)
            if c > 0:
                axs[repetition, c - 1].plot(t, auth_est)
                # axs[repetition, c - 1].plot(t, auth, alpha=0.3, label='Measured authority')
                axs[repetition, c - 1].set_ylim(-1.5, 1.5)
                axs[repetition, c - 1].set_xlim(t_start, t_end)
                if repetition == 0:
                    axs[repetition, c - 1].set_title(title)
                    axs[repetition, c - 1].set_xticks([])
                    # axs[repetition, c - 1].legend(prop={"size": 8}, loc='lower right')
                elif repetition == 3:
                    axs[repetition, c - 1].set_xlabel('Time ($s$)', **self.hfont)
                else:
                    axs[repetition, c - 1].set_xticks([])
            if c == 1:
                axs[repetition, c - 1].set_ylabel('Share (-)', **self.hfont_small)
            plt.tight_layout(pad=1)

            figc.suptitle("Controller gain distribution", **self.csfont)
            stacks = axsb[repetition, c].stackplot(t, Lhhat_pos, Lr_pos, baseline='zero', colors=colors,
                                              edgecolor='black', linewidth=0.8, labels=labels)
            hatches = ["\\\\", "//"]
            for stack, hatch in zip(stacks, hatches):
                stack.set_hatch(hatch)
            axsb[repetition, c].set_ylim(-1*180/np.pi, 12*180/np.pi)
            axsb[repetition, c].set_xlim(t_start, t_end)
            if repetition == 0:
                axsb[repetition, c].set_title(title, **self.hfont)
                axsb[repetition, c].set_xticks([])
                axsb[repetition, c].legend(prop=self.legend_font_small, loc='upper left')

            if repetition == 3:
                axsb[repetition, c].set_xlabel('Time ($s$)', **self.hfont)
            else:
                axsb[repetition, c].set_xticks([])
            if c == 0:
                axsb[repetition, c].set_ylabel("Gain \n ($Nm/^\circ$)", **self.hfont_small)
            else:
                axsb[repetition, c].set_yticks([])
            # axsb[repetition, c].ticklabel_format(axis="y", style="sci", scilimits=(0,0))
            plt.tight_layout(pad=1)



