import matplotlib.pyplot as plt
import os
from datetime import datetime
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages

class PlotStuff:
    def __init__(self):
        self.lw = 4
        self.title_size = 24
        self.label_size = 22
        self.legend_size = 18
        self.csfont = {'fontname': 'Georgia', 'size': self.title_size}
        self.hfont = {'fontname': 'Georgia', 'size': self.label_size}
        self.legend_font = {'family': 'Georgia', 'size': self.legend_size}

        # Colors
        self.tud_blue = "#0066A2"
        self.tud_black = "#000000"
        self.tud_red = "#c3312f"
        self.tud_green = "#00A390"
        self.tud_orange = "#EB7245"
        self.tud_yellow = "#F1BE3E"

        print("Plotting stuff")

    def plot(self, human_data, virt_data, sim_data):
        # UNPACK DATA
        # Human experiment data
        t_h = human_data["time"]
        ur_h = human_data["torque"]
        x_h = human_data["steering_angle"]
        r_h = human_data["reference_angle"]
        xdot_h = human_data["steering_rate"]
        rdot_h = human_data["reference_rate"]
        t_ex_h = human_data["execution_time"]
        x_hat_h = human_data["state_estimate_pos"]
        x_hatdot_h = human_data["state_estimate_vel"]
        Lhhat_pos_h = human_data["estimated_human_gain_pos"]
        Lhhat_vel_h = human_data["estimated_human_gain_vel"]
        Lr_pos_h = human_data["robot_gain_pos"]
        Lr_vel_h = human_data["robot_gain_vel"]
        Qhhat_pos_h = human_data["estimated_human_cost_1"]
        Qhhat_vel_h = human_data["estimated_human_cost_2"]
        Qr_pos_h = human_data["robot_cost_pos"]
        Qr_vel_h = human_data["robot_cost_vel"]
        uhhat_h = np.array(human_data["estimated_human_input"])

        # Virtual human data
        Lh_pos_vir = virt_data["virtual_human_gain_pos"]
        Lh_vel_vir = virt_data["virtual_human_gain_vel"]
        uh_vir = np.array(virt_data["virtual_human_torque"])
        t_vir = virt_data["time"]
        ur_vir = virt_data["torque"]
        x_vir = virt_data["steering_angle"]
        r_vir = virt_data["reference_angle"]
        xdot_vir = virt_data["steering_rate"]
        xddot_vir = virt_data["acceleration"]
        rdot_vir = virt_data["reference_rate"]
        t_ex_vir = virt_data["execution_time"]
        x_hat_vir = virt_data["state_estimate_pos"]
        x_hatdot_vir = virt_data["state_estimate_vel"]
        Lhhat_pos_vir = virt_data["estimated_human_gain_pos"]
        Lhhat_vel_vir = virt_data["estimated_human_gain_vel"]
        Lr_pos_vir = virt_data["robot_gain_pos"]
        Lr_vel_vir = virt_data["robot_gain_vel"]
        Qhhat_pos_vir = virt_data["estimated_human_cost_1"]
        Qhhat_vel_vir = virt_data["estimated_human_cost_2"]
        Qr_pos_vir = virt_data["robot_cost_pos"]
        Qr_vel_vir = virt_data["robot_cost_vel"]
        Qh_pos_vir = virt_data["virtual_human_cost_pos"]
        Qh_vel_vir = virt_data["virtual_human_cost_vel"]
        uhhat_vir = np.array(virt_data["estimated_human_input"])
        uhtilde_vir = np.array(virt_data["input_estimation_error"])
        uh_measured = uhhat_vir - uhtilde_vir
        uh_meas = virt_data["measured_human_input"]

        # Simulation data
        t_sim = sim_data["time"]
        ur_sim = sim_data["robot_input"]
        states_sim = sim_data["states"]
        x_sim = states_sim[:, 0]
        xdot_sim = states_sim[:, 1]
        dstates_sim = sim_data["ydot"]
        xddot_sim = dstates_sim[:, 1]
        xi_sim = sim_data["error_states"]

        Lhhat_sim = sim_data["human_estimated_gain"]
        Lhhat_pos_sim = Lhhat_sim[:, 0]
        Lhhat_vel_sim = Lhhat_sim[:, 1]
        Qhhat_sim = sim_data["human_estimated_Q"]
        Qhhat_pos_sim = Qhhat_sim[:, 0, 0]
        Qhhat_vel_sim = Qhhat_sim[:, 1, 1]
        Qr_sim = sim_data["robot_gain"]
        # Lr_pos_sim = Lr_sim[:, 0]
        # Lr_vel_sim = Lr_sim[:, 1]
        uhhat_sim = sim_data["human_estimated_input"]

        # Let's get plottin'
        # VIRTUAL HUMAN MODEL FIT


        # Steering torques
        plt.figure()
        plt.title("Steering torque estimation", **self.csfont)
        plt.plot(t_vir, uh_measured, self.tud_red, linewidth=self.lw, linestyle="-", alpha=0.5,
                 label="Computed Torque")
        plt.plot(t_vir, uh_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=1,
                 label="Actual Torque")
        plt.plot(t_vir, uhhat_vir, self.tud_red, linewidth=self.lw, linestyle="-", alpha=1,
                 label="Estimated Torque")
        plt.xlabel('Time ($s$)', **self.hfont)
        plt.ylabel('Human Steering Torque ($Nm$)', **self.hfont)
        plt.legend(prop=self.legend_font, loc='upper right')
        plt.xlim(0, 30)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.title("Steering torque estimation", **self.csfont)
        plt.plot(t_vir, uh_measured, self.tud_red, linewidth=self.lw, linestyle="-", alpha=0.5,
                 label="Computed Torque")
        plt.plot(t_vir, uh_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=1,
                 label="Actual Torque")
        plt.plot(t_vir, uhhat_vir, self.tud_red, linewidth=self.lw, linestyle="-", alpha=1,
                 label="Estimated Torque")
        plt.xlabel('Time ($s$)', **self.hfont)
        plt.ylabel('Human Steering Torque ($Nm$)', **self.hfont)
        plt.legend(prop=self.legend_font, loc='upper right')
        plt.xlim(0, t_vir[-1])
        plt.tight_layout(pad=1)

        # Steering torques
        plt.figure()
        plt.title("Steering torque estimation", **self.csfont)
        plt.stackplot(t_vir, uhhat_vir, ur_vir, -np.array(uhtilde_vir), colors=[self.tud_red, self.tud_blue, self.tud_orange],
                            labels=['Estimated human', 'Robot', 'Estimation error'], edgecolor='black', linewidth=0.1)
        plt.plot(t_vir, uh_vir, self.tud_black, linewidth=2, linestyle="-", alpha=1,
                 label="Virtual human")
        plt.xlabel('Time ($s$)', **self.hfont)
        plt.ylabel('Steering Torque ($Nm$)', **self.hfont)
        plt.legend(prop=self.legend_font, loc='upper right')
        plt.xlim(65, 105)
        plt.ylim(-1.5, 1.5)
        plt.tight_layout(pad=1)

        # Steering angle
        plt.figure()
        plt.title("Measured and estimated steering angle", **self.csfont)
        plt.plot(t_vir, r_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Reference $\phi_r(t)$")
        plt.plot(t_vir, x_vir, self.tud_blue, linestyle="--", linewidth=self.lw, label="Steering angle $\phi(t)$")
        plt.plot(t_sim, x_sim[:-1], self.tud_blue, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Simulated $\phi_{sim}(t)$")
        plt.plot(t_vir, x_hat_vir, self.tud_red, linewidth=self.lw, linestyle="--", label="Estimated $\hat{\phi}(t)$")
        plt.xlabel('Time ($s$)', **self.hfont)
        plt.ylabel('Steering angle ($rad$)', **self.hfont)
        plt.legend(prop=self.legend_font, loc='upper right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.title("Filtered acceleration", **self.csfont)
        plt.plot(t_vir, xddot_vir, self.tud_blue, linewidth=self.lw, linestyle="--", label="Filtered signal")
        plt.plot(t_sim, xddot_sim, self.tud_blue, linewidth=self.lw, linestyle="-", alpha=0.7, label="Simulated signal")
        plt.xlabel('Time ($s$)', **self.hfont)
        plt.ylabel('Steering acceleration ($rad/s^2$)', **self.hfont)
        plt.legend(prop=self.legend_font, loc='upper right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.title("Filtered velocity", **self.csfont)
        plt.plot(t_vir, xdot_vir, self.tud_blue, linewidth=self.lw, linestyle="--", label="Filtered signal")
        plt.plot(t_sim, xdot_sim[:-1], self.tud_blue, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Simulated signal")
        plt.xlabel('Time ($s$)', **self.hfont)
        plt.ylabel('Steering rate ($rad/s$)', **self.hfont)
        plt.legend(prop=self.legend_font, loc='upper right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        # Steering rate
        plt.figure()
        plt.title("Measured and estimated steering rate", **self.csfont)
        plt.plot(t_vir, rdot_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Reference $\phi_r(t)$")
        plt.plot(t_vir, xdot_vir, self.tud_blue, linestyle="--", linewidth=self.lw,
                 label="Steering rate $\dot{\phi}(t)$")
        plt.plot(t_sim, xdot_sim[:-1], self.tud_blue, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Simulated $\dot{\phi_{sim}}(t)$")
        plt.plot(t_vir, x_hatdot_vir, self.tud_red, linewidth=self.lw, linestyle="--",
                 label="Estimated $\dot{\hat{\phi}}(t)$")
        plt.xlabel('Time ($s$)', **self.hfont)
        plt.ylabel('Steering angle ($rad$)', **self.hfont)
        plt.legend(prop=self.legend_font, loc='lower right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        options = {"arrowstyle": '->'}
        n = len(Lh_vel_vir)

        # Cost function weights
        plt.figure()
        plt.plot(t_sim, Qhhat_pos_sim[:-1], self.tud_red, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Simulated human cost weight")
        plt.plot(t_vir, Qhhat_pos_vir, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated human cost weight")
        plt.plot(t_vir, Qh_pos_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Virtual human cost weight")

        # plt.annotate("Weak steering", (7.5, Qh_pos_vir[int(n / 12)]), xytext=(7.5, Qh_pos_vir[int(n / 12)] + 5),
        #              arrowprops=options)
        # plt.annotate("No steering", (22.5, Qh_pos_vir[int(3 * n / 12)]),
        #              xytext=(18, Qh_pos_vir[int(3 * n / 12)] - 5), arrowprops=options)
        # plt.annotate("Strong steering", (37.5, Qh_pos_vir[int(5 * n / 12)]),
        #              xytext=(34, Qh_pos_vir[int(5 * n / 12)] + 5), arrowprops=options)
        # plt.annotate("No steering", (52.5, Qh_pos_vir[int(7 * n / 12)]),
        #              xytext=(38, Qh_pos_vir[int(7 * n / 12)] - 5), arrowprops=options)
        # plt.annotate("Counteracting steering", (67.5, Qh_pos_vir[int(9 * n / 12)]),
        #              xytext=(55, Qh_pos_vir[int(9 * n / 12)] - 10), arrowprops=options)
        # plt.annotate("No steering", (82.5, Qh_pos_vir[int(11 * n / 12)]),
        #              xytext=(70, Qh_pos_vir[int(11 * n / 12)] + 10), arrowprops=options)

        plt.title('Virtual Human: Angle error weight', **self.csfont)
        plt.xlabel('Time ($s$)', **self.hfont)
        plt.ylabel('Weight value (-)', **self.hfont)
        plt.legend(prop=self.legend_font, loc='lower left')
        plt.xlim(0, t_vir[-1])
        plt.ylim(-30, 35)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(t_sim, Qhhat_vel_sim[:-1], self.tud_red, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Simulated human cost weight")
        plt.plot(t_vir, Qhhat_vel_vir, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated human cost weight")
        plt.plot(t_vir, Qh_vel_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Virtual human cost weight")

        # plt.annotate("Weak steering", (7.5, Qh_vel_vir[int(n / 12)]), xytext=(4.5, Qh_vel_vir[int(n / 12)] + 0.2),
        #              arrowprops=options)
        # plt.annotate("No steering", (22.5, Qh_vel_vir[int(3 * n / 12)]),
        #              xytext=(10, Qh_vel_vir[int(3 * n / 12)] - 0.2), arrowprops=options)
        # plt.annotate("Strong steering", (37.5, Qh_vel_vir[int(5 * n / 12)]),
        #              xytext=(34, Qh_vel_vir[int(5 * n / 12)] + 0.2), arrowprops=options)
        # plt.annotate("No steering", (52.5, Qh_vel_vir[int(7 * n / 12)]),
        #              xytext=(38, Qh_vel_vir[int(7 * n / 12)] - 0.2), arrowprops=options)
        # plt.annotate("Counteracting steering", (67.5, Qh_vel_vir[int(9 * n / 12)]),
        #              xytext=(54, Qh_vel_vir[int(9 * n / 12)] - 0.4), arrowprops=options)
        # plt.annotate("No steering", (82.5, Qh_vel_vir[int(11 * n / 12)]),
        #              xytext=(70, Qh_vel_vir[int(11 * n / 12)] + 0.2), arrowprops=options)

        plt.title('Virtual Human: Rate error weight', **self.csfont)
        plt.xlabel('Time ($s$)', **self.hfont)
        plt.ylabel('Weight value (-)', **self.hfont)
        plt.legend(prop=self.legend_font, loc='lower left')
        plt.xlim(0, t_vir[-1])
        plt.ylim(-1.5, 1.5)
        plt.tight_layout(pad=1)

        plt.figure()
        if sim_data != None:
            # plt.plot(t_sim, Lr_pos_sim, self.tud_blue, linewidth=self.lw, label="Simulated $L_{r,sim}(t)$", alpha=0.7, )
            plt.plot(t_sim, Lhhat_pos_sim[:-1], self.tud_red, linewidth=self.lw, linestyle="-", alpha=0.7,
                     label="Simulated human gain")
        plt.plot(t_vir, Lhhat_pos_vir, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated human gain")
        plt.plot(t_vir, Lh_pos_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Virtual human gain")


        plt.title('Virtual Human: Steering angle gain', **self.csfont)
        plt.xlabel('Time ($s$)', **self.hfont)
        plt.ylabel('Gain value ($Nm/rad$)', **self.hfont)
        plt.legend(prop=self.legend_font, loc='lower left')
        plt.xlim(0, t_vir[-1])
        plt.ylim(-4, 5)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(t_sim, Lhhat_vel_sim[:-1], self.tud_red, linewidth=self.lw, linestyle="-", alpha=0.7,
             label="Simulated human gain")
        plt.plot(t_vir, Lhhat_vel_vir, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated human gain")
        plt.plot(t_vir, Lh_vel_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7, label="Virtual human gain")

        plt.title('Virtual Human: Steering rate gain', **self.csfont)
        plt.xlabel('Time ($s$)', **self.hfont)
        plt.ylabel('Gain value ($Nms/rad$)', **self.hfont)
        plt.legend(prop=self.legend_font, loc='lower left')
        plt.xlim(0, t_vir[-1])
        plt.ylim(-0.6, 0.6)
        plt.tight_layout(pad=1)

        # ACTUAL HUMAN

        # Cost function weights
        plt.figure()
        plt.plot(t_h, Qhhat_pos_h, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated human cost weight")
        plt.plot(t_vir, Qh_pos_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Virtual human cost weight")
        # plt.annotate("Weak steering", (7.5, Qh_pos_vir[int(n / 12)]), xytext=(7.5, Qh_pos_vir[int(n / 12)] + 20),
        #              arrowprops=options)
        # plt.annotate("No steering", (22.5, Qh_pos_vir[int(3 * n / 12)]),
        #              xytext=(18, Qh_pos_vir[int(3 * n / 12)] - 5), arrowprops=options)
        # plt.annotate("Strong steering", (37.5, Qh_pos_vir[int(5 * n / 12)]),
        #              xytext=(20, Qh_pos_vir[int(5 * n / 12)] + 15), arrowprops=options)
        # plt.annotate("No steering", (52.5, Qh_pos_vir[int(7 * n / 12)]),
        #              xytext=(48, Qh_pos_vir[int(7 * n / 12)] + 15), arrowprops=options)
        # plt.annotate("Counteracting steering", (67.5, Qh_pos_vir[int(9 * n / 12)]),
        #              xytext=(60, Qh_pos_vir[int(9 * n / 12)] - 7.5), arrowprops=options)
        # plt.annotate("No steering", (82.5, Qh_pos_vir[int(11 * n / 12)]),
        #              xytext=(70, Qh_pos_vir[int(11 * n / 12)] + 15), arrowprops=options)
        plt.title('Real Human: Angle error weight', **self.csfont)
        plt.xlabel('Time ($s$)', **self.hfont)
        plt.ylabel('Weight value (-)', **self.hfont)
        plt.legend(prop=self.legend_font, loc='lower left')
        plt.xlim(0, t_vir[-1])
        plt.ylim(-25, 45)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(t_h, Qhhat_vel_h, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated human cost weight")
        plt.plot(t_vir, Qh_vel_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Virtual human cost weight")
        # plt.annotate("Weak steering", (7.5, Qh_vel_vir[int(n / 12)]), xytext=(7.5, Qh_vel_vir[int(n / 12)] + 0.5),
        #              arrowprops=options)
        # plt.annotate("No steering", (22.5, Qh_vel_vir[int(3 * n / 12)]),
        #              xytext=(10, Qh_vel_vir[int(3 * n / 12)] - 0.5), arrowprops=options)
        # plt.annotate("Strong steering", (37.5, Qh_vel_vir[int(5 * n / 12)]),
        #              xytext=(34, Qh_vel_vir[int(5 * n / 12)] + 0.5), arrowprops=options)
        # plt.annotate("No steering", (52.5, Qh_vel_vir[int(7 * n / 12)]),
        #              xytext=(50, Qh_vel_vir[int(7 * n / 12)] + 0.5), arrowprops=options)
        # plt.annotate("Counteracting steering", (67.5, Qh_vel_vir[int(9 * n / 12)]),
        #              xytext=(48, Qh_vel_vir[int(9 * n / 12)] - 0.5), arrowprops=options)
        # plt.annotate("No steering", (82.5, Qh_vel_vir[int(11 * n / 12)]),
        #              xytext=(70, Qh_vel_vir[int(11 * n / 12)] + 0.5), arrowprops=options)
        plt.title('Real Human: Rate error weight', **self.csfont)
        plt.xlabel('Time ($s$)', **self.hfont)
        plt.ylabel('Weight value (-)', **self.hfont)
        plt.legend(prop=self.legend_font, loc='lower left')
        plt.xlim(0, t_vir[-1])
        plt.ylim(-3, 2)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(t_h, Lhhat_pos_h, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated human gain")
        # plt.plot(t_h, Lr_pos_h, self.tud_blue, linewidth=self.lw, linestyle="--", alpha=1,
        #          label="Robot gain $L_{r}(t)$")
        plt.plot(t_vir, Lh_pos_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Virtual human gain")
        plt.annotate("Weak steering", (15, Lh_pos_vir[int(n/12)]), xytext=(2.5, Lh_pos_vir[int(n/12)] + 2), arrowprops=options)
        plt.annotate("No steering", (50, Lh_pos_vir[int(3*n/12)]), xytext=(30, Lh_pos_vir[int(3*n/12)] + 2), arrowprops=options)
        plt.annotate("Strong steering", (80, Lh_pos_vir[int(5*n/12)]), xytext=(50, Lh_pos_vir[int(5*n/12)] + 1), arrowprops=options)
        plt.annotate("No steering", (115, Lh_pos_vir[int(7*n/12)]), xytext=(90, Lh_pos_vir[int(7*n/12)] - 1), arrowprops=options)
        plt.annotate("Counteracting steering", (150, Lh_pos_vir[int(9*n/12)]), xytext=(135, Lh_pos_vir[int(9*n/12)] - 1.5), arrowprops=options)
        plt.annotate("No steering", (180, Lh_pos_vir[int(11*n/12)]), xytext=(150, Lh_pos_vir[int(11*n/12)] + 1.5), arrowprops=options)
        plt.title('Real human: Steering angle gain', **self.csfont)
        plt.xlabel('Time ($s$)', **self.hfont)
        plt.ylabel('Gain value ($Nm/rad$)', **self.hfont)
        plt.legend(prop=self.legend_font, loc='lower left')
        plt.xlim(0, t_vir[-1])
        plt.ylim(-4, 5)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(t_h, Lhhat_vel_h, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated human gain")

        plt.plot(t_vir, Lh_vel_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Virtual human gain")
        # plt.plot(t_h, Lr_vel_h, self.tud_blue, linewidth=self.lw, linestyle="--", alpha=1,
        #          label="Robot gain $L_{r}(t)$")
        plt.annotate("Weak steering", (15, Lh_vel_vir[int(n / 12)]), xytext=(2.5, Lh_vel_vir[int(n / 12)] + 0.25),
                     arrowprops=options)
        plt.annotate("No steering", (50, Lh_vel_vir[int(3 * n / 12)]),
                     xytext=(30, Lh_vel_vir[int(3 * n / 12)] + 0.3), arrowprops=options)
        plt.annotate("Strong steering", (80, Lh_vel_vir[int(5 * n / 12)]),
                     xytext=(50, Lh_vel_vir[int(5 * n / 12)] + 0.2), arrowprops=options)
        plt.annotate("No steering", (115, Lh_vel_vir[int(7 * n / 12)]),
                     xytext=(115, Lh_vel_vir[int(7 * n / 12)] + 0.25), arrowprops=options)
        plt.annotate("Counteracting steering", (150, Lh_vel_vir[int(9 * n / 12)]),
                     xytext=(135, Lh_vel_vir[int(9 * n / 12)] - 0.2),
                     arrowprops=options)
        plt.annotate("No steering", (180, Lh_vel_vir[int(11 * n / 12)]),
                     xytext=(150, Lh_vel_vir[int(11 * n / 12)] + 0.35), arrowprops=options)
        plt.title('Real human: Steering rate gain', **self.csfont)
        plt.xlabel('Time ($s$)', **self.hfont)
        plt.ylabel('Gain value ($Nms/rad$)', **self.hfont)
        plt.legend(prop=self.legend_font, loc='lower left')
        plt.xlim(0, t_vir[-1])
        plt.ylim(-0.8, 0.8)
        plt.tight_layout(pad=1)

        # Metrics
        plt.figure()
        plt.plot(t_sim, Lhhat_pos_sim[:-1], self.tud_red, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Simulated human gain")
        plt.plot(t_vir, Lhhat_pos_vir, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated human gain")
        plt.plot(t_vir, Lh_pos_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Virtual human gain")
        self.stepinfo(t_vir, Lhhat_pos_vir, Lh_pos_vir)
        plt.title('Virtual human: Steering angle gain', **self.csfont)
        plt.xlabel('Time ($s$)', **self.hfont)
        plt.ylabel('Gain value ($Nm/rad$)', **self.hfont)
        plt.legend(prop=self.legend_font, loc='upper left')
        plt.xlim(0, 33)
        plt.ylim(-0.1, 2.5)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(t_sim, Lhhat_vel_sim[:-1], self.tud_red, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Simulated human gain")
        plt.plot(t_vir, Lhhat_vel_vir, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated human gain")
        plt.plot(t_vir, Lh_vel_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Virtual human gain")
        self.stepinfo(t_vir, Lhhat_vel_vir, Lh_vel_vir)
        plt.title('Virtual human: Steering rate gain', **self.csfont)
        plt.xlabel('Time ($s$)', **self.hfont)
        plt.ylabel('Gain value ($Nms/rad$)', **self.hfont)
        plt.legend(prop=self.legend_font, loc='upper left')
        plt.xlim(0, 33)
        plt.ylim(-0.1, 0.6)
        plt.tight_layout(pad=1)

        self.save_all_figures()
        plt.show()

    def stepinfo(self, t, L, Lref):
        n = len(L)
        t_stop = 200/6
        dL = np.array(L) / Lref[int(n/12)]
        print(t[-1])
        end_index = int(min(np.argwhere(np.array(t) > t_stop)))
        rise_start = int(min(np.argwhere(dL[0:end_index] > 0.1)))
        # try:
        rise_end = int(min(np.argwhere(dL[0:end_index] > 0.9)))
        # except:
        #     rise_end = 0
        if len(np.argwhere(dL[0:end_index] > 1.05)) > 0:
            try:
                settling_time = int(max(max(np.argwhere(0.95 > dL[0:end_index])), max(np.argwhere(1.05 < dL[0:end_index]))))
            except:
                settling_time = t_stop
        else:
            try:
                settling_time = int(max(np.argwhere(0.95 > dL[0:end_index])))
            except:
                settling_time = t_stop
        print("settling time: ", t[settling_time])

        rise_time = t[rise_end] - t[rise_start]

        print("gain reference: ", Lref[int(n/12)])

        # Take timeseries from the risetime to compute bias and variance
        L_ss = L[rise_end:end_index]
        mean = np.mean(L_ss)
        median = np.median(L_ss)
        variance = np.var(L_ss)
        print("mean, median, bias, variance = ", mean, median, mean-Lref[int(n/12)], variance)

        print("rise time: ", rise_time)
        plt.plot([t[rise_start], t[rise_start]], [-20, L[rise_start]], 'k--', alpha=0.4)
        plt.plot([t[rise_end], t[rise_end]], [-20, L[rise_end]], 'k--', alpha=0.4)
        plt.plot([t[rise_start]], [L[rise_start]], 'k.', alpha=0.7)
        plt.plot([t[rise_end]], [L[rise_end]], 'k.', alpha=0.7)
        options = {"arrowstyle": '<->'}
        plt.annotate("Rise time", (t[rise_start], L[rise_start]), xytext=(t[rise_end]*1.05, L[rise_start]*0.85),
                     arrowprops=options)
        plt.plot([t[settling_time], t[settling_time]], [-20, L[settling_time]], 'k--', alpha=0.7)
        plt.plot([t[0], t[-1]], [1.05 * Lref[int(n/12)], 1.05 * Lref[int(n/12)]], 'k--', alpha=0.4)
        plt.plot([t[0], t[-1]], [0.95 * Lref[int(n/12)], 0.95 * Lref[int(n/12)]], 'k--', alpha=0.4)
        if t[settling_time] < t_stop-0.5:
            plt.plot([t[settling_time]], [L[settling_time]], 'ko', alpha=0.7)
            options = {"arrowstyle": '->'}
            plt.annotate("Settling time", (t[settling_time], L[settling_time]), xytext=(t[settling_time]*0.8, Lref[int(n/12)]*0.8), arrowprops=options)


        print("start, end and risetime", rise_start, rise_end, rise_time)



    def save_all_figures(self):
        pp = PdfPages('validation.pdf')
        figs = None
        if figs is None:
            figs = [plt.figure(n) for n in plt.get_fignums()]
        for fig in figs:
            fig.savefig(pp, format='pdf')
        pp.close()
