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
        self.legend_size = 13
        self.csfont = {'fontname': 'Georgia', 'size': self.title_size}
        self.hfont = {'fontname': 'Georgia', 'size': self.label_size}

        # Colors
        self.tud_blue = "#0066A2"
        self.tud_black = "#000000"
        self.tud_red = "#c3312f"
        self.tud_green = "#00A390"
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
        # VIRTUAL HUMAN

        # Steering angle
        plt.figure()
        plt.title("Measured and estimated steering angle", **self.csfont)
        plt.plot(t_vir, r_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Reference $\phi_r(t)$")
        plt.plot(t_vir, x_vir, self.tud_blue, linestyle="--", linewidth=self.lw, label="Steering angle $\phi(t)$")
        plt.plot(t_sim, x_sim[:-1], self.tud_blue, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Simulated $\phi_{sim}(t)$")
        plt.plot(t_vir, x_hat_vir, self.tud_red, linewidth=self.lw, linestyle="--", label="Estimated $\hat{\phi}(t)$")
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Steering angle (rad)', **self.hfont)
        plt.legend(prop={"size": self.legend_size}, loc='upper right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.title("Accelation filtered", **self.csfont)
        plt.plot(t_vir, xddot_vir, self.tud_blue, linewidth=self.lw, linestyle="--", label="Filtered $\ddot{\phi}(t)$")
        plt.plot(t_sim, xddot_sim, self.tud_blue, linewidth=self.lw, linestyle="-", alpha=0.7, label="Simulated $\ddot{\phi}(t)$")
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Steering acceleration (rad/s^2)', **self.hfont)
        plt.legend(prop={"size": self.legend_size}, loc='upper right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.title("velocity filtered", **self.csfont)
        plt.plot(t_vir, xdot_vir, self.tud_blue, linewidth=self.lw, linestyle="--", label="Filtered $\dot{\phi}(t)$")
        plt.plot(t_sim, xdot_sim[:-1], self.tud_blue, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Simulated $\dot{\phi}(t)$")
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Steering rate (rad/s)', **self.hfont)
        plt.legend(prop={"size": self.legend_size}, loc='upper right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(t_vir, t_ex_vir)

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
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Steering angle (rad)', **self.hfont)
        plt.legend(prop={"size": self.legend_size}, loc='lower right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        options = {"arrowstyle": '->'}
        n = len(Lh_vel_vir)


        # Cost function weights
        plt.figure()
        plt.plot(t_sim, Qhhat_pos_sim[:-1], self.tud_red, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Simulated (human) $\hat{Q}_{h,1}(t)$")
        plt.plot(t_vir, Qhhat_pos_vir, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated (human) $\hat{Q}_{h,1}(t)$")
        plt.plot(t_vir, Qh_pos_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Virtual (human) $Q_{h,1,vir}(t)$")
        plt.annotate("Weak action", (7.5, Qh_pos_vir[int(n / 12)]), xytext=(7.5, Qh_pos_vir[int(n / 12)] + 5),
                     arrowprops=options)
        plt.annotate("No interaction", (22.5, Qh_pos_vir[int(3 * n / 12)]),
                     xytext=(18, Qh_pos_vir[int(3 * n / 12)] - 5), arrowprops=options)
        plt.annotate("Strong action", (37.5, Qh_pos_vir[int(5 * n / 12)]),
                     xytext=(34, Qh_pos_vir[int(5 * n / 12)] + 5), arrowprops=options)
        plt.annotate("No interaction", (52.5, Qh_pos_vir[int(7 * n / 12)]),
                     xytext=(48, Qh_pos_vir[int(7 * n / 12)] + 5), arrowprops=options)
        plt.annotate("Counteracting steering", (67.5, Qh_pos_vir[int(9 * n / 12)]),
                     xytext=(48, Qh_pos_vir[int(9 * n / 12)] - 5), arrowprops=options)
        plt.annotate("No interaction", (82.5, Qh_pos_vir[int(11 * n / 12)]),
                     xytext=(70, Qh_pos_vir[int(11 * n / 12)] + 5), arrowprops=options)
        plt.title('Steering angle error weight', **self.csfont)
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Weight value (-)', **self.hfont)
        plt.legend(prop={"size": self.legend_size}, loc='lower left')
        plt.xlim(0, t_vir[-1])
        plt.ylim(-30, 35)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(t_sim, Qhhat_vel_sim[:-1], self.tud_red, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Simulated (human) $\hat{Q}_{h,2}(t)$")
        plt.plot(t_vir, Qhhat_vel_vir, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated (human) $\hat{Q}_{h,2}(t)$")
        plt.plot(t_vir, Qh_vel_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Virtual (human) $Q_{h,2,vir}(t)$")
        plt.annotate("Weak action", (7.5, Qh_vel_vir[int(n / 12)]), xytext=(7.5, Qh_vel_vir[int(n / 12)] + 0.2),
                     arrowprops=options)
        plt.annotate("No interaction", (22.5, Qh_vel_vir[int(3 * n / 12)]),
                     xytext=(18, Qh_vel_vir[int(3 * n / 12)] - 0.2), arrowprops=options)
        plt.annotate("Strong action", (37.5, Qh_vel_vir[int(5 * n / 12)]),
                     xytext=(34, Qh_vel_vir[int(5 * n / 12)] + 0.2), arrowprops=options)
        plt.annotate("No interaction", (52.5, Qh_vel_vir[int(7 * n / 12)]),
                     xytext=(48, Qh_vel_vir[int(7 * n / 12)] + 0.2), arrowprops=options)
        plt.annotate("Counteracting steering", (67.5, Qh_vel_vir[int(9 * n / 12)]),
                     xytext=(48, Qh_vel_vir[int(9 * n / 12)] - 0.2), arrowprops=options)
        plt.annotate("No interaction", (82.5, Qh_vel_vir[int(11 * n / 12)]),
                     xytext=(70, Qh_vel_vir[int(11 * n / 12)] + 0.2), arrowprops=options)
        plt.title('Steering rate error weight', **self.csfont)
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Weight value (-)', **self.hfont)
        plt.legend(prop={"size": self.legend_size}, loc='lower left')
        plt.xlim(0, t_vir[-1])
        plt.ylim(-1, 1.5)
        plt.tight_layout(pad=1)

        plt.figure()
        # plt.plot(t_vir, Lr_pos_vir, self.tud_blue, linewidth=self.lw, linestyle="--", label="Gain (robot) $L_{r}(t)$")
        if sim_data != None:
            # plt.plot(t_sim, Lr_pos_sim, self.tud_blue, linewidth=self.lw, label="Simulated $L_{r,sim}(t)$", alpha=0.7, )
            plt.plot(t_sim, Lhhat_pos_sim[:-1], self.tud_red, linewidth=self.lw, linestyle="-", alpha=0.7,
                     label="Simulated (human) $\hat{L}_{h,sim}(t)$")
        plt.plot(t_vir, Lhhat_pos_vir, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated (human) $\hat{L}_{h}(t)$")
        plt.plot(t_vir, Lh_pos_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Virtual (human) $L_{h,vir}(t)$")
        plt.annotate("Weak action", (7.5, Lh_pos_vir[int(n/12)]), xytext=(7.5, Lh_pos_vir[int(n/12)] + 2), arrowprops=options)
        plt.annotate("No interaction", (22.5, Lh_pos_vir[int(3*n/12)]), xytext=(18, Lh_pos_vir[int(3*n/12)] - 2), arrowprops=options)
        plt.annotate("Strong action", (37.5, Lh_pos_vir[int(5*n/12)]), xytext=(34, Lh_pos_vir[int(5*n/12)] + 1.5), arrowprops=options)
        plt.annotate("No interaction", (52.5, Lh_pos_vir[int(7*n/12)]), xytext=(48, Lh_pos_vir[int(7*n/12)] + 2), arrowprops=options)
        plt.annotate("Counteracting steering", (67.5, Lh_pos_vir[int(9*n/12)]), xytext=(48, Lh_pos_vir[int(9*n/12)] - 1.5), arrowprops=options)
        plt.annotate("No interaction", (82.5, Lh_pos_vir[int(11*n/12)]), xytext=(70, Lh_pos_vir[int(11*n/12)] + 2), arrowprops=options)

        plt.title('Steering angle gain', **self.csfont)
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Gain value (Nm/rad)', **self.hfont)
        plt.legend(prop={"size": self.legend_size}, loc='lower left')
        plt.xlim(0, t_vir[-1])
        plt.ylim(-3, 6)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(t_vir, Lhhat_vel_vir, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated (human) $\hat{L}_{h}(t)$")
        plt.plot(t_sim, Lhhat_vel_sim[:-1], self.tud_red, linewidth=self.lw, linestyle="-", alpha=0.7,
             label="Simulated (human) $\hat{L}_{h,sim}(t)$")
        plt.plot(t_vir, Lh_vel_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7, label="Virtual (human) $L_{h,vir}(t)$")
        plt.annotate("Weak action", (7.5, Lh_vel_vir[int(n/12)]), xytext=(7.5, Lh_vel_vir[int(n/12)] + 0.5), arrowprops=options)
        plt.annotate("No interaction", (22.5, Lh_vel_vir[int(3*n/12)]), xytext=(15, Lh_vel_vir[int(3*n/12)] - 0.5), arrowprops=options)
        plt.annotate("Strong action", (37.5, Lh_vel_vir[int(5*n/12)]), xytext=(20, Lh_vel_vir[int(5*n/12)] + 0.2), arrowprops=options)
        plt.annotate("No interaction", (52.5, Lh_vel_vir[int(7*n/12)]), xytext=(48, Lh_vel_vir[int(7*n/12)] + 0.65), arrowprops=options)
        plt.annotate("Counteracting steering", (67.5, Lh_vel_vir[int(9*n/12)]), xytext=(55, Lh_vel_vir[int(9*n/12)] - 0.2),
                     arrowprops=options)
        plt.annotate("No interaction", (82.5, Lh_vel_vir[int(11*n/12)]), xytext=(70, Lh_vel_vir[int(11*n/12)] + 0.65), arrowprops=options)
        plt.title('Steering rate gain', **self.csfont)
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Gain value (Nms/rad)', **self.hfont)
        plt.legend(prop={"size": self.legend_size}, loc='lower left')
        plt.xlim(0, t_vir[-1])
        plt.ylim(-0.5, 1.0)
        plt.tight_layout(pad=1)

        # TODO --> Fix
        # try:
        #     self.stepinfo(t_vir, Lhhat_vel_vir, Lh_vel_vir)
        #
        # except:
        #     print("does not work")

        # ACTUAL HUMAN

        # Cost function weights
        plt.figure()
        plt.plot(t_h, Qhhat_pos_h, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated (human) $\hat{Q}_{h,1}(t)$")
        plt.plot(t_vir, Qh_pos_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Virtual (human) $Q_{h,1,vir}(t)$")
        plt.annotate("Weak action", (7.5, Qh_pos_vir[int(n / 12)]), xytext=(7.5, Qh_pos_vir[int(n / 12)] + 5),
                     arrowprops=options)
        plt.annotate("No interaction", (22.5, Qh_pos_vir[int(3 * n / 12)]),
                     xytext=(18, Qh_pos_vir[int(3 * n / 12)] - 5), arrowprops=options)
        plt.annotate("Strong action", (37.5, Qh_pos_vir[int(5 * n / 12)]),
                     xytext=(34, Qh_pos_vir[int(5 * n / 12)] + 5), arrowprops=options)
        plt.annotate("No interaction", (52.5, Qh_pos_vir[int(7 * n / 12)]),
                     xytext=(48, Qh_pos_vir[int(7 * n / 12)] + 5), arrowprops=options)
        plt.annotate("Counteracting steering", (67.5, Qh_pos_vir[int(9 * n / 12)]),
                     xytext=(48, Qh_pos_vir[int(9 * n / 12)] - 5), arrowprops=options)
        plt.annotate("No interaction", (82.5, Qh_pos_vir[int(11 * n / 12)]),
                     xytext=(70, Qh_pos_vir[int(11 * n / 12)] + 5), arrowprops=options)
        plt.title('Steering angle error weight', **self.csfont)
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Weight value (-)', **self.hfont)
        plt.legend(prop={"size": self.legend_size}, loc='lower left')
        plt.xlim(0, t_vir[-1])
        plt.ylim(-40, 45)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(t_h, Qhhat_vel_h, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated (human) $\hat{Q}_{h,2}(t)$")
        plt.plot(t_vir, Qh_vel_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Virtual (human) $Q_{h,2,vir}(t)$")
        plt.annotate("Weak action", (7.5, Qh_vel_vir[int(n / 12)]), xytext=(7.5, Qh_vel_vir[int(n / 12)] + 0.5),
                     arrowprops=options)
        plt.annotate("No interaction", (22.5, Qh_vel_vir[int(3 * n / 12)]),
                     xytext=(18, Qh_vel_vir[int(3 * n / 12)] - 0.5), arrowprops=options)
        plt.annotate("Strong action", (37.5, Qh_vel_vir[int(5 * n / 12)]),
                     xytext=(34, Qh_vel_vir[int(5 * n / 12)] + 0.5), arrowprops=options)
        plt.annotate("No interaction", (52.5, Qh_vel_vir[int(7 * n / 12)]),
                     xytext=(48, Qh_vel_vir[int(7 * n / 12)] + 0.5), arrowprops=options)
        plt.annotate("Counteracting steering", (67.5, Qh_vel_vir[int(9 * n / 12)]),
                     xytext=(48, Qh_vel_vir[int(9 * n / 12)] - 0.5), arrowprops=options)
        plt.annotate("No interaction", (82.5, Qh_vel_vir[int(11 * n / 12)]),
                     xytext=(70, Qh_vel_vir[int(11 * n / 12)] + 0.5), arrowprops=options)
        plt.title('Steering rate error weight', **self.csfont)
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Weight value (-)', **self.hfont)
        plt.legend(prop={"size": self.legend_size}, loc='lower left')
        plt.xlim(0, t_vir[-1])
        plt.ylim(-3, 2)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(t_h, Lhhat_pos_h, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated (human) $\hat{L}_{h}(t)$")
        plt.plot(t_vir, Lh_pos_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Virtual (human) $L_{h,vir}(t)$")
        plt.annotate("Weak action", (7.5, Lh_pos_vir[int(n / 12)]), xytext=(7.5, Lh_pos_vir[int(n / 12)] + 2),
                     arrowprops=options)
        plt.annotate("No interaction", (22.5, Lh_pos_vir[int(3 * n / 12)]),
                     xytext=(18, Lh_pos_vir[int(3 * n / 12)] - 2), arrowprops=options)
        plt.annotate("Strong action", (37.5, Lh_pos_vir[int(5 * n / 12)]),
                     xytext=(34, Lh_pos_vir[int(5 * n / 12)] + 1.5), arrowprops=options)
        plt.annotate("No interaction", (52.5, Lh_pos_vir[int(7 * n / 12)]),
                     xytext=(48, Lh_pos_vir[int(7 * n / 12)] + 2), arrowprops=options)
        plt.annotate("Counteracting steering", (67.5, Lh_pos_vir[int(9 * n / 12)]),
                     xytext=(48, Lh_pos_vir[int(9 * n / 12)] - 1.5), arrowprops=options)
        plt.annotate("No interaction", (82.5, Lh_pos_vir[int(11 * n / 12)]),
                     xytext=(70, Lh_pos_vir[int(11 * n / 12)] + 2), arrowprops=options)

        plt.title('Steering angle gain', **self.csfont)
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Gain value (Nm/rad)', **self.hfont)
        plt.legend(prop={"size": self.legend_size}, loc='lower right')
        plt.xlim(0, t_vir[-1])
        plt.ylim(-4.5, 6)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(t_h, Lhhat_vel_h, self.tud_red, linewidth=self.lw, linestyle="--", alpha=1,
                 label="Estimated (human) $\hat{L}_{h}(t)$")

        plt.plot(t_vir, Lh_vel_vir, self.tud_black, linewidth=self.lw, linestyle="-", alpha=0.7,
                 label="Virtual (human) $L_{h,vir}(t)$")
        plt.annotate("Weak action", (7.5, Lh_vel_vir[int(n / 12)]), xytext=(7.5, Lh_vel_vir[int(n / 12)] + 0.5),
                     arrowprops=options)
        plt.annotate("No interaction", (22.5, Lh_vel_vir[int(3 * n / 12)]),
                     xytext=(15, Lh_vel_vir[int(3 * n / 12)] - 0.5), arrowprops=options)
        plt.annotate("Strong action", (37.5, Lh_vel_vir[int(5 * n / 12)]),
                     xytext=(20, Lh_vel_vir[int(5 * n / 12)] + 0.2), arrowprops=options)
        plt.annotate("No interaction", (52.5, Lh_vel_vir[int(7 * n / 12)]),
                     xytext=(48, Lh_vel_vir[int(7 * n / 12)] + 0.65), arrowprops=options)
        plt.annotate("Counteracting steering", (67.5, Lh_vel_vir[int(9 * n / 12)]),
                     xytext=(55, Lh_vel_vir[int(9 * n / 12)] - 0.2),
                     arrowprops=options)
        plt.annotate("No interaction", (82.5, Lh_vel_vir[int(11 * n / 12)]),
                     xytext=(70, Lh_vel_vir[int(11 * n / 12)] + 0.65), arrowprops=options)
        plt.title('Steering rate gain', **self.csfont)
        plt.xlabel('Time (s)', **self.hfont)
        plt.ylabel('Gain value (Nms/rad)', **self.hfont)
        plt.legend(prop={"size": self.legend_size}, loc='upper right')
        plt.xlim(0, t_vir[-1])
        plt.ylim(-0.75, 1.25)
        plt.tight_layout(pad=1)

        self.save_all_figures()
        plt.show()

    def stepinfo(self, t, L, Lref):
        n = len(L)
        dL = np.array(L) / Lref[int(n/12)]
        print(t[-1])
        end_index = int(min(np.argwhere(np.array(t) > 15)))
        rise_start = int(min(np.argwhere(dL[0:end_index] > 0.1)))
        rise_end = int(min(np.argwhere(dL[0:end_index] > 0.9)))
        if len(np.argwhere(dL[0:end_index] > 1.05)) > 0:
            settling_time = int(max(max(np.argwhere(0.95 > dL[0:end_index])), max(np.argwhere(1.05 < dL[0:end_index]))))
        else:
            settling_time = int(max(np.argwhere(0.95 > dL[0:end_index])))
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
        if t[settling_time] < 14:
            plt.plot([t[settling_time]], [L[settling_time]], 'ko', alpha=0.7)
            options = {"arrowstyle": '->'}
            plt.annotate("Settling time", (t[settling_time], L[settling_time]), xytext=(5, Lref[int(n/12)]*1.4), arrowprops=options)


        print("start, end and risetime", rise_start, rise_end, rise_time)



    def save_all_figures(self):
        pp = PdfPages('validation.pdf')
        figs = None
        if figs is None:
            figs = [plt.figure(n) for n in plt.get_fignums()]
        for fig in figs:
            fig.savefig(pp, format='pdf')
        pp.close()
