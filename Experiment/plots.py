import matplotlib.pyplot as plt
import os
from datetime import datetime

class PlotStuff:
    def __init__(self):
        print("Plotting stuff")

    def plot_stuff(self, data, type):
        # Figure properties
        csfont = {'fontname': 'Georgia'}
        hfont = {'fontname': 'Georgia'}

        # Colors
        tud_blue = "#0066A2"
        tud_black = "#000000"
        tud_grey = "#808080"
        tud_red = "#c3312f"
        tud_green = "#00A390"

        # UNPACK DATA
        # General
        t = data["time"]
        ur = data["input"]
        x = data["position"]
        r = data["reference"]
        xdot = data["velocity"]
        xddot = data["acceleration"]
        rdot = data["reference_vel"]

        if type == "Gain_observer":
            Lhhat_pos = data["estimated_human_gain_pos"]
            Lhhat_vel = data["estimated_human_gain_vel"]
            Lr_pos = data["robot_gain_pos"]
            Lr_vel = data["robot_gain_vel"]
            uhhat = data["estimated_human_input"]
            uhtilde = data["input_estimation_error"]

        # Plot stuff
        fig1 = plt.figure()
        title = type
        fig1.suptitle(title, **csfont)

        gs1 = fig1.add_gridspec(2, 2)
        ax1a = fig1.add_subplot(gs1[0, :])
        ax1b = fig1.add_subplot(gs1[1, 0])
        ax1c = fig1.add_subplot(gs1[1, 1])

        ax1a.plot(t, r, tud_black, linewidth=2.5, linestyle="-", alpha=0.7, label="Reference $r(t)$")
        ax1a.plot(t, x, tud_blue, linestyle="--", linewidth=2.5, label="State $x(t)$")
        ax1a.set_title('Position', **csfont)
        ax1a.set_xlabel('Time (s)', **hfont)
        ax1a.set_ylabel('Position (m)', **hfont)

        ax1b.plot(t, rdot, tud_black, linewidth=2, linestyle="-", alpha=0.7, label="Reference $r(t)$")
        ax1b.plot(t, xdot, tud_blue, linestyle="--", linewidth=2, label="Velocity $v(t)$")
        ax1b.set_title('Velocity', **csfont)
        ax1b.set_xlabel('Time (s)', **hfont)
        ax1b.set_ylabel('Velocity (m/s)', **hfont)

        ax1c.plot(t, ur, tud_blue, linestyle="--", linewidth=2, label="$u_r(t)$")

        # ax1c.plot(t, uh, tud_red, label="$u_h(t)$")
        if type == "Gain_observer":
            ax1c.plot(t, uhhat, tud_red, linestyle="--", linewidth=2, alpha=1, label="$\hat{u}_h(t)$")

        # ax1c.plot(t, ut, tud_green, alpha=1, label="$u(t)$")
        ax1c.set_title('Control action', **csfont)
        ax1c.set_xlabel('Time (s)', **hfont)
        ax1c.set_ylabel('Input force (N)', **hfont)

        ax1a.legend(prop={"size": 8}, loc='upper right')
        ax1a.set(xlim=(0, t[-1]))
        ax1b.legend(prop={"size": 8}, loc='upper right')
        ax1b.set(xlim=(0, t[-1]))
        ax1c.legend(prop={"size": 8}, loc='upper right')
        ax1c.set(xlim=(0, t[-1]))

        fig1.tight_layout(pad=1)

        plt.figure()
        plt.plot(t, xddot)

        if type == "Gain_observer":
            fig2 = plt.figure()
            title = type
            fig2.suptitle(title, **csfont)

            gs2 = fig2.add_gridspec(2, 6)
            ax2a = fig2.add_subplot(gs2[0, :])
            ax2b = fig2.add_subplot(gs2[1, :])

            # ax2f = fig2.add_subplot(gs2[1, 0:3])
            # ax2g = fig2.add_subplot(gs2[1, 3:6])

            ax2a.plot(t, Lhhat_pos, tud_red, linewidth=2.5, linestyle="--", alpha=1, label="$\hat{L}_{h,1}(t)$")
            ax2b.plot(t, Lhhat_vel, tud_red, linewidth=2.5, linestyle="--", alpha=1, label="$\hat{L}_{h,2}(t)$")
            ax2a.plot(t, Lr_pos, tud_blue, linewidth=2.5, linestyle="--", label="$L_{r,1}(t)$")
            ax2b.plot(t, Lr_vel, tud_blue, linewidth=2.5, linestyle="--", label="$L_{r,2}(t)$")

            ax2a.set_title('Position gain', **csfont)
            ax2a.set_xlabel('Time (s)', **hfont)
            ax2a.set_ylabel('Gain value (N/m)', **hfont)
            ax2a.legend(prop={"size": 8}, loc='upper right')
            ax2a.set(xlim=(0, t[-1]))

            ax2b.set_title('Velocity gain', **csfont)
            ax2b.set_xlabel('Time (s)', **hfont)
            ax2b.set_ylabel('Gain value (Ns/m)', **hfont)
            ax2b.legend(prop={"size": 8}, loc='upper right')
            ax2b.set(xlim=(0, t[-1]))

            # # ax2f.plot(T, Qh[:, 0, 0], tud_red, label="$Q_{h,(1,1)}(t)$")
            # # ax2f.plot(T, Qhhat[:-1, 0, 0], tud_red, linestyle="--", alpha=0.5, label="$\hat{Q}_{h,(1,1)}(t)$")
            # # ax2f.plot(T, Qr[:, 0, 0], tud_blue, label="$Q_{r,(1,1)}(t)$")
            # # ax2g.plot(T, Qh[:, 1, 1], tud_red, label="$Q_{h,(2,2)}(t)$")
            # # ax2g.plot(T, Qhhat[:-1, 1, 1], tud_red, linestyle="--", alpha=0.5, label="$\hat{Q}_{h,(2,2)}(t)$")
            # # ax2g.plot(T, Qr[:, 1, 1], tud_blue, label="$Q_{r,(2,2)}(t)$")
            #
            # ax2f.set_title('Position cost weight', **csfont)
            # ax2f.set_xlabel('Time (s)', **hfont)
            # ax2f.set_ylabel('Weight value (-)', **hfont)
            # ax2f.legend(prop={"size": 8}, loc='upper right')
            # ax2f.set(xlim=(0, t[-1]))
            #
            # ax2g.set_title('Velocity cost weight', **csfont)
            # ax2g.set_xlabel('Time (s)', **hfont)
            # ax2g.set_ylabel('Weight value (-)', **hfont)
            # ax2g.legend(prop={"size": 8}, loc='upper right')
            # ax2g.set(xlim=(0, t[-1]))

            fig2.tight_layout(pad=1)

        plt.show()

    def plot_stuff_with_sim_data(self, exp_data, sim_data, type):
        csfont = {'fontname': 'Georgia'}
        hfont = {'fontname': 'Georgia'}

        # Colors
        tud_blue = "#0066A2"
        tud_black = "#000000"
        tud_grey = "#808080"
        tud_red = "#c3312f"
        tud_green = "#00A390"
        tud_yellow = "#F1BE3E"

        # UNPACK DATA
        # General
        t = exp_data["time"]
        ur = exp_data["input"]
        ureal = exp_data["measured_input"]
        x = exp_data["position"]
        r = exp_data["reference"]
        xdot = exp_data["velocity"]
        rdot = exp_data["reference_vel"]
        e = exp_data["error"]
        edot = exp_data["error_vel"]
        t_ex = exp_data["execution_time"]

        # Simulation
        t_sim = sim_data["time"]
        ur_sim = sim_data["robot_input"]
        states_sim = sim_data["states"]
        ref_sim = sim_data["reference_signal"]
        r_sim = ref_sim[:, 0]
        rdot_sim = ref_sim[:, 1]
        x_sim = states_sim[:, 0]
        xdot_sim = states_sim[:, 1]
        xi_sim = sim_data["error_states"]
        e_sim = xi_sim[:, 0]
        edot_sim = xi_sim[:, 1]

        if type == "Gain_observer":
            Lhhat_pos = exp_data["estimated_human_gain_pos"]
            Lhhat_vel = exp_data["estimated_human_gain_vel"]
            Lr_pos = exp_data["robot_gain_pos"]
            Lr_vel = exp_data["robot_gain_vel"]
            uhhat = exp_data["estimated_human_input"]
            uhtilde = exp_data["input_estimation_error"]
            Lhhat_sim = sim_data["human_estimated_gain"]
            Lhhat_pos_sim = Lhhat_sim[:, 0]
            Lhhat_vel_sim = Lhhat_sim[:, 1]
            Lr_sim = sim_data["robot_gain"]
            Lr_pos_sim = Lr_sim[:, 0]
            Lr_vel_sim = Lr_sim[:, 1]
            uhhat_sim = sim_data["human_estimated_input"]


        # Plot stuff
        fig1 = plt.figure()
        title = type
        fig1.suptitle(title, **csfont)

        gs1 = fig1.add_gridspec(2, 2)
        ax1a = fig1.add_subplot(gs1[0, :])
        ax1b = fig1.add_subplot(gs1[1, 0])
        ax1c = fig1.add_subplot(gs1[1, 1])

        ax1a.plot(t, r, tud_black, linewidth=2.5, linestyle="-", alpha=0.7, label="Reference $r(t)$")
        ax1a.plot(t, x, tud_blue, linestyle="--", linewidth=2.5, label="State $x(t)$")
        ax1a.plot(t_sim, x_sim[:-1], tud_blue, linewidth=2.5, linestyle="-", alpha=0.5, label="Simulated state $x_{sim}(t)$")
        ax1a.set_title('Position', **csfont)
        ax1a.set_xlabel('Time (s)', **hfont)
        ax1a.set_ylabel('Position (m)', **hfont)


        ax1b.plot(t, rdot, tud_black, linewidth=2, linestyle="-", alpha=0.7, label="Reference $r(t)$")
        ax1b.plot(t, xdot, tud_blue, linestyle="--", linewidth=2, label="Velocity $v(t)$")
        ax1b.plot(t_sim, xdot_sim[:-1], tud_blue, linestyle="-", linewidth=2, alpha=0.5, label="Simulated velocity $v_{sim}(t)$")
        ax1b.set_title('Velocity', **csfont)
        ax1b.set_xlabel('Time (s)', **hfont)
        ax1b.set_ylabel('Velocity (m/s)', **hfont)

        ax1c.plot(t, ur, tud_blue, linestyle="--", linewidth=2, label="$u_r(t)$")
        ax1c.plot(t_sim, ur_sim, tud_blue, linestyle="-", linewidth=2, alpha=0.5, label="$u_{r,sim}(t)$")
        # ax1c.plot(t, uh, tud_red, label="$u_h(t)$")
        if type == "Gain_observer":
            ax1c.plot(t, uhhat, tud_red, linestyle="--", linewidth=2, alpha=1, label="$\hat{u}_h(t)$")
            ax1c.plot(t_sim, uhhat_sim, tud_red, linewidth=2, linestyle="-", alpha=0.5, label="$\hat{u}_h(t)$")
        # ax1c.plot(t, ut, tud_green, alpha=1, label="$u(t)$")
        ax1c.set_title('Control action', **csfont)
        ax1c.set_xlabel('Time (s)', **hfont)
        ax1c.set_ylabel('Input force (N)', **hfont)

        ax1a.legend(prop={"size": 8}, loc='upper right')
        ax1a.set(xlim=(0, t[-1]))
        ax1b.legend(prop={"size": 8}, loc='upper right')
        ax1b.set(xlim=(0, t[-1]))
        ax1c.legend(prop={"size": 8}, loc='upper right')
        ax1c.set(xlim=(0, t[-1]))

        fig1.tight_layout(pad=1)
        self.save_figure_dump(type=1)

        if type == "Gain_observer":
            fig2 = plt.figure()
            title = type
            fig2.suptitle(title, **csfont)

            gs2 = fig2.add_gridspec(2, 6)
            ax2a = fig2.add_subplot(gs2[0, :])
            ax2b = fig2.add_subplot(gs2[1, :])

            # ax2f = fig2.add_subplot(gs2[1, 0:3])
            # ax2g = fig2.add_subplot(gs2[1, 3:6])

            ax2a.plot(t, Lhhat_pos, tud_red, linewidth=2.5, linestyle="--", alpha=1, label="$\hat{L}_{h,1}(t)$")
            ax2b.plot(t, Lhhat_vel, tud_red, linewidth=2.5, linestyle="--", alpha=1, label="$\hat{L}_{h,2}(t)$")
            ax2a.plot(t, Lr_pos, tud_blue, linewidth=2.5, linestyle="--", label="$L_{r,1}(t)$")
            ax2b.plot(t, Lr_vel, tud_blue, linewidth=2.5, linestyle="--", label="$L_{r,2}(t)$")
            ax2a.plot(t_sim, Lhhat_pos_sim, tud_red, linewidth=2.5, linestyle="-", alpha=0.5, label="$\hat{L}_{h,1}(t)$")
            ax2b.plot(t_sim, Lhhat_vel_sim, tud_red, linewidth=2.5, linestyle="-", alpha=0.5, label="$\hat{L}_{h,2}(t)$")
            ax2a.plot(t_sim, Lr_pos_sim, tud_blue, linewidth=2.5, label="$L_{r,1}(t)$", alpha=0.5,)
            ax2b.plot(t_sim, Lr_vel_sim, tud_blue, linewidth=2.5, label="$L_{r,2}(t)$", alpha=0.5,)

            ax2a.set_title('Position gain', **csfont)
            ax2a.set_xlabel('Time (s)', **hfont)
            ax2a.set_ylabel('Gain value (N/m)', **hfont)
            ax2a.legend(prop={"size": 8}, loc='upper right')
            ax2a.set(xlim=(0, t[-1]))

            ax2b.set_title('Velocity gain', **csfont)
            ax2b.set_xlabel('Time (s)', **hfont)
            ax2b.set_ylabel('Gain value (Ns/m)', **hfont)
            ax2b.legend(prop={"size": 8}, loc='upper right')
            ax2b.set(xlim=(0, t[-1]))

            # # ax2f.plot(T, Qh[:, 0, 0], tud_red, label="$Q_{h,(1,1)}(t)$")
            # # ax2f.plot(T, Qhhat[:-1, 0, 0], tud_red, linestyle="--", alpha=0.5, label="$\hat{Q}_{h,(1,1)}(t)$")
            # # ax2f.plot(T, Qr[:, 0, 0], tud_blue, label="$Q_{r,(1,1)}(t)$")
            # # ax2g.plot(T, Qh[:, 1, 1], tud_red, label="$Q_{h,(2,2)}(t)$")
            # # ax2g.plot(T, Qhhat[:-1, 1, 1], tud_red, linestyle="--", alpha=0.5, label="$\hat{Q}_{h,(2,2)}(t)$")
            # # ax2g.plot(T, Qr[:, 1, 1], tud_blue, label="$Q_{r,(2,2)}(t)$")
            #
            # ax2f.set_title('Position cost weight', **csfont)
            # ax2f.set_xlabel('Time (s)', **hfont)
            # ax2f.set_ylabel('Weight value (-)', **hfont)
            # ax2f.legend(prop={"size": 8}, loc='upper right')
            # ax2f.set(xlim=(0, t[-1]))
            #
            # ax2g.set_title('Velocity cost weight', **csfont)
            # ax2g.set_xlabel('Time (s)', **hfont)
            # ax2g.set_ylabel('Weight value (-)', **hfont)
            # ax2g.legend(prop={"size": 8}, loc='upper right')
            # ax2g.set(xlim=(0, t[-1]))

            fig2.tight_layout(pad=1)
            self.save_figure_dump(type=2)

        fig3 = plt.figure()
        title = type
        fig3.suptitle(title, **csfont)

        gs3 = fig3.add_gridspec(2, 3)
        ax3a = fig3.add_subplot(gs3[0, :])
        ax3b = fig3.add_subplot(gs3[1, 0])
        ax3c = fig3.add_subplot(gs3[1, 1])
        ax3d = fig3.add_subplot(gs3[1, 2])

        ax3a.plot(t, e, tud_blue, linestyle="--", linewidth=2.5, label="Error $\\xi(t)$")
        ax3a.plot(t_sim, e_sim, tud_blue, linewidth=2.5, linestyle="-", alpha=0.5,
                  label="Simulated error $\\xi_{sim}(t)$")
        ax3a.set_title('Error', **csfont)
        ax3a.set_xlabel('Time (s)', **hfont)
        ax3a.set_ylabel('Position error (m)', **hfont)

        ax3b.plot(t, edot, tud_blue, linestyle="--", linewidth=2, label="Error $\\dot{\\xi}(t)$")
        ax3b.plot(t_sim, edot_sim, tud_blue, linestyle="-", linewidth=2, alpha=0.5,
                  label="Simulated error $\\dot{\\xi}_{sim}(t)$")
        ax3b.set_title('Velocity error', **csfont)
        ax3b.set_xlabel('Time (s)', **hfont)
        ax3b.set_ylabel('Velocity error (m/s)', **hfont)

        ax3a.legend(prop={"size": 8}, loc='upper right')
        ax3a.set(xlim=(0, t[-1]))
        ax3b.legend(prop={"size": 8}, loc='lower right')
        ax3b.set(xlim=(0, t[-1]))

        ax3c.plot(t, ureal, tud_red, linewidth=2.5, linestyle="-", alpha=1, label="Measured input $u_{meas}(t)$")
        ax3c.plot(t, ur, tud_blue, linewidth=2.5, linestyle="--", alpha=1, label="Commanded input $u(t)$")
        ax3c.plot(t, ur_sim, tud_blue, linewidth=2.5, linestyle="-", alpha=0.7, label="Simulated input $u_{sim}(t)$")
        ax3c.set_title('Loop time', **csfont)
        ax3c.set_xlabel('Time (s)', **hfont)
        ax3c.set_ylabel('Time (s)', **hfont)
        ax3c.legend(prop={"size": 8}, loc='upper right')
        ax3c.set(xlim=(0, t[-1]))

        ax3d.plot(t, t_ex, tud_blue, linewidth=0.7, linestyle="-", alpha=1, label="Execution time $r(t)$")
        ax3d.set_title('Loop time', **csfont)
        ax3d.set_xlabel('Time (s)', **hfont)
        ax3d.set_ylabel('Time (s)', **hfont)
        ax3d.legend(prop={"size": 8}, loc='upper right')
        ax3d.set(xlim=(0, t[-1]))

        fig3.tight_layout(pad=1)
        self.save_figure_dump(type=3)

        plt.show()

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
