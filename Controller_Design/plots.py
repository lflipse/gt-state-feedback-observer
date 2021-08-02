import matplotlib.pyplot as plt
import os
from datetime import datetime
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages

class PlotStuff:
    def __init__(self):
        print("Plotting stuff")

    def plot_stuff(self, data, type, virtual_human):
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
        ur = data["torque"]
        # ureal = exp_data["measured_input"]
        x = data["steering_angle"]
        ref = np.array(data["reference"])
        r = ref[:, 0]
        xdot = data["steering_rate"]
        rdot = ref[:, 1]
        e = data["angle_error"]
        edot = data["rate_error"]
        t_ex = data["execution_time"]

        if type == "Gain_observer":
            x_hat = np.array(data["state_estimate"])
            Lhhat_pos = data["estimated_human_gain_pos"]
            Lhhat_vel = data["estimated_human_gain_vel"]
            Lr_pos = data["robot_gain_pos"]
            Lr_vel = data["robot_gain_vel"]
            uhhat = data["estimated_human_input"]
            uh_vir = data["virtual_human_torque"]
            if virtual_human:
                Lh_vir_pos = data["virtual_human_gain_pos"]
                Lh_vir_vel = data["virtual_human_gain_vel"]
            uhtilde = data["input_estimation_error"]
            xddot = data["acceleration"]
            xi_gamma = np.array(data["xi_gamma"])
            xhatdot = np.array(data["state_estimate_derivative"])
            # xdot_test = np.array(data["xdot_test"])
            # print(xi_gamma.shape)

        # Plot stuff
        fig0 = plt.figure()
        fig0.suptitle("follow the money", **csfont)
        gs0 = fig0.add_gridspec(2, 2)
        ax0a = fig0.add_subplot(gs0[0, 0])
        ax0b = fig0.add_subplot(gs0[0, 1])
        ax0c = fig0.add_subplot(gs0[1, 0])
        ax0d = fig0.add_subplot(gs0[1, 1])

        fig1 = plt.figure()
        title = type
        fig1.suptitle(title, **csfont)
        xtilde1 = np.array(x_hat[:, 0].flatten()) - np.array(x)
        xtilde2 = np.array(x_hat[:, 1].flatten()) - np.array(xdot)
        ax0a.plot(t, xtilde1, tud_blue, linestyle="--", linewidth=2.5, label="Estimation error $\\tilde{x}_1(t)$")
        ax0a.plot(t, xtilde2, tud_red, linestyle="-", linewidth=2.5, label="Estimation error $\\tilde{x}_2(t)$")
        ax0b.plot(t, uhtilde, tud_blue, linestyle="--", linewidth=2.5, label="Input estimate $\\tilde{u}_h(t)$")
        ax0b.plot(t, xi_gamma[:, 0, 1], tud_red, linestyle="-", linewidth=2.5, alpha=0.7, label="$\\xi_{gamma}2(t)$")
        ax0c.plot(t, xddot, tud_blue, linestyle="-", linewidth=2.5, label="Acceleration $\\ddot{x}(t)$")


        # print(xhatdot.shape)
        ax0c.plot(t, xhatdot[:, 1, 0], tud_red, linestyle="-", linewidth=2.5, alpha=0.7, label="$\\ddot{\hat{x}}(t)$")
        ax0d.plot(t, xi_gamma[:, 0, 0], tud_blue, linestyle="-", linewidth=2.5, label="$\\xi_{gamma}1(t)$")
        ax0d.plot(t, xi_gamma[:, 0, 1], tud_red, linestyle="-", linewidth=2.5, label="$\\xi_{gamma}2(t)$")

        print(np.mean(xi_gamma[:, 0, 0]))
        print(np.mean(xi_gamma[:, 0, 1]))

        ax0a.legend(prop={"size": 8}, loc='upper right')
        ax0a.set(xlim=(0, t[-1]))
        ax0b.legend(prop={"size": 8}, loc='upper right')
        ax0b.set(xlim=(0, t[-1]))
        ax0c.legend(prop={"size": 8}, loc='upper right')
        ax0c.set(xlim=(0, t[-1]))
        ax0d.legend(prop={"size": 8}, loc='upper right')
        ax0d.set(xlim=(0, t[-1]))

        gs1 = fig1.add_gridspec(2, 2)
        ax1a = fig1.add_subplot(gs1[0, :])
        ax1b = fig1.add_subplot(gs1[1, 0])
        ax1c = fig1.add_subplot(gs1[1, 1])
        print(x_hat.shape)

        ax1a.plot(t, r, tud_black, linewidth=2.5, linestyle="-", alpha=0.7, label="Reference $r(t)$")
        ax1a.plot(t, x, tud_blue, linestyle="--", linewidth=2.5, label="State $x(t)$")
        ax1a.plot(t, x_hat[:, 0], tud_red, linestyle="--", linewidth=2.5, label="Estimated state $\hat{x}(t)$")
        ax1a.set_title('Position', **csfont)
        ax1a.set_xlabel('Time (s)', **hfont)
        ax1a.set_ylabel('Position (m)', **hfont)

        ax1b.plot(t, rdot, tud_black, linewidth=2, linestyle="-", alpha=0.7, label="Reference $r(t)$")
        ax1b.plot(t, xdot, tud_blue, linestyle="--", linewidth=2, label="Velocity $v(t)$")
        ax1b.plot(t, x_hat[:, 1], tud_red, linestyle="--", linewidth=2.5, label="Estimated velocity $\hat{v}(t)$")
        ax1b.set_title('Velocity', **csfont)
        ax1b.set_xlabel('Time (s)', **hfont)
        ax1b.set_ylabel('Velocity (m/s)', **hfont)

        ax1c.plot(t, ur, tud_blue, linestyle="--", linewidth=2, label="$u_r(t)$")

        # ax1c.plot(t, uh, tud_red, label="$u_h(t)$")
        if type == "Gain_observer":
            ax1c.plot(t, uhhat, tud_red, linestyle="--", linewidth=2, alpha=1, label="$\hat{u}_h(t)$")

        if virtual_human:
            ax1c.plot(t, uh_vir, tud_green, linestyle="--", linewidth=2, alpha=1, label="$u_{h,vir}(t)$")

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

        # plt.figure()
        # plt.plot(t, xddot)

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

            if virtual_human:
                ax2a.plot(t, Lh_vir_pos, tud_green, linewidth=2.5, linestyle="--", alpha=1, label="$L_{h_{vir},1}(t)$")
                ax2b.plot(t, Lh_vir_vel, tud_green, linewidth=2.5, linestyle="--", alpha=1, label="$L_{h_{vir},2}(t)$")

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

    def plot_stuff_with_sim_data(self, exp_data, sim_data, type, virtual_human):
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
        ur = exp_data["torque"]
        # ureal = exp_data["measured_input"]
        x = exp_data["steering_angle"]

        ref = np.array(exp_data["reference"])
        r = ref[:, 0]
        xdot = exp_data["steering_rate"]
        rdot = ref[:, 1]
        e = exp_data["angle_error"]
        edot = exp_data["rate_error"]
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
            x_hat = np.array(exp_data["state_estimate"])
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
            uhtilde = exp_data["input_estimation_error"]
            xddot = exp_data["acceleration"]
            xi_gamma = np.array(exp_data["xi_gamma"])
            xhatdot = np.array(exp_data["state_estimate_derivative"])
            ydot_sim = sim_data["ydot"]
            acc_sim = ydot_sim[:, 1]
            if virtual_human:
                Lh_vir_pos = exp_data["virtual_human_gain_pos"]
                Lh_vir_vel = exp_data["virtual_human_gain_vel"]
                Qh_vir_pos = exp_data["virtual_human_cost_pos"]
                Qh_vir_vel = exp_data["virtual_human_cost_vel"]
                uh_vir = exp_data["virtual_human_torque"]



        # Plot stuff
        # plt.figure()
        # plt.plot(t, xddot, tud_blue)
        # plt.plot(t_sim, acc_sim, tud_red)


        
        fig1 = plt.figure()
        title = type
        fig1.suptitle(title, **csfont)

        gs1 = fig1.add_gridspec(2, 2)
        ax1a = fig1.add_subplot(gs1[0, :])
        ax1b = fig1.add_subplot(gs1[1, 0])
        ax1c = fig1.add_subplot(gs1[1, 1])

        ax1a.plot(t, r, tud_black, linewidth=2.5, linestyle="-", alpha=0.7, label="Reference $r(t)$")
        # ax1a.plot(t, r_sim, tud_black, linewidth=2.5, linestyle="--", alpha=0.3, label="Reference $r_{sim}(t)$")
        ax1a.plot(t, x, tud_blue, linestyle="--", linewidth=2.5, label="State $x(t)$")
        ax1a.plot(t_sim, x_sim[:-1], tud_blue, linewidth=2.5, linestyle="-", alpha=0.5, label="Simulated state $x_{sim}(t)$")
        if type == "Gain_observer":
            ax1a.plot(t, x_hat[:, 0], tud_red, linewidth=2.5, linestyle="--", label="Estimated state $\hat{x}(t)$")
        ax1a.set_title('Position', **csfont)
        ax1a.set_xlabel('Time (s)', **hfont)
        ax1a.set_ylabel('Position (m)', **hfont)


        ax1b.plot(t, rdot, tud_black, linewidth=2, linestyle="-", alpha=0.7, label="Reference $r(t)$")
        # ax1b.plot(t, rdot_sim, tud_black, linewidth=2, linestyle="-", alpha=0.3, label="Reference $r_{sim}(t)$")
        ax1b.plot(t, xdot, tud_blue, linestyle="--", linewidth=2, label="Velocity $v(t)$")
        ax1b.plot(t_sim, xdot_sim[:-1], tud_blue, linestyle="-", linewidth=2, alpha=0.5, label="Simulated velocity $v_{sim}(t)$")
        if type == "Gain_observer":
            ax1b.plot(t, x_hat[:, 1], tud_red, linewidth=2.5, linestyle="--", alpha=1, label="Estimated state $\dot{\hat{x}}(t)$")
        ax1b.set_title('Velocity', **csfont)
        ax1b.set_xlabel('Time (s)', **hfont)
        ax1b.set_ylabel('Velocity (m/s)', **hfont)

        ax1c.plot(t, ur, tud_blue, linestyle="--", linewidth=2, label="$u_r(t)$")
        ax1c.plot(t_sim, ur_sim, tud_blue, linestyle="-", linewidth=2, alpha=0.5, label="$u_{r,sim}(t)$")
        # ax1c.plot(t, uh, tud_red, label="$u_h(t)$")
        if type == "Gain_observer":
            ax1c.plot(t, uhhat, tud_red, linestyle="--", linewidth=2, alpha=1, label="$\hat{u}_h(t)$")
            ax1c.plot(t_sim, uhhat_sim, tud_red, linewidth=2, linestyle="-", alpha=0.5, label="$\hat{u}_h(t)$")

        if virtual_human:
            ax1c.plot(t, uh_vir, tud_green, linestyle="--", linewidth=2, alpha=1, label="$u_{h,vir}(t)$")

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
            if virtual_human:
                ax2a.plot(t, Lh_vir_pos, tud_green, linewidth=2.5, linestyle="--", alpha=1, label="$L_{h_{vir},1}(t)$")
                ax2b.plot(t, Lh_vir_vel, tud_green, linewidth=2.5, linestyle="--", alpha=1, label="$L_{h_{vir},2}(t)$")

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

        delta = x - x_sim[:-1]
        deltadot = xdot - xdot_sim[:-1]
        u = np.array(ur)
        u_sim = np.array(ur_sim)
        du = u.flatten() - u_sim


        # ax3c.plot(t, r, tud_black, linewidth=2.5, linestyle="-", alpha=1, label="$r(t)$")
        # ax3c.plot(t, rdot, tud_black, linewidth=1.5, linestyle="-.", alpha=1, label="$\dot{r}(t)$")
        ax3c.plot(t, delta, tud_blue, linewidth=2.5, linestyle="--", alpha=1, label="$\Delta x(t)$")
        ax3c.plot(t, deltadot, tud_red, linewidth=1.5, linestyle="-", alpha=0.5, label="$\Delta \dot{x}(t)$")
        ax3c.plot(t, du, tud_green, linewidth=1.5, linestyle="-", alpha=0.5, label="$\Delta u(t)$")
        ax3c.set_title('Extra dynamics', **csfont)
        ax3c.set_xlabel('$\Delta$', **hfont)
        ax3c.set_ylabel('Time (s)', **hfont)
        ax3c.legend(prop={"size": 8}, loc='upper right')
        ax3c.set(xlim=(0, t[-1]))

        ax3d.plot(t, t_ex, tud_blue, linewidth=0.7, linestyle="-", alpha=1, label="Execution time $t_{exc}(t)$")
        ax3d.set_title('Loop time', **csfont)
        ax3d.set_xlabel('Time (s)', **hfont)
        ax3d.set_ylabel('Time (s)', **hfont)
        ax3d.legend(prop={"size": 8}, loc='upper right')
        ax3d.set(xlim=(0, t[-1]))

        fig3.tight_layout(pad=1)
        self.save_figure_dump(type=3)

        plt.show()

    def plot_report(self, exp_data, sim_data):
        csfont = {'fontname': 'Georgia'}
        hfont = {'fontname': 'Georgia'}

        # Colors
        tud_blue = "#0066A2"
        tud_black = "#000000"
        tud_red = "#c3312f"
        tud_green = "#00A390"
        tud_yellow = "#F1BE3E"

        # UNPACK DATA
        # General
        t = exp_data["time"]
        ur = exp_data["torque"]
        x = exp_data["steering_angle"]
        ref = np.array(exp_data["reference"])
        r = ref[:, 0]
        xdot = exp_data["steering_rate"]
        rdot = ref[:, 1]
        e = exp_data["angle_error"]
        edot = exp_data["rate_error"]
        t_ex = exp_data["execution_time"]

        if sim_data != None:

            # Simulation
            t_sim = sim_data["time"]
            ur_sim = sim_data["robot_input"]
            states_sim = sim_data["states"]
            x_sim = states_sim[:, 0]
            xdot_sim = states_sim[:, 1]
            xi_sim = sim_data["error_states"]
            e_sim = xi_sim[:, 0]
            edot_sim = xi_sim[:, 1]

            Lhhat_sim = sim_data["human_estimated_gain"]
            Lhhat_pos_sim = Lhhat_sim[:, 0]
            Lhhat_vel_sim = Lhhat_sim[:, 1]
            Lr_sim = sim_data["robot_gain"]
            Lr_pos_sim = Lr_sim[:, 0]
            Lr_vel_sim = Lr_sim[:, 1]
            uhhat_sim = sim_data["human_estimated_input"]

            ydot_sim = sim_data["ydot"]
            acc_sim = ydot_sim[:, 1]


        x_hat = np.array(exp_data["state_estimate"])
        Lhhat_pos = exp_data["estimated_human_gain_pos"]
        Lhhat_vel = exp_data["estimated_human_gain_vel"]
        Lr_pos = exp_data["robot_gain_pos"]
        Lr_vel = exp_data["robot_gain_vel"]
        uhhat = exp_data["estimated_human_input"]
        uhtilde = exp_data["input_estimation_error"]

        uhtilde = exp_data["input_estimation_error"]
        xddot = exp_data["acceleration"]
        xi_gamma = np.array(exp_data["xi_gamma"])
        xhatdot = np.array(exp_data["state_estimate_derivative"])

        Lh_vir_pos = exp_data["virtual_human_gain_pos"]
        Lh_vir_vel = exp_data["virtual_human_gain_vel"]
        uh_vir = exp_data["virtual_human_torque"]

        # Steering angle
        plt.figure()
        plt.title("Measured and estimated steering angle", **csfont)
        plt.plot(t, r, tud_black, linewidth=2.5, linestyle="-", alpha=0.7, label="Reference $\phi_r(t)$")
        plt.plot(t, x, tud_blue, linestyle="--", linewidth=2.5, label="Steering angle $\phi(t)$")

        if sim_data != None:
            plt.plot(t_sim, x_sim[:-1], tud_blue, linewidth=2.5, linestyle="-", alpha=0.5,
                    label="Simulated $\phi_{sim}(t)$")
        plt.plot(t, x_hat[:, 0], tud_red, linewidth=2.5, linestyle="--", label="Estimated $\hat{\phi}(t)$")
        plt.xlabel('Time (s)', **hfont)
        plt.ylabel('Steering angle (rad)', **hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        # Steering rate
        plt.figure()
        plt.title("Measured and estimated steering rate", **csfont)
        plt.plot(t, rdot, tud_black, linewidth=2.5, linestyle="-", alpha=0.7, label="Reference $\dot{\phi}_r(t)$")
        plt.plot(t, xdot, tud_blue, linestyle="--", linewidth=2.5, label="Steering rate $\dot{\phi}(t)$")
        if sim_data != None:
            plt.plot(t_sim, xdot_sim[:-1], tud_blue, linewidth=2.5, linestyle="-", alpha=0.5,
                    label="Simulated $\dot{\phi}_{sim}(t)$")
        plt.plot(t, xhatdot[:, 0], tud_red, linewidth=2.5, linestyle="--", label="Estimated $\dot{\hat{\phi}}(t)$")
        plt.xlabel('Time (s)', **hfont)
        plt.ylabel('Steering rate (rad/s)', **hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        # Input torque (robot)
        plt.figure()
        plt.title("Input torque", **csfont)
        plt.plot(t, ur, tud_blue, linestyle="--", linewidth=2, label="Input torque (robot) $u_r(t)$")
        plt.plot(t, uhhat, tud_red, linestyle="--", linewidth=2, alpha=1, label="Estimated (human) $\hat{u}_h(t)$")
        if sim_data != None:
            plt.plot(t_sim, ur_sim, tud_blue, linestyle="-", linewidth=2, alpha=0.5, label="Simulated (robot) $u_{r,sim}(t)$")
            plt.plot(t_sim, uhhat_sim, tud_red, linewidth=2, linestyle="-", alpha=0.5, label="Simulated (human) $\hat{u}_{h,sim}(t)$")
        plt.plot(t, uh_vir, tud_black, linestyle="--", linewidth=2, alpha=1, label="Virtual (human) $u_{h,vir}(t)$")
        plt.xlabel('Time (s)', **hfont)
        plt.ylabel('Torque (Nm)', **hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        # Input torque (human)
        plt.figure()
        plt.title("Input torque", **csfont)
        # plt.plot(t, ur, tud_blue, linestyle="--", linewidth=2, label="Input torque (robot) $u_r(t)$")
        if sim_data != None:
            plt.plot(t_sim, ur_sim, tud_blue, linestyle="-", linewidth=2, alpha=0.5,
                    label="Simulated (robot) $u_{r,sim}(t)$")
        # plt.plot(t, uhhat, tud_red, linestyle="--", linewidth=2, alpha=1, label="Estimated (human) $\hat{u}_h(t)$")
            plt.plot(t_sim, uhhat_sim, tud_red, linewidth=2, linestyle="-", alpha=0.5,
                    label="Simulated (human) $\hat{u}_{h,sim}(t)$")
        # plt.plot(t, uh_vir, tud_black, linestyle="--", linewidth=2, alpha=1, label="Virtual (human) $u_{h,vir}(t)$")
        plt.xlabel('Time (s)', **hfont)
        plt.ylabel('Torque (Nm)', **hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, 10)
        plt.tight_layout(pad=1)

        options = {"arrowstyle": '->'}

        n = len(Lh_vir_vel)

        # Controller gains
        plt.figure()
        plt.plot(t, Lr_pos, tud_blue, linewidth=2.5, linestyle="--", label="Gain (robot) $L_{r}(t)$")
        if sim_data != None:
            plt.plot(t_sim, Lr_pos_sim, tud_blue, linewidth=2.5, label="Simulated $L_{r,sim}(t)$", alpha=0.5, )
            plt.plot(t_sim, Lhhat_pos_sim, tud_red, linewidth=2.5, linestyle="-", alpha=0.5,
                     label="Simulated (human) $\hat{L}_{h,sim}(t)$")
        plt.plot(t, Lhhat_pos, tud_red, linewidth=2.5, linestyle="--", alpha=1, label="Estimated (human) $\hat{L}_{h}(t)$")
        plt.plot(t, Lh_vir_pos, tud_black, linewidth=2.5, linestyle="--", alpha=1, label="Virtual (human) $L_{h,vir}(t)$")

        try:
            self.stepinfo(t, Lhhat_pos, Lh_vir_pos)
        except:
            print("does not work")

        plt.title('Steering angle gain', **csfont)
        plt.xlabel('Time (s)', **hfont)
        plt.ylabel('Gain value (Nm/rad)', **hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, 15)
        plt.ylim(-0.2, 6)

        try:
            Qh1 = exp_data["estimated_human_cost_1"]
            Qh2 = exp_data["estimated_human_cost_2"]
            Qh_vir_pos = exp_data["virtual_human_cost_pos"]
            Qh_vir_vel = exp_data["virtual_human_cost_vel"]

            plt.figure()
            # plt.plot(t, Lr_pos, tud_blue, linewidth=2.5, linestyle="--", label="Gain (robot) $L_{r}(t)$")
            # if sim_data != None:
            #     plt.plot(t_sim, Lr_pos_sim, tud_blue, linewidth=2.5, label="Simulated $L_{r,sim}(t)$", alpha=0.5, )
            #     plt.plot(t_sim, Lhhat_pos_sim, tud_red, linewidth=2.5, linestyle="-", alpha=0.5,
            #              label="Simulated (human) $\hat{L}_{h,sim}(t)$")
            plt.plot(t, Qh1, tud_red, linewidth=2.5, linestyle="--", alpha=1,
                     label="Estimated (human) $\hat{Q}_{h,1}(t)$")
            plt.plot(t, Qh_vir_pos, tud_black, linewidth=2.5, linestyle="--", alpha=1,
                     label="Virtual (human) $Q_{h,1,vir}(t)$")
            plt.annotate("Weak action", (7.5, Qh_vir_pos[int(n / 12)]), xytext=(7.5, Qh_vir_pos[int(n / 12)] + 5),
                         arrowprops=options)
            plt.annotate("No interaction", (22.5, Qh_vir_pos[int(3 * n / 12)]),
                         xytext=(18, Qh_vir_pos[int(3 * n / 12)] - 5), arrowprops=options)
            plt.annotate("Strong action", (37.5, Qh_vir_pos[int(5 * n / 12)]),
                         xytext=(34, Qh_vir_pos[int(5 * n / 12)] + 5), arrowprops=options)
            plt.annotate("No interaction", (52.5, Qh_vir_pos[int(7 * n / 12)]),
                         xytext=(48, Qh_vir_pos[int(7 * n / 12)] + 5), arrowprops=options)
            plt.annotate("Counteracting steering", (67.5, Qh_vir_pos[int(9 * n / 12)]),
                         xytext=(48, Qh_vir_pos[int(9 * n / 12)] - 5), arrowprops=options)
            plt.annotate("No interaction", (82.5, Qh_vir_pos[int(11 * n / 12)]),
                         xytext=(70, Qh_vir_pos[int(11 * n / 12)] + 5), arrowprops=options)

            plt.title('Steering angle error weight', **csfont)
            plt.xlabel('Time (s)', **hfont)
            plt.ylabel('Weight value (-)', **hfont)
            plt.legend(prop={"size": 8}, loc='upper right')
            plt.xlim(0, t[-1])
            plt.ylim(-30, 35)

            plt.figure()
            # plt.plot(t, Lr_pos, tud_blue, linewidth=2.5, linestyle="--", label="Gain (robot) $L_{r}(t)$")
            # if sim_data != None:
            #     plt.plot(t_sim, Lr_pos_sim, tud_blue, linewidth=2.5, label="Simulated $L_{r,sim}(t)$", alpha=0.5, )
            #     plt.plot(t_sim, Lhhat_pos_sim, tud_red, linewidth=2.5, linestyle="-", alpha=0.5,
            #              label="Simulated (human) $\hat{L}_{h,sim}(t)$")
            plt.plot(t, Qh2, tud_red, linewidth=2.5, linestyle="--", alpha=1,
                     label="Estimated (human) $\hat{Q}_{h,2}(t)$")
            plt.plot(t, Qh_vir_vel, tud_black, linewidth=2.5, linestyle="--", alpha=1,
                     label="Virtual (human) $Q_{h,2,vir}(t)$")
            plt.annotate("Weak action", (7.5, Qh_vir_vel[int(n / 12)]), xytext=(7.5, Qh_vir_vel[int(n / 12)] + 0.5),
                         arrowprops=options)
            plt.annotate("No interaction", (22.5, Qh_vir_vel[int(3 * n / 12)]),
                         xytext=(18, Qh_vir_vel[int(3 * n / 12)] - 0.5), arrowprops=options)
            plt.annotate("Strong action", (37.5, Qh_vir_vel[int(5 * n / 12)]),
                         xytext=(34, Qh_vir_vel[int(5 * n / 12)] + 0.5), arrowprops=options)
            plt.annotate("No interaction", (52.5, Qh_vir_vel[int(7 * n / 12)]),
                         xytext=(48, Qh_vir_vel[int(7 * n / 12)] + 0.5), arrowprops=options)
            plt.annotate("Counteracting steering", (67.5, Qh_vir_vel[int(9 * n / 12)]),
                         xytext=(48, Qh_vir_vel[int(9 * n / 12)] - 0.5), arrowprops=options)
            plt.annotate("No interaction", (82.5, Qh_vir_vel[int(11 * n / 12)]),
                         xytext=(70, Qh_vir_vel[int(11 * n / 12)] + 0.5), arrowprops=options)

            plt.title('Steering rate error weight', **csfont)
            plt.xlabel('Time (s)', **hfont)
            plt.ylabel('Weight value (-)', **hfont)
            plt.legend(prop={"size": 8}, loc='upper right')
            plt.xlim(0, t[-1])
            plt.ylim(-1, 1)

            plt.figure()
            plt.plot(t, Qh1)
            plt.plot(t, Qh_vir_pos)
            plt.figure()
            plt.plot(t, Qh2)
            plt.plot(t, Qh_vir_vel)
        except:
            Qh = np.array(exp_data["estimated_human_cost"])
            print(Qh.size)
            print("well that did not work...")

        plt.figure()
        plt.plot(t, Lr_pos, tud_blue, linewidth=2.5, linestyle="--", label="Gain (robot) $L_{r}(t)$")
        if sim_data != None:
            plt.plot(t_sim, Lr_pos_sim, tud_blue, linewidth=2.5, label="Simulated $L_{r,sim}(t)$", alpha=0.5, )
            plt.plot(t_sim, Lhhat_pos_sim, tud_red, linewidth=2.5, linestyle="-", alpha=0.5,
                     label="Simulated (human) $\hat{L}_{h,sim}(t)$")
        plt.plot(t, Lhhat_pos, tud_red, linewidth=2.5, linestyle="--", alpha=1,
                 label="Estimated (human) $\hat{L}_{h}(t)$")
        plt.plot(t, Lh_vir_pos, tud_black, linewidth=2.5, linestyle="--", alpha=1,
                 label="Virtual (human) $L_{h,vir}(t)$")
        plt.annotate("Weak action", (7.5, Lh_vir_pos[int(n/12)]), xytext=(7.5, Lh_vir_pos[int(n/12)] + 2), arrowprops=options)
        plt.annotate("No interaction", (22.5, Lh_vir_pos[int(3*n/12)]), xytext=(18, Lh_vir_pos[int(3*n/12)] - 2), arrowprops=options)
        plt.annotate("Strong action", (37.5, Lh_vir_pos[int(5*n/12)]), xytext=(34, Lh_vir_pos[int(5*n/12)] + 1.5), arrowprops=options)
        plt.annotate("No interaction", (52.5, Lh_vir_pos[int(7*n/12)]), xytext=(48, Lh_vir_pos[int(7*n/12)] + 2), arrowprops=options)
        plt.annotate("Counteracting steering", (67.5, Lh_vir_pos[int(9*n/12)]), xytext=(48, Lh_vir_pos[int(9*n/12)] - 1.5), arrowprops=options)
        plt.annotate("No interaction", (82.5, Lh_vir_pos[int(11*n/12)]), xytext=(70, Lh_vir_pos[int(11*n/12)] + 2), arrowprops=options)

        plt.title('Steering angle gain', **csfont)
        plt.xlabel('Time (s)', **hfont)
        plt.ylabel('Gain value (Nm/rad)', **hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, t[-1])
        plt.ylim(-4.5, 9)



        plt.figure()
        plt.plot(t, Lr_vel, tud_blue, linewidth=2.5, linestyle="--", label="Gain (robot) $L_{r}(t)$")
        plt.plot(t, Lhhat_vel, tud_red, linewidth=2.5, linestyle="--", alpha=1,
                 label="Estimated (human) $\hat{L}_{h}(t)$")
        if sim_data != None:
            no_sim = True
            plt.plot(t_sim, Lr_vel_sim, tud_blue, linewidth=2.5, label="Simulated (robot) $L_{r,sim}(t)$", alpha=0.5)
            plt.plot(t_sim, Lhhat_vel_sim, tud_red, linewidth=2.5, linestyle="-", alpha=0.5,
                 label="Simulated (human) $\hat{L}_{h,sim}(t)$")
        else:
            no_sim = False

        plt.plot(t, Lh_vir_vel, tud_black, linewidth=2.5, linestyle="--", alpha=1, label="Virtual (human) $L_{h,vir}(t)$")

        plt.annotate("Weak action", (7.5, Lh_vir_vel[int(n/12)]), xytext=(7.5, Lh_vir_vel[int(n/12)] + 0.5), arrowprops=options)
        plt.annotate("No interaction", (22.5, Lh_vir_vel[int(3*n/12)]), xytext=(15, Lh_vir_vel[int(3*n/12)] - 0.5), arrowprops=options)
        plt.annotate("Strong action", (37.5, Lh_vir_vel[int(5*n/12)]), xytext=(20, Lh_vir_vel[int(5*n/12)] + 0.2), arrowprops=options)
        plt.annotate("No interaction", (52.5, Lh_vir_vel[int(7*n/12)]), xytext=(48, Lh_vir_vel[int(7*n/12)] + 0.65), arrowprops=options)
        plt.annotate("Counteracting steering", (67.5, Lh_vir_vel[int(9*n/12)]), xytext=(55, Lh_vir_vel[int(9*n/12)] - 0.2),
                     arrowprops=options)
        plt.annotate("No interaction", (82.5, Lh_vir_vel[int(11*n/12)]), xytext=(70, Lh_vir_vel[int(11*n/12)] + 0.65), arrowprops=options)

        plt.title('Steering rate gain', **csfont)
        plt.xlabel('Time (s)', **hfont)
        plt.ylabel('Gain value (Nms/rad)', **hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, t[-1])
        plt.ylim(-0.9, 1.5)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(t, Lr_vel, tud_blue, linewidth=2.5, linestyle="--", label="Gain (robot) $L_{r}(t)$")
        plt.plot(t, Lhhat_vel, tud_red, linewidth=2.5, linestyle="--", alpha=1,
                 label="Estimated (human) $\hat{L}_{h}(t)$")
        if sim_data != None:
            no_sim = True
            plt.plot(t_sim, Lr_vel_sim, tud_blue, linewidth=2.5, label="Simulated (robot) $L_{r,sim}(t)$", alpha=0.5)
            plt.plot(t_sim, Lhhat_vel_sim, tud_red, linewidth=2.5, linestyle="-", alpha=0.5,
                     label="Simulated (human) $\hat{L}_{h,sim}(t)$")
        else:
            no_sim = False

        plt.plot(t, Lh_vir_vel, tud_black, linewidth=2.5, linestyle="--", alpha=1,
                 label="Virtual (human) $L_{h,vir}(t)$")

        try:
            self.stepinfo(t, Lhhat_vel, Lh_vir_vel)
        except:
            print("does not work")

        plt.title('Steering rate gain', **csfont)
        plt.xlabel('Time (s)', **hfont)
        plt.ylabel('Gain value (Nms/rad)', **hfont)
        plt.legend(prop={"size": 8}, loc='upper right')
        plt.xlim(0, 15)
        plt.ylim(-0.2, 0.6)
        plt.tight_layout(pad=1)

        self.save_all_figures(no_sim)

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

    def save_all_figures(self, no_sim):
        if no_sim is True:
            pp = PdfPages('validation_virtual.pdf')
        else:
            pp = PdfPages('validation_human.pdf')
        figs = None
        if figs is None:
            figs = [plt.figure(n) for n in plt.get_fignums()]
        for fig in figs:
            fig.savefig(pp, format='pdf')
        pp.close()
