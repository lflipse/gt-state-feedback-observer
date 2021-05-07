import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

class PlotStuff:
    def __init__(self):
        print("Plotting stuff")

    def plot_stuff(self, inputs, outputs):
        # Unpack stuff
        T = inputs["time_vector"]
        t = T[-1]
        c = inputs["c"]
        d = inputs["d"]
        controller = inputs["controller_type"]
        dynamics = inputs["dynamics_type"]
        controller_name = inputs["controller_type_name"]
        dynamics_name = inputs["dynamics_type_name"]
        save = inputs["save"]
        r = outputs["reference_signal"]
        x = outputs["states"]
        e = outputs["error_states"]
        Jr = outputs["robot_costs"]
        Jh = outputs["human_costs"]
        Jt = Jr + Jh
        ur = outputs["robot_input"]
        uh = outputs["human_input"]
        ut = ur + uh

        # c > 1 when estimating stuff
        if c > 1:
            Jhhat = outputs["human_estimated_costs"]
            uhhat = outputs["human_estimated_input"]
            Qh = outputs["human_Q"]
            Qr = outputs["robot_Q"]
            Qhhat = outputs["human_estimated_Q"]
            Lh = outputs["human_gain"]
            Lr = outputs["robot_gain"]
            Lhhat = outputs["human_estimated_gain"]
            Ph = outputs["human_P"]
            Pr = outputs["robot_P"]
            Phhat = outputs["human_estimated_P"]

        # Figure properties
        csfont = {'fontname': 'Georgia'}
        hfont = {'fontname': 'Georgia'}

        # Colors
        tud_blue = "#0066A2"
        tud_black = "#000000"
        tud_grey = "#808080"
        tud_red = "#c3312f"
        tud_green = "#00A390"

        # Plot stuff
        fig1 = plt.figure()
        title = controller + " for " + dynamics
        fig1.suptitle(title, **csfont)

        gs1 = fig1.add_gridspec(2,2)
        ax1a = fig1.add_subplot(gs1[0, :])
        ax1b = fig1.add_subplot(gs1[1, 0])
        ax1c = fig1.add_subplot(gs1[1, 1])

        ax1a.plot(T, x[:-1, 0], tud_black, label="State $x(t)$")
        ax1a.plot(T, r[:, 0], tud_black, linestyle="--", alpha=0.5, label="Reference $r(t)$")
        ax1a.set_title('Position', **csfont)
        ax1a.set_xlabel('Time (s)', **hfont)
        ax1a.set_ylabel('Position (m)', **hfont)

        ax1b.plot(T[1:], Jr[1:], tud_blue, label="$J_r(t)$")
        ax1b.plot(T[1:], Jh[1:], tud_red, label="$J_h(t)$")
        # Estimators for the human
        if c > 1:
            ax1b.plot(T[1:], Jhhat[1:], tud_red, linestyle="--", alpha=0.5, label="$\hat{J}_h(t)$")

        ax1b.set_title('Cost function', **csfont)
        ax1b.set_xlabel('Time (s)', **hfont)
        ax1b.set_ylabel('Cost function value (-)', **hfont)

        ax1c.plot(T, ur, tud_blue, label="$u_r(t)$")
        ax1c.plot(T, uh, tud_red, label="$u_h(t)$")
        if c > 1:
            ax1c.plot(T, uhhat, tud_red, linestyle="--", alpha=0.5, label="$\hat{u}_h(t)$")
        ax1c.plot(T, ut, tud_green, alpha=1, label="$u(t)$")
        ax1c.set_title('Control action', **csfont)
        ax1c.set_xlabel('Time (s)', **hfont)
        ax1c.set_ylabel('Input force (N)', **hfont)

        ax1a.legend(prop={"size": 8}, loc='upper right')
        ax1a.set(xlim=(0, t))
        ax1b.legend(prop={"size": 8}, loc='upper right')
        ax1b.set(xlim=(0, t))
        ax1c.legend(prop={"size": 8}, loc='upper right')
        ax1c.set(xlim=(0, t))

        fig1.tight_layout(pad=1)

        if save > 0:
            location = "C:\\Users\\Lorenzo\\Repositories\\thesis\\img\\simulations\\"
            string = location + dynamics_name + "_" + controller_name + ".pdf"
            plt.savefig(string)


        if c > 1:
            fig2 = plt.figure()
            title = controller + " for " + dynamics
            fig2.suptitle(title, **csfont)

            gs2 = fig2.add_gridspec(2, 6)
            ax2a = fig2.add_subplot(gs2[0, 0:3])
            ax2b = fig2.add_subplot(gs2[0, 3:6])

            ax2f = fig2.add_subplot(gs2[1, 0:3])
            ax2g = fig2.add_subplot(gs2[1, 3:6])

            ax2a.plot(T, Lh[:, 0], tud_red, label="$L_{h,1}(t)$")
            ax2b.plot(T, Lh[:, 1], tud_red, label="$L_{h,2}(t)$")
            ax2a.plot(T, Lhhat[:, 0], tud_red, linestyle="--", alpha=0.5, label="$\hat{L}_{h,1}(t)$")
            ax2b.plot(T, Lhhat[:, 1], tud_red, linestyle="--", alpha=0.5, label="$\hat{L}_{h,2}(t)$")
            ax2a.plot(T, Lr[:, 0], tud_blue, label="$L_{r,1}(t)$")
            ax2b.plot(T, Lr[:, 1], tud_blue, label="$L_{r,2}(t)$")

            ax2a.set_title('Position gain', **csfont)
            ax2a.set_xlabel('Time (s)', **hfont)
            ax2a.set_ylabel('Gain value (N/m)', **hfont)
            ax2a.legend(prop={"size": 8}, loc='upper right')
            ax2a.set(xlim=(0, t))

            ax2b.set_title('Velocity gain', **csfont)
            ax2b.set_xlabel('Time (s)', **hfont)
            ax2b.set_ylabel('Gain value (Ns/m)', **hfont)
            ax2b.legend(prop={"size": 8}, loc='upper right')
            ax2b.set(xlim=(0, t))

            ax2f.plot(T, Qh[:, 0, 0], tud_red, label="$Q_{h,(1,1)}(t)$")
            ax2f.plot(T, Qhhat[:-1, 0, 0], tud_red, linestyle="--", alpha=0.5, label="$\hat{Q}_{h,(1,1)}(t)$")
            ax2f.plot(T, Qr[:, 0, 0], tud_blue, label="$Q_{r,(1,1)}(t)$")
            ax2g.plot(T, Qh[:, 1, 1], tud_red, label="$Q_{h,(2,2)}(t)$")
            ax2g.plot(T, Qhhat[:-1, 1, 1], tud_red, linestyle="--", alpha=0.5, label="$\hat{Q}_{h,(2,2)}(t)$")
            ax2g.plot(T, Qr[:, 1, 1], tud_blue, label="$Q_{r,(2,2)}(t)$")



            ax2f.set_title('Position cost weight', **csfont)
            ax2f.set_xlabel('Time (s)', **hfont)
            ax2f.set_ylabel('Weight value (-)', **hfont)
            ax2f.legend(prop={"size": 8}, loc='upper right')
            ax2f.set(xlim=(0, t))

            ax2g.set_title('Velocity cost weight', **csfont)
            ax2g.set_xlabel('Time (s)', **hfont)
            ax2g.set_ylabel('Weight value (-)', **hfont)
            ax2g.legend(prop={"size": 8}, loc='upper right')
            ax2g.set(xlim=(0, t))

            fig2.tight_layout(pad=0.1)

            if save > 0:
                location = "C:\\Users\\Lorenzo\\Repositories\\thesis\\img\\simulations\\"
                string = location + dynamics_name + "_" + controller_name + "_gaincostp.pdf"
                plt.savefig(string)

        if save == 0:
            plt.show()
        elif save == 2:
            plt.show()

    def plot_comparison(self, inputs, inputs2, outputs, outputs2):
        # Unpack stuff
        T = inputs["time_vector"]
        t = T[-1]
        c = inputs["c"]
        c_compare = inputs2["c"]
        d = inputs["d"]
        controller1 = inputs["controller_type"]
        controller2 = inputs2["controller_type"]
        dynamics = inputs["dynamics_type"]
        controller_name = inputs["controller_type_name"]
        controller_name2 = inputs2["controller_type_name"]
        dynamics_name = inputs["dynamics_type_name"]
        save = inputs["save"]
        r = outputs["reference_signal"]
        x1 = outputs["states"]
        x2 = outputs2["states"]
        e = outputs["error_states"]
        Jr1 = outputs["robot_costs"]
        Jh1 = outputs["human_costs"]
        Jr2 = outputs2["robot_costs"]
        Jh2 = outputs2["human_costs"]
        ur1 = outputs["robot_input"]
        uh1 = outputs["human_input"]
        ur2 = outputs2["robot_input"]
        uh2 = outputs2["human_input"]
        Qh1 = outputs["human_Q"]
        Qr1 = outputs["robot_Q"]
        Qh2 = outputs2["human_Q"]
        Qr2 = outputs2["robot_Q"]
        Lh1 = outputs["human_gain"]
        Lr1 = outputs["robot_gain"]
        Lh2 = outputs2["human_gain"]
        Lr2 = outputs2["robot_gain"]
        # ut = ur + uh

        # c > 1 when estimating stuff
        if c > 1:
            Jhhat1 = outputs["human_estimated_costs"]
            uhhat1 = outputs["human_estimated_input"]
            Qhhat1 = outputs["human_estimated_Q"]
            Lhhat1 = outputs["human_estimated_gain"]
        if c_compare > 1:
            Jhhat2 = outputs2["human_estimated_costs"]
            uhhat2 = outputs2["human_estimated_input"]
            Qhhat2 = outputs2["human_estimated_Q"]
            Lhhat2 = outputs2["human_estimated_gain"]


        # Figure properties
        csfont = {'fontname': 'Georgia'}
        hfont = {'fontname': 'Georgia'}

        # Colors
        tud_blue = "#0066A2"
        tud_black = "#000000"
        tud_grey = "#808080"
        tud_red = "#c3312f"
        tud_green = "#99D28C"
        tud_yellow = "#F1BE3E"
        tud_orange = "#EB7245"

        # Plot stuff
        fig1 = plt.figure()
        title = "Comparison: " + controller1 + " vs " + controller2
        fig1.suptitle(title, **csfont)

        gs1 = fig1.add_gridspec(2, 2)
        ax1a = fig1.add_subplot(gs1[0, :])
        ax1b = fig1.add_subplot(gs1[1, 0])
        ax1c = fig1.add_subplot(gs1[1, 1])

        ax1a.plot(T, x1[:-1, 0], tud_red, linestyle="-.", label="State $x_1(t)$")
        ax1a.plot(T, x2[:-1, 0], tud_blue, linestyle="-", alpha=1, label="State $x_2(t)$")
        ax1a.plot(T, r[:, 0], tud_black, linestyle="--", alpha=1, lw=0.5, label="Reference $r(t)$")
        ax1a.set_title('Position', **csfont)
        ax1a.set_xlabel('Time (s)', **hfont)
        ax1a.set_ylabel('Position (m)', **hfont)

        ax1b.plot(T[1:], Jr1[1:], tud_blue, linestyle="-.", label="$J_{1,r}(t)$")
        ax1b.plot(T[1:], Jh1[1:], tud_red, linestyle="-.", label="$J_{1,h}(t)$")
        ax1b.plot(T[1:], Jr2[1:], tud_blue, linestyle="-", alpha=0.65, label="$J_{2,r}(t)$")
        ax1b.plot(T[1:], Jh2[1:], tud_red, linestyle="-", alpha=0.65, label="$J_{2,h}(t)$")
        # Estimators for the human
        if c > 1:
            ax1b.plot(T[1:], Jhhat1[1:], tud_red, linestyle="--", alpha=0.4, label="$\hat{J}_{1,h}(t)$")
        if c_compare > 1:
            ax1b.plot(T[1:], Jhhat2[1:], tud_red, linestyle="--", alpha=0.4, label="$\hat{J}_{2,h}(t)$")

        ax1b.set_title('Cost function', **csfont)
        ax1b.set_xlabel('Time (s)', **hfont)
        ax1b.set_ylabel('Cost function value (-)', **hfont)

        ax1c.plot(T, ur1, tud_blue, linestyle="-", label="$u_{1,r}(t)$")
        ax1c.plot(T, uh1, tud_red, linestyle="-", label="$u_{1,h}(t)$")
        ax1c.plot(T, ur2, tud_blue, linestyle="-", alpha=1, label="$u_{2,r}(t)$")
        ax1c.plot(T, uh2, tud_red, linestyle="-", alpha=1, label="$u_{2,h}(t)$")
        if c > 1:
            ax1c.plot(T, uhhat1, tud_red, linestyle="--", alpha=0.5, label="$\hat{u}_{1,h}(t)$")
        if c_compare > 1:
            ax1c.plot(T, uhhat2, tud_red, linestyle="--", alpha=0.5, label="$\hat{u}_{2,h}(t)$")

        ax1c.set_title('Control action', **csfont)
        ax1c.set_xlabel('Time (s)', **hfont)
        ax1c.set_ylabel('Input force (N)', **hfont)

        ax1a.legend(prop={"size": 8}, loc='upper right')
        ax1a.set(xlim=(0, t))
        ax1b.legend(prop={"size": 8}, loc='upper right')
        ax1b.set(xlim=(0, t))
        ax1c.legend(prop={"size": 8}, loc='upper right')
        ax1c.set(xlim=(0, t))

        fig1.tight_layout(pad=1)

        if save > 0:
            location = "C:\\Users\\Lorenzo\\Repositories\\thesis\\img\\simulations\\"
            string = "comp" + dynamics_name + "_" + controller_name + "_" + controller_name + ".pdf"
            plt.savefig(string)



        fig2 = plt.figure()
        title = "Compare " + controller1 + " vs " + controller2
        fig2.suptitle(title, **csfont)

        gs2 = fig2.add_gridspec(2, 6)
        ax2a = fig2.add_subplot(gs2[0, 0:3])
        ax2b = fig2.add_subplot(gs2[0, 3:6])

        ax2f = fig2.add_subplot(gs2[1, 0:3])
        ax2g = fig2.add_subplot(gs2[1, 3:6])

        ax2a.plot(T, Lh1[:, 0], tud_red, linestyle="-", label="$L_{h,1}^1(t)$")
        ax2b.plot(T, Lh1[:, 1], tud_red, linestyle="-", label="$L_{h,2}^1(t)$")
        ax2a.plot(T, Lh2[:, 0], tud_blue, linestyle="-", label="$L_{h,1}^2(t)$")
        ax2b.plot(T, Lh2[:, 1], tud_blue, linestyle="-", label="$L_{h,2}^2(t)$")
        # ax2a.plot(T, Lr2[:, 0], tud_blue, linestyle="-", label="$L_{r,1}(t)$")
        # ax2b.plot(T, Lr2[:, 1], tud_blue, linestyle="-", label="$L_{r,2}(t)$")
        if c > 1:
            ax2a.plot(T, Lhhat1[:, 0], tud_red, linestyle="--", alpha=0.5, label="$\hat{L}_{h,1}^1(t)$")
            ax2b.plot(T, Lhhat1[:, 1], tud_red, linestyle="--", alpha=0.5, label="$\hat{L}_{h,2}^1(t)$")
        if c_compare > 1:
            ax2a.plot(T, Lhhat2[:, 0], tud_blue, linestyle="--", alpha=0.5, label="$\hat{L}_{h,1}^2(t)$")
            ax2b.plot(T, Lhhat2[:, 1], tud_blue, linestyle="--", alpha=0.5, label="$\hat{L}_{h,2}^2(t)$")

        ax2a.set_title('Human position gain', **csfont)
        ax2a.set_xlabel('Time (s)', **hfont)
        ax2a.set_ylabel('Gain value (N/m)', **hfont)
        ax2a.legend(prop={"size": 8}, loc='upper right')
        ax2a.set(xlim=(0, t))

        ax2b.set_title('Human velocity gain', **csfont)
        ax2b.set_xlabel('Time (s)', **hfont)
        ax2b.set_ylabel('Gain value (Ns/m)', **hfont)
        ax2b.legend(prop={"size": 8}, loc='upper right')
        ax2b.set(xlim=(0, t))

        ax2f.plot(T, Qh1[:, 0, 0], tud_red, label="$Q_{h,(1,1)}^1(t)$")
        if c > 1:
            ax2f.plot(T, Qhhat1[:-1, 0, 0], tud_red, linestyle="--", alpha=0.5, label="$\hat{Q}_{h,(1,1)}^1(t)$")
        if c_compare > 1:
            ax2f.plot(T, Qhhat2[:-1, 0, 0], tud_red, linestyle="-.", alpha=0.5, label="$\hat{Q}_{h,(1,1)}^1(t)$")
        ax2f.plot(T, Qr1[:, 0, 0], tud_blue, label="$Q_{r,(1,1)}^1(t)$")
        ax2g.plot(T, Qh2[:, 1, 1], tud_red, label="$Q_{h,(2,2)}^1(t)$")
        if c > 1:
            ax2g.plot(T, Qhhat1[:-1, 1, 1], tud_red, linestyle="--", alpha=0.5, label="$\hat{Q}_{h,(2,2)}^1(t)$")
        if c_compare > 1:
            ax2g.plot(T, Qhhat2[:-1, 1, 1], tud_red, linestyle="-.", alpha=0.5, label="$\hat{Q}_{h,(2,2)}^1(t)$")
        ax2g.plot(T, Qr2[:, 1, 1], tud_blue, label="$Q_{r,(2,2)}^1(t)$")



        ax2f.set_title('Position cost weight', **csfont)
        ax2f.set_xlabel('Time (s)', **hfont)
        ax2f.set_ylabel('Weight value (-)', **hfont)
        ax2f.legend(prop={"size": 8}, loc='upper right')
        ax2f.set(xlim=(0, t))

        ax2g.set_title('Velocity cost weight', **csfont)
        ax2g.set_xlabel('Time (s)', **hfont)
        ax2g.set_ylabel('Weight value (-)', **hfont)
        ax2g.legend(prop={"size": 8}, loc='upper right')
        ax2g.set(xlim=(0, t))

        fig2.tight_layout(pad=0.1)

        if save > 0:
            location = "C:\\Users\\Lorenzo\\Repositories\\thesis\\img\\simulations\\"
            string = location + dynamics_name + "_" + controller_name + "_gaincostp.pdf"
            plt.savefig(string)

        if save == 0:
            plt.show()
        elif save == 2:
            plt.show()

    def plot_sensitivity(self, inputs, inputs2, inputs3, outputs, outputs2, outputs3):
    # def plot_sensitivity(self, inputs, inputs2, inputs3, inputs4, outputs, outputs2, outputs3, outputs4):
        # Unpack stuff
        T = inputs["time_vector"]
        t = T[-1]
        c = inputs["c"]
        d = inputs["d"]
        controller = inputs["controller_type"]
        dynamics = inputs["dynamics_type"]
        controller_name = inputs["controller_type_name"]
        dynamics_name = inputs["dynamics_type_name"]
        save = inputs["save"]
        r = outputs["reference_signal"]
        var = inputs["variable"]

        # Compare states
        x1 = outputs["states"]
        x2 = outputs2["states"]
        x3 = outputs3["states"]
        # x4 = outputs4["states"]

        ur = outputs3["robot_input"]
        uh = outputs3["human_input"]
        uhhat1 = outputs["human_estimated_input"]
        uhhat2 = outputs2["human_estimated_input"]
        uhhat3 = outputs3["human_estimated_input"]
        # uhhat4 = outputs4["human_estimated_input"]

        Qh = outputs3["human_Q"]
        Qr = outputs3["robot_Q"]
        Qhhat1 = outputs["human_estimated_Q"]
        Qhhat2 = outputs2["human_estimated_Q"]
        Qhhat3 = outputs3["human_estimated_Q"]
        # Qhhat4 = outputs4["human_estimated_Q"]

        Lh1 = outputs["human_gain"]
        Lh2 = outputs2["human_gain"]
        Lh3 = outputs3["human_gain"]
        # Lh4 = outputs4["human_gain"]
        Lhhat1 = outputs["human_estimated_gain"]
        Lhhat2 = outputs2["human_estimated_gain"]
        Lhhat3 = outputs3["human_estimated_gain"]
        # Lhhat4 = outputs4["human_estimated_gain"]


        # Figure properties
        csfont = {'fontname': 'Georgia'}
        hfont = {'fontname': 'Georgia'}

        # Colors
        tud_blue = "#0066A2"
        tud_black = "#000000"
        tud_red = "#c3312f"
        tud_green = "#99D28C"
        tud_yellow = "#F1BE3E"

        # Plot stuff
        fig1 = plt.figure()
        title = controller + " for " + dynamics
        fig1.suptitle(title, **csfont)

        gs1 = fig1.add_gridspec(2,2)
        ax1a = fig1.add_subplot(gs1[0, :])
        ax1c = fig1.add_subplot(gs1[1, :])

        var_tex = "$" + var + "$"

        label1 = var_tex + " = " + str(inputs[var])
        label2 = var_tex + " = " + str(inputs2[var])
        label3 = var_tex + " = " + str(inputs3[var])
        # label4 = var_tex + " = " + str(inputs4[var])

        fig1.tight_layout(pad=1)

        if save > 0:
            location = "C:\\Users\\Lorenzo\\Repositories\\thesis\\img\\simulations\\"
            string = location + dynamics_name + "_" + controller_name + "_" + var + ".pdf"
            plt.savefig(string)



        fig2 = plt.figure()
        title = controller + " for " + dynamics
        fig2.suptitle(title, **csfont)

        gs2 = fig2.add_gridspec(2, 6)
        ax2a = fig2.add_subplot(gs2[0, 0:3])
        ax2b = fig2.add_subplot(gs2[0, 3:6])

        ax2f = fig2.add_subplot(gs2[1, 0:3])
        ax2g = fig2.add_subplot(gs2[1, 3:6])

        ax2a.plot(T, Lh1[:, 0], tud_blue, label=label1)
        ax2b.plot(T, Lh1[:, 1], tud_blue, label=label1)
        ax2a.plot(T, Lh2[:, 0], tud_green, label=label2)
        ax2b.plot(T, Lh2[:, 1], tud_green, label=label2)
        ax2a.plot(T, Lh3[:, 0], tud_red, label=label3)
        ax2b.plot(T, Lh3[:, 1], tud_red, label=label3)
        # ax2a.plot(T, Lh4[:, 0], tud_yellow, label=label4)
        # ax2b.plot(T, Lh4[:, 1], tud_yellow, label=label4)
        ax2a.plot(T, Lhhat1[:, 0], tud_blue, linestyle="--", alpha=0.5)
        ax2b.plot(T, Lhhat1[:, 1], tud_blue, linestyle="--", alpha=0.5)
        ax2a.plot(T, Lhhat2[:, 0], tud_green, linestyle="--", alpha=0.5)
        ax2b.plot(T, Lhhat2[:, 1], tud_green, linestyle="--", alpha=0.5)
        ax2a.plot(T, Lhhat3[:, 0], tud_red, linestyle="--", alpha=0.5)
        ax2b.plot(T, Lhhat3[:, 1], tud_red, linestyle="--", alpha=0.5)
        # ax2a.plot(T, Lhhat4[:, 0], tud_yellow, linestyle="--", alpha=0.5)
        # ax2b.plot(T, Lhhat4[:, 1], tud_yellow, linestyle="--", alpha=0.5)
        # ax2a.plot(T, Lhhat1[:, 0], tud_blue, linestyle="--", alpha=0.5, label="Estimated gain")
        # ax2b.plot(T, Lhhat1[:, 1], tud_blue, linestyle="--", alpha=0.5, label="Estimated gain")
        # ax2a.plot(T, Lhhat2[:, 0], tud_green, linestyle="--", alpha=0.5, label="Estimated gain")
        # ax2b.plot(T, Lhhat2[:, 1], tud_green, linestyle="--", alpha=0.5, label="Estimated gain")
        # ax2a.plot(T, Lhhat3[:, 0], tud_red, linestyle="--", alpha=0.5, label="Estimated gain")
        # ax2b.plot(T, Lhhat3[:, 1], tud_red, linestyle="--", alpha=0.5, label="Estimated gain")
        # ax2a.plot(T, Lhhat4[:, 0], tud_yellow, linestyle="--", alpha=0.5, label="Estimated gain")
        # ax2b.plot(T, Lhhat4[:, 1], tud_yellow, linestyle="--", alpha=0.5, label="Estimated gain")


        ax2a.set_title('Position gain', **csfont)
        ax2a.set_xlabel('Time (s)', **hfont)
        ax2a.set_ylabel('Gain value (N/m)', **hfont)
        h1, l1 = ax2a.get_legend_handles_labels()
        h1.append(Line2D([0], [0], color='k', alpha=0.5, linestyle="--", lw=1, label='Line'))
        l1.append("Estimated gain")
        ax2a.legend(h1, l1, prop={"size": 8}, loc='lower right')
        ax2a.set(xlim=(0, t))

        ax2b.set_title('Velocity gain', **csfont)
        ax2b.set_xlabel('Time (s)', **hfont)
        ax2b.set_ylabel('Gain value (Ns/m)', **hfont)
        ax2b.legend(h1, l1, prop={"size": 8}, loc='lower right')
        ax2b.set(xlim=(0, t))

        ax2f.plot(T, Qh[:, 0, 0], tud_black, lw=0.5, label="True value")
        ax2f.plot(T, Qhhat1[:-1, 0, 0], tud_blue, linestyle="--", alpha=0.5, label=label1)
        ax2f.plot(T, Qhhat2[:-1, 0, 0], tud_green, linestyle="--", alpha=0.5, label=label2)
        ax2f.plot(T, Qhhat3[:-1, 0, 0], tud_red, linestyle="--", alpha=0.5, label=label3)
        # ax2f.plot(T, Qhhat4[:-1, 0, 0], tud_yellow, linestyle="--", alpha=0.5, label=label4)

        ax2g.plot(T, Qh[:, 1, 1], tud_black, lw=0.5, label="True value")
        ax2g.plot(T, Qhhat1[:-1, 1, 1], tud_blue, linestyle="--", alpha=0.5, label=label1)
        ax2g.plot(T, Qhhat2[:-1, 1, 1], tud_green, linestyle="--", alpha=0.5, label=label2)
        ax2g.plot(T, Qhhat3[:-1, 1, 1], tud_red, linestyle="--", alpha=0.5, label=label3)
        # ax2g.plot(T, Qhhat4[:-1, 1, 1], tud_yellow, linestyle="--", alpha=0.5, label=label4)

        ax2f.set_title('Position cost weight', **csfont)
        ax2f.set_xlabel('Time (s)', **hfont)
        ax2f.set_ylabel('Weight value (-)', **hfont)
        ax2f.legend(prop={"size": 8}, loc='lower right')
        ax2f.set(xlim=(0, t))

        ax2g.set_title('Velocity cost weight', **csfont)
        ax2g.set_xlabel('Time (s)', **hfont)
        ax2g.set_ylabel('Weight value (-)', **hfont)
        ax2g.legend(prop={"size": 8}, loc='lower right')
        ax2g.set(xlim=(0, t))

        fig2.tight_layout(pad=0.1)

        if save > 0:
            location = "C:\\Users\\Lorenzo\\Repositories\\thesis\\img\\simulations\\"
            string = location + dynamics_name + "_" + controller_name + "_" + var + "_gaincostp.pdf"
            plt.savefig(string)

        if save == 0:
            plt.show()
        elif save == 2:
            plt.show()