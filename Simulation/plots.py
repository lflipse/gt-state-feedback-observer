import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.backends.backend_pdf import PdfPages

class PlotStuff:
    def __init__(self):
        print("Plotting stuff")

    def plot_stuff(self, inputs1, outputs1, inputs2, outputs2, inputs3, outputs3):
        # Figure properties
        csfont = {'fontname': 'Georgia'}
        hfont = {'fontname': 'Georgia'}

        # Colors
        tud_blue = "#0066A2"
        tud_blue2 = "#61A4B4"
        tud_blue3 = "#82D7C6"
        tud_black = "#000000"
        tud_grey = "#808080"
        tud_red = "#c3312f"
        tud_orange = "#EB7245"
        tud_yellow = "#F1BE3E"
        tud_green = "#00A390"

        # List of figures
        f1 = plt.figure(1)
        f2 = plt.figure(2)
        f3 = plt.figure(3)
        f4 = plt.figure(4)
        f5 = plt.figure(5)
        f6 = plt.figure(6)
        f7 = plt.figure(7)

        robot_colors = [tud_blue, tud_blue2, tud_blue3]
        human_colors = [tud_blue, tud_blue2, tud_blue3]

        list_inputs = [inputs1, inputs2, inputs3]
        list_outputs = [outputs1, outputs2, outputs3]

        if inputs3 != None:
            print("we're doing sensitivity analysis")
            m = 3
            robot_colors = [tud_blue, tud_blue2, tud_blue3]
            human_colors = [tud_blue, tud_blue2, tud_blue3]
        elif inputs2 != None:
            print("we're comparing")
            m = 2
            robot_colors = [tud_blue, tud_blue2]
            human_colors = [tud_red, tud_green]
        else:
            print("we're checking performance")
            m = 1
            human_colors = [tud_red]
            robot_colors = [tud_blue]

        for i in range(m):
            inputs = list_inputs[i]
            outputs = list_outputs[i]
            print(i)

            # Unpack stuff
            T = inputs["time_vector"]
            t = T[-1]
            c = inputs["c"]
            controller = inputs["controller_type"]
            dynamics = inputs["dynamics_type"]
            controller_name = inputs["controller_type_name"]
            dynamics_name = inputs["dynamics_type_name"]
            r = outputs["reference_signal"]
            x = outputs["states"]
            e = outputs["error_states"]
            Jr = outputs["robot_costs"]
            Jh = outputs["human_costs"]
            Jt = Jr + Jh
            ur = outputs["robot_input"]
            uh = outputs["human_input"]
            ut = ur + uh
            Lh = outputs["human_gain"]
            Lr = outputs["robot_gain"]
            Qh = outputs["human_Q"]
            Qr = outputs["robot_Q"]

            print(c)

            if c > 1:
                Jhhat = outputs["human_estimated_costs"]
                uhhat = outputs["human_estimated_input"]
                Qhhat = outputs["human_estimated_Q"]
                Lhhat = outputs["human_estimated_gain"]

            # Plot stuff
            plt.figure(f1.number)
            if i == 0:
                plt.plot(T, r[:, 0], tud_black, linewidth=2.5, linestyle="-", alpha=0.7, label="Reference $r(t)$")
            plt.plot(T, x[:-1, 0], robot_colors[i], linestyle="-", linewidth=2.5, label="State $x(t)$")
            plt.title('Position', **csfont)
            plt.xlabel('Time (s)', **hfont)
            plt.ylabel('Position (m)', **hfont)
            plt.legend(prop={"size": 8}, loc='upper right')
            plt.legend(prop={"size": 8}, loc='upper right')
            plt.xlim(0, t)
            plt.tight_layout(pad=1)

            # Cost functions
            plt.figure(f2.number)
            plt.plot(T[1:], Jr[1:], robot_colors[i], linestyle="-", linewidth=2.5, label="Robot cost $J_r(t)$")
            if c > 1:
                if i == 0:
                    plt.plot(T[1:], Jh[1:], human_colors[i], linestyle="-", linewidth=2.5, alpha=0.5, label="Real human cost $J_h(t)$")
                plt.plot(T[1:], Jhhat[1:], human_colors[i], linestyle="--", alpha=1, label="Estimated human cost $\hat{J}_h(t)$")
            else:
                if i == 0:
                    plt.plot(T[1:], Jh[1:], human_colors[i], linestyle="-", linewidth=2.5, alpha=1, label="Human cost $J_h(t)$")
            plt.title('Cost function', **csfont)
            plt.xlabel('Time (s)', **hfont)
            plt.ylabel('Cost function value (-)', **hfont)
            plt.legend(prop={"size": 8}, loc='upper right')
            plt.xlim(0, t)
            plt.tight_layout(pad=1)

            # Inputs
            plt.figure(f3.number)
            plt.plot(T, ur, robot_colors[i], linestyle="-", linewidth=2.5, label="Robot input $u_r(t)$")
            if c > 1:
                plt.plot(T, uhhat, human_colors[i], linestyle="--", linewidth=2.5, label="Estimated human input $\hat{u}_h(t)$")
                if i == 0:
                    plt.plot(T, uh, human_colors[i], linestyle="-", linewidth=2.5, alpha=0.5, label="Real human input $u_h(t)$")
            else:
                if i == 0:
                    plt.plot(T, uh, human_colors[i], linestyle="-", linewidth=2.5, label="Human input $u_h(t)$")
            plt.plot(T, ut, tud_green, alpha=1, linestyle="-", linewidth=2.5, label="Total input $u(t)$")
            plt.title('Control action', **csfont)
            plt.xlabel('Time (s)', **hfont)
            plt.ylabel('Input force (N)', **hfont)
            plt.legend(prop={"size": 8}, loc='upper right')
            plt.xlim(0, t)
            plt.tight_layout(pad=1)

            plt.figure(f4.number)
            if i == 0:
                plt.plot(T, Lh[:, 0], human_colors[i], linewidth=2.5, alpha=0.7, label="Real human gain $L_{h,1}(t)$")
            try:
                plt.plot(T, Lhhat[:, 0], human_colors[i], linewidth=2.5, linestyle="--", label="Estimated human gain $\hat{L}_{h,1}(t)$")
            except:
                a=1
            if m < 2:
                plt.plot(T, Lr[:, 0], robot_colors[i], linewidth=2.5, label="Robot gain $L_{r,1}(t)$")
            plt.title('Position gain', **csfont)
            plt.xlabel('Time (s)', **hfont)
            plt.ylabel('Gain value (N/m)', **hfont)
            plt.legend(prop={"size": 8}, loc='upper right')
            plt.xlim(0, t)

            plt.figure(f5.number)
            if i == 0:
                plt.plot(T, Lh[:, 1], human_colors[i], linewidth=2.5, alpha=0.7, label="Real human gain $L_{h,2}(t)$")
            try:
                plt.plot(T, Lhhat[:, 1], human_colors[i], linewidth=2.5, linestyle="--", label="Estimated human gain $\hat{L}_{h,2}(t)$")
            except:
                a=1
            if m < 2:
                plt.plot(T, Lr[:, 1], robot_colors[i], linewidth=2.5, label="Robot gain $L_{r,2}(t)$")
            plt.title('Velocity gain', **csfont)
            plt.xlabel('Time (s)', **hfont)
            plt.ylabel('Gain value (Ns/m)', **hfont)
            plt.legend(prop={"size": 8}, loc='upper right')
            plt.xlim(0, t)
            plt.tight_layout(pad=1)

            plt.figure(f6.number)
            if i == 0:
                plt.plot(T, Qh[:, 0, 0], human_colors[i], linewidth=2.5, alpha=0.7, label="Real human weight $Q_{h,(1,1)}(t)$")
            try:
                plt.plot(T, Qhhat[:-1, 0, 0], human_colors[i], linewidth=2.5, linestyle="--",  label="Estimated human weight$\hat{Q}_{h,(1,1)}(t)$")
            except:
                a=1
            if m < 2:
                plt.plot(T, Qr[:, 0, 0], robot_colors[i], linewidth=2.5, label="Robot weight $Q_{r,(1,1)}(t)$")
            plt.title('Position cost weight', **csfont)
            plt.xlabel('Time (s)', **hfont)
            plt.ylabel('Weight value (-)', **hfont)
            plt.legend(prop={"size": 8}, loc='upper right')
            plt.xlim(0, t)
            plt.tight_layout(pad=1)

            plt.figure(f7.number)
            if i == 0:
                plt.plot(T, Qh[:, 1, 1], human_colors[i], linewidth=2.5, alpha=0.7, label="Real human weight $Q_{h,(2,2)}(t)$")
            try:
                plt.plot(T, Qhhat[:-1, 1, 1], human_colors[i], linewidth=2.5, linestyle="--", label="Estimated human weight $\hat{Q}_{h,(2,2)}(t)$")
            except:
                a=1
            if m < 2:
                plt.plot(T, Qr[:, 1, 1], robot_colors[i], linewidth=2.5, label="Robot weight $Q_{r,(2,2)}(t)$")
            plt.title('Velocity cost weight', **csfont)
            plt.xlabel('Time (s)', **hfont)
            plt.ylabel('Weight value (-)', **hfont)
            plt.legend(prop={"size": 8}, loc='upper right')
            plt.xlim(0, t)
            plt.tight_layout(pad=1)

        if m == 1:
            self.save_all_figures('performance.pdf')
        elif m == 2:
            self.save_all_figures('comparison.pdf')
        else:
            self.save_all_figures('sensitivity.pdf')

        plt.show()

    def save_all_figures(self, title):
        pp = PdfPages(title)
        figs = None
        if figs is None:
            figs = [plt.figure(n) for n in plt.get_fignums()]
        for fig in figs:
            fig.savefig(pp, format='pdf')
        pp.close()