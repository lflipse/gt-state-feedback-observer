import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.backends.backend_pdf import PdfPages

class PlotStuff:
    def __init__(self):
        print("Plotting stuff")

    def plot_stuff(self, inputs1, outputs1, inputs2, outputs2, inputs3, outputs3, save, v, size="small"):
        # Figure properties
        # Colors
        tud_blue = "#0066A2"
        tud_blue2 = "#61A4B4"
        tud_blue3 = "#007188"
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
        f8 = plt.figure(8)
        
        if size == "small":
            lw = 2.5
            title_size = 14
            label_size = 12
            legend_size = 8
        else:
            lw = 4
            title_size = 24
            label_size = 22
            legend_size = 14

        csfont = {'fontname': 'Georgia', 'size': title_size}
        hfont = {'fontname': 'Georgia', 'size': label_size}
        legend_font = {'family': 'Georgia', 'size': legend_size}

        list_inputs = [inputs1, inputs2, inputs3]
        list_outputs = [outputs1, outputs2, outputs3]
        variable = "kappa"
        var_label = None
        if inputs3 != None:
            print("we're doing sensitivity analysis")
            m = 3

            robot_colors = [tud_blue, tud_blue2, tud_blue3]
            human_colors = [tud_red, tud_blue3, tud_green]

            if int(v) == 0:
                variable = "kappa"
                var_label = "$\kappa$"
            elif int(v) == 1:
                variable = "K"
                var_label = "$K$"
            elif int(v) == 2:
                variable = "gain_estimation_bias"
                var_label = "$\~L_{h,2}$"
            else:
                exit("erreur")

        elif inputs2 != None:
            print("we're comparing")
            m = 2
            robot_colors = [tud_blue3, tud_orange]
            human_colors = robot_colors
            # robot_colors = [tud_blue, tud_orange]
            # human_colors = [tud_red, tud_green]
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
            controller = inputs["controller_type"]
            dynamics = inputs["dynamics_type"]
            controller_name = inputs["controller_type_name"]
            dynamics_name = inputs["dynamics_type_name"]

            r = outputs["reference_signal"]
            x = outputs["states"]
            e = outputs["error_states"]
            Jr = outputs["robot_costs"]
            Jh = outputs["human_costs"]
            try:
                Jt = Jr + Jh
            except:
                Jt = Jr
            ur = outputs["robot_input"]
            uh = outputs["human_input"]
            try:
                ut = ur + uh
            except:
                ut = ur
            Lh = outputs["human_gain"]
            Lr = outputs["robot_gain"]
            try:
                Lt = Lr + Lh
            except:
                Lt = Lr
            Qh = outputs["human_Q"]
            Qr = outputs["robot_Q"]

            print(controller)

            if controller != "Optimal Control" and controller != "Differential Game":
                estimates = True
            else:
                estimates = False

            if estimates:
                print("using estimates")
                Jhhat = outputs["human_estimated_costs"]
                uhhat = outputs["human_estimated_input"]
                Qhhat = outputs["human_estimated_Q"]
                Lhhat = outputs["human_estimated_gain"]
                xhat = outputs["estimated_states"]
            else:
                Jhhat = None
                uhhat = None
                Qhhat = None
                Lhhat = None
                xhat = None

            value = inputs[variable]
            if v == 1:
                # print(value)
                value = int(value[0, 0])
            if v == 2:
                print(value)
                try:
                    value = value[0, 1]
                except:
                    value = value[1]

            # Plot stuff
            # Position
            if m == 1:
                label = "State $x(t)$"
            elif m == 2:
                label = controller

            plt.figure(f1.number)
            if i == 0:
                plt.plot(T, r[:, 0], tud_black, linewidth=lw, linestyle="-", alpha=0.7, label="Reference $r(t)$")
            try:
                plt.plot(T, xhat[:-1, 0], robot_colors[i], linestyle="--", linewidth=lw,
                         label=self.create_labels(value, var_label, "state $\hat{x}(t)$"))
                plt.plot(T, x[:-1, 0], robot_colors[i], linestyle="-", alpha=0.7, linewidth=lw, label=label)
            except:
                plt.plot(T, x[:-1, 0], robot_colors[i], linestyle="-", linewidth=lw, label=label)
            plt.title('Position', **csfont)
            plt.xlabel('Time ($s$)', **hfont)
            plt.ylabel('Position ($m$)', **hfont)
            plt.legend(prop=legend_font, loc='upper right')
            plt.xlim(0, t)
            plt.tight_layout(pad=1)

            # Velocity

            if m == 2:
                label = controller
            else:
                label = "State $\dot{x}(t)$"

            plt.figure(f8.number)
            if i == 0:
                plt.plot(T, r[:, 1], tud_black, linewidth=lw, linestyle="-", alpha=0.7, label="Reference $\dot{r}(t)$")
            try:
                plt.plot(T, xhat[:-1, 1], robot_colors[i], linestyle="--", linewidth=lw,
                         label=self.create_labels(value, var_label, "state $\dot{\hat{x}}(t)$"))
                plt.plot(T, x[:-1, 1], robot_colors[i], linestyle="-", alpha=0.7, linewidth=lw, label=label)
            except:
                plt.plot(T, x[:-1, 1], robot_colors[i], linestyle="-", linewidth=lw, label=label)
            plt.title('Velocity', **csfont)
            plt.xlabel('Time ($s$)', **hfont)
            plt.ylabel('Velocity ($m/s$)', **hfont)
            plt.legend(prop=legend_font, loc='upper right')
            plt.xlim(0, t)
            plt.tight_layout(pad=1)

            # Cost functions

            plt.figure(f2.number)
            if m != 2:
                plt.plot(T[1:], Jr[1:], robot_colors[i], linestyle="-", linewidth=lw, label="Robot cost $J_r(t)$")
                if estimates:
                    if i == 0:
                        plt.plot(T[1:], Jh[1:], human_colors[i], linestyle="-", linewidth=lw, alpha=0.7, label="Real human cost $J_h(t)$")
                    plt.plot(T[1:], Jhhat[1:], human_colors[i], linestyle="--", linewidth=lw, alpha=1,
                             label=self.create_labels(value, var_label, "human cost $\hat{J}_h(t)$"))
                else:
                    if i == 0:
                        plt.plot(T[1:], Jh[1:], human_colors[i], linestyle="-", linewidth=lw, alpha=1, label="Human cost $J_h(t)$")
            else:
                plt.plot(T[1:], Jt[1:], robot_colors[i], linestyle="-", linewidth=lw, label=controller)
            plt.title('Cost function', **csfont)
            plt.xlabel('Time ($s$)', **hfont)
            plt.ylabel('Cost function value (-)', **hfont)
            plt.legend(prop=legend_font, loc='best')
            plt.xlim(0, t)
            plt.tight_layout(pad=1)

            # Inputs

            plt.figure(f3.number)
            # label_text =
            if m != 2:
                plt.plot(T, ur, robot_colors[i], linestyle="-", linewidth=lw, label="Robot input $u_r(t)$")
                if estimates:
                    if i == 0:
                        plt.plot(T, uh, human_colors[i], linestyle="-", linewidth=lw, alpha=0.7, label="Real human input $u_h(t)$")
                    plt.plot(T, uhhat, human_colors[i], linestyle="--", linewidth=lw,
                             label=self.create_labels(value, var_label, "human input $\hat{u}_h(t)$"))
                else:
                    if i == 0:
                        plt.plot(T, uh, human_colors[i], linestyle="-", linewidth=lw, label="Human input $u_h(t)$")
            else:
                plt.plot(T, ut, robot_colors[i], linestyle="-", linewidth=lw, label=controller)
            plt.title('Control action', **csfont)
            plt.xlabel('Time ($s$)', **hfont)
            plt.ylabel('Input force ($N$)', **hfont)
            plt.legend(prop=legend_font, loc='best')
            plt.xlim(0, t)
            plt.tight_layout(pad=1)

            plt.figure(f4.number)
            if m != 2:
                if i == 0:
                    plt.plot(T, Lh[:, 0], human_colors[i], linewidth=lw, alpha=0.7, label="Real human gain $L_{h,1}(t)$")
                try:
                    plt.plot(T, Lhhat[:-1, 0], human_colors[i], linewidth=lw, linestyle="--",
                             label=self.create_labels(value, var_label, "human gain $\hat{L}_{h,1}(t)$"))
                except:
                    plt.plot(T, Lhhat[:, 0], human_colors[i], linewidth=lw, linestyle="--",
                             label=self.create_labels(value, var_label, "human gain $\hat{L}_{h,1}(t)$"))
                if m < 2:
                    plt.plot(T, Lr[:, 0], robot_colors[i], linewidth=lw, label="Robot gain $L_{r,1}(t)$")
            else:
                if i == 0:
                    plt.plot(T, Lh[:, 0], human_colors[i], linewidth=lw, alpha=0.7, label="Real human gain")
                try:
                    plt.plot(T, Lhhat[:-1, 0], human_colors[i], linewidth=lw, linestyle="--",
                             label=label)
                except:
                    plt.plot(T, Lhhat[:, 0], human_colors[i], linewidth=lw, linestyle="--",
                             label=label)
            plt.title('Position gain', **csfont)
            plt.xlabel('Time ($s$)', **hfont)
            plt.ylabel('Gain value ($N/m$)', **hfont)
            plt.legend(prop=legend_font, loc='best')
            plt.xlim(0, t)
            plt.tight_layout(pad=1)

            plt.figure(f5.number)
            if m != 2:
                if i == 0:
                    plt.plot(T, Lh[:, 1], human_colors[i], linewidth=lw, alpha=0.7, label="Real human gain $L_{h,2}(t)$")
                try:
                    plt.plot(T, Lhhat[:-1, 1], human_colors[i], linewidth=lw, linestyle="--",
                             label=self.create_labels(value, var_label, "human gain $\hat{L}_{h,2}(t)$"))
                except:
                    plt.plot(T, Lhhat[:, 1], human_colors[i], linewidth=lw, linestyle="--",
                             label=self.create_labels(value, var_label, "human gain $\hat{L}_{h,2}(t)$"))
                if m < 2:
                    plt.plot(T, Lr[:, 1], robot_colors[i], linewidth=lw, label="Robot gain $L_{r,2}(t)$")
            else:
                if i == 0:
                    plt.plot(T, Lh[:, 1], human_colors[i], linewidth=lw, alpha=0.7,
                             label="Real human gain")
                try:
                    plt.plot(T, Lhhat[:-1, 1], human_colors[i], linewidth=lw, linestyle="--",
                             label=label)
                except:
                    plt.plot(T, Lhhat[:, 1], human_colors[i], linewidth=lw, linestyle="--",
                             label=label)
            plt.title('Velocity gain', **csfont)
            plt.xlabel('Time ($s$)', **hfont)
            plt.ylabel('Gain value ($Ns/m$)', **hfont)
            plt.legend(prop=legend_font, loc='best')
            plt.xlim(0, t)
            plt.tight_layout(pad=1)

            plt.figure(f6.number)
            if i == 0:
                plt.plot(T, Qh[:, 0, 0], human_colors[i], linewidth=lw, alpha=0.7, label="Real human weight")
            try:
                if m != 2:
                    plt.plot(T, Qhhat[:-1, 0, 0], human_colors[i], linewidth=lw, linestyle="--",
                         label=self.create_labels(value, var_label, "human weight"))
                else:
                    plt.plot(T, Qhhat[:-1, 0, 0], human_colors[i], linewidth=lw, linestyle="--",
                             label=label)
            except:
                a=1
            if m < 2:
                plt.plot(T, Qr[:, 0, 0], robot_colors[i], linewidth=lw, label="Robot weight")
            plt.title('Position cost weight', **csfont)
            plt.xlabel('Time ($s$)', **hfont)
            plt.ylabel('Weight value (-)', **hfont)
            plt.legend(prop=legend_font, loc='best')
            plt.xlim(0, t)
            plt.tight_layout(pad=1)

            plt.figure(f7.number)
            if i == 0:
                plt.plot(T, Qh[:, 1, 1], human_colors[i], linewidth=lw, alpha=0.7, label="Real human weight")
            try:
                if m != 2:
                    plt.plot(T, Qhhat[:-1, 1, 1], human_colors[i], linewidth=lw, linestyle="--",
                         label=self.create_labels(value, var_label, "human weight"))
                else:
                    plt.plot(T, Qhhat[:-1, 1, 1], human_colors[i], linewidth=lw, linestyle="--",
                             label=label)
            except:
                a=1
            if m < 2:
                plt.plot(T, Qr[:, 1, 1], robot_colors[i], linewidth=lw, label="Robot weight")
            plt.title('Velocity cost weight', **csfont)
            plt.xlabel('Time ($s$)', **hfont)
            plt.ylabel('Weight value (-)', **hfont)
            plt.legend(prop=legend_font, loc='best')
            plt.xlim(0, t)
            plt.tight_layout(pad=1)

        # Change to your desired location
        location = "..\\..\\thesis\\img\\simulations\\"

        try:
            if save:
                if m == 1:
                    name = location + size + '_performance_li.pdf'
                elif m == 2:
                    name = location + size + '_comparison.pdf'
                else:
                    name = location + size + '_sensitivity_' + variable + '.pdf'
                self.save_all_figures(name)
        except:
            exit("location not found, change in line 245")

        plt.show()

    def save_all_figures(self, title):
        pp = PdfPages(title)
        figs = None
        if figs is None:
            figs = [plt.figure(n) for n in plt.get_fignums()]
        for fig in figs:
            fig.savefig(pp, format='pdf')
        pp.close()

    def create_labels(self, val, name, signal):
        if name != None:
            if (abs(val) > 1000 or abs(val) < 0.005) and val!= 0:
                str_val = "{:.1e}".format(val)
            else:
                str_val = str(round(val, 2))

            label = "Estimate with " + name + " = " + str_val
        else:
            label = "Estimated " + signal
        return label