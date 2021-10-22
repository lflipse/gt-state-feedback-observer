import scipy.linalg as cp
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


class Strategy():
    def __init__(self):
        self.csfont = {'fontname': 'Georgia', 'size': 24}
        self.hfont = {'fontname': 'Georgia', 'size': 22}
        self.linewidth = 4

        # Colors
        tud_black = "#000000"
        self.tud_blue = "#0066A2"
        self.tud_red = "#c3312f"
        self.tud_green = "#00A390"
        self.tud_yellow = "#F1BE3E"
        self.tud_orange = "#EB7245"
        self.tud_lightblue = "#00B7D3"
    
    def do(self):
        outputs = self.run()
        self.plot_strategies(outputs)
    
    def run(self):
        # Take 100 steps of possible gains and costs
        n = 200  # steps
        m = 3  # strategies

        # Compute possible costs from 0 to 40
        q1h = np.linspace(0, 140, num=n)
        Qh = np.zeros((n, 2, 2))
        Qr1 = np.zeros((n, 2, 2))
        Qr2 = np.zeros((n, 2, 2))
        Lh1 = np.zeros((n, 2))
        Lr1 = np.zeros((n, 2))
        Lh2 = np.zeros((n, 2))
        Lr2 = np.zeros((n, 2))
        C1 = np.zeros(n)
        C2 = np.zeros(n)
        Qh[:, 0, 0] = q1h

        # C = np.array([[25.0, 0], [0, 0.1]])
        C = np.array([[80.0, 0], [0, 0.1]])

        # Dynamics
        Jw = 0.04914830792783059
        Bw = 0.3  # Max = 0.5
        Kw = 0.0  # Max = 2.5
        A = np.array([[0, 1], [- Kw / Jw, - Bw / Jw]])
        B = np.array([[0], [1 / Jw]])

        for i in range(n):
            # New laws
            # alpha = np.array([[0.1, 0], [0, 1.0]])
            # gamma = np.array([[1.5, 0], [0, -1.0]])
            # zeta = np.array([[1.0, 0], [0, 1.0]])

            # Old laws
            alpha = np.array([[0.02, 0], [0, 1.0]])
            gamma = np.array([[2.5, 0], [0, -1.0]])
            zeta = np.array([[2.0, 0], [0, 1.0]])

            if Qh[i, 0, 0] < 1 / zeta[0, 0] * C[0, 0]:
                Qr1[i, :, :] = C - np.matmul(zeta, Qh[i, :, :])
            Qr2[i, :, :] = np.matmul(alpha, C) + np.matmul(gamma, Qh[i, :, :])


            C1[i], Lr1[i, :], Lh1[i, :] = self.compute_gains(Qr1[i, :, :], Qh[i, :, :], A, B)
            C2[i], Lr2[i, :], Lh2[i, :] = self.compute_gains(Qr2[i, :, :], Qh[i, :, :], A, B)

    
        outputs = {
            "cost_human": Qh,
            "cost_robot_pos": Qr1,
            "gain_robot_pos": Lr1,
            "gain_human_pos": Lh1,
            "cost_robot_neg": Qr2,
            "gain_robot_neg": Lr2,
            "gain_human_neg": Lh2,
            "auth_pos": C1,
            "auth_neg": C2,
        }
        return outputs
    
    def plot_strategies(self, inputs):
        Qh = inputs["cost_human"]
        Qr1 = inputs["cost_robot_pos"]
        Lr1 = inputs["gain_robot_pos"]
        Lh1 = inputs["gain_human_pos"]
        Qr2 = inputs["cost_robot_neg"]
        Lr2 = inputs["gain_robot_neg"]
        Lh2 = inputs["gain_human_neg"]
        C1 = inputs["auth_pos"]
        C2 = inputs["auth_neg"]
        
        plt.figure()
        plt.plot(Qh[:, 0, 0], Qr1[:, 0, 0], linewidth=self.linewidth)
        plt.title("Negative reinforcement", self.csfont)
        plt.xlabel('Human cost weight (-)', self.hfont)
        plt.ylabel('Robot cost weight (-)', self.hfont)
        # plt.legend(prop={"size": 14}, loc='upper right')
        plt.xlim(Qh[0, 0, 0], Qh[-1, 0, 0])
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(Qh[:, 0, 0], Qr2[:, 0, 0], linewidth=self.linewidth)
        plt.title("No reinforcement", self.csfont)
        plt.xlabel('Human cost weight (-)', self.hfont)
        plt.ylabel('Robot cost weight (-)', self.hfont)
        # plt.legend(prop={"size": 14}, loc='upper right')
        plt.xlim(Qh[0, 0, 0], Qh[-1, 0, 0])
        plt.tight_layout(pad=1)


        labels = ['Human Gain', 'Robot Gain']
        colors = [self.tud_red, self.tud_blue]

        plt.figure()
        plt.stackplot(Lh1[:, 0], Lh1[:, 0], Lr1[:, 0], labels=labels, colors=colors)
        plt.title("Negative reinforcement", self.csfont)
        plt.xlabel('Human gain (Nm)', self.hfont)
        plt.ylabel('Robot gain (Nm)', self.hfont)
        plt.legend(prop={"size": 14}, loc='upper right')
        plt.xlim(0, 5)
        plt.ylim(0, 15)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.stackplot(Lh2[:, 0], Lh2[:, 0], Lr2[:, 0], labels=labels, colors=colors)
        plt.title("Positive reinforcement", self.csfont)
        plt.xlabel('Human gain (Nm)', self.hfont)
        plt.ylabel('Robot gain (Nm)', self.hfont)
        plt.legend(prop={"size": 14}, loc='upper right')
        plt.xlim(0, 5)
        plt.ylim(0, 15)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(Qh[:, 0, 0], Qr1[:, 0, 0], self.tud_blue, linewidth=self.linewidth, label="Negative Reinforcement")
        plt.plot(Qh[:, 0, 0], Qr2[:, 0, 0], self.tud_red, linewidth=self.linewidth, label="Positive Reinforcement")
        plt.title("Interaction strategies: Costs", self.csfont)
        plt.xlabel('Human cost weight (-)', self.hfont)
        plt.ylabel('Robot cost weight (-)', self.hfont)
        plt.legend(prop={"size": 14}, loc='upper right')
        xmin = min(Qh[:, 0, 0])
        xmax = max(Qh[:, 0, 0])
        ymin = min(min(Qr1[:, 0, 0]), min(Qr2[:, 0, 0])) - 0.1
        ymax = max(max(Qr1[:, 0, 0]), max(Qr2[:, 0, 0])) + 0.1
        # plt.xlim(xmin, xmax)
        # plt.ylim(ymin, ymax)
        plt.xlim(0, 40)
        plt.ylim(0, 65)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(Lh1[:, 0], Lr1[:, 0], self.tud_blue, linewidth=self.linewidth, label="Negative Reinforcement")
        plt.plot(Lh2[:, 0], Lr2[:, 0], self.tud_red, linewidth=self.linewidth, label="Positive Reinforcement")
        plt.title("Interaction strategies: Gains", self.csfont)
        plt.xlabel('Human gain (Nm)', self.hfont)
        plt.ylabel('Robot gain (Nm)', self.hfont)
        plt.legend(prop={"size": 14}, loc='upper right')
        xmin = min(min(Lh1[:, 0]), min(Lh2[:, 0]))
        xmax = min(max(Lh1[:, 0]), max(Lh2[:, 0]))
        ymin = min(min(Lr1[:, 0]), min(Lr2[:, 0])) - 0.1
        ymax = max(max(Lr1[:, 0]), max(Lr2[:, 0])) + 0.1
        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(Lh1[:, 0], C1, self.tud_blue, linewidth=self.linewidth, label="Negative Reinforcement")
        plt.plot(Lh2[:, 0], C2, self.tud_red, linewidth=self.linewidth, label="Positive Reinforcement")

        plt.title("Interaction strategies: Authority", self.csfont)
        plt.xlabel('Human gain (Nm)', self.hfont)
        plt.ylabel('Robot gain (Nm)', self.hfont)
        plt.legend(prop={"size": 14}, loc='upper right')
        xmin = min(min(Lh1[:, 0]), min(Lh2[:, 0]))
        xmax = min(max(Lh1[:, 0]), max(Lh2[:, 0]))
        ymin = min(min(C1), min(C2)) - 0.1
        ymax = max(max(C1), max(C2)) + 0.1
        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)
        plt.tight_layout(pad=1)

        self.save_all_figures()

        plt.show()

    def save_all_figures(self):
        pp = PdfPages('../Experiment/strategy.pdf')
        figs = None
        if figs is None:
            figs = [plt.figure(n) for n in plt.get_fignums()]
        for fig in figs:
            fig.savefig(pp, format='pdf')
        pp.close()

    def compute_gains(self, Qr, Qh, A, B):
        # Iterative procedure for calculating gains
        Lh = np.matmul(B.transpose(), cp.solve_continuous_are(A, B, Qh, 1))
        Lr = np.matmul(B.transpose(), cp.solve_continuous_are(A, B, Qr, 1))
        for i in range(10):
            Lh = np.matmul(B.transpose(), cp.solve_continuous_are(A - B * Lr, B, Qh, 1))
            Lr = np.matmul(B.transpose(), cp.solve_continuous_are(A - B * Lh, B, Qr, 1))
        C = (Lr[0, 0] - Lh[0, 0]) / (Lr[0, 0] + Lh[0, 0])
        return C, Lr, Lh

# strategy = Strategy()
# strategy.do()