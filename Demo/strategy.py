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
        L1h = np.linspace(-2, 4, num=n)


        Qh1 = np.zeros((n, 2, 2))
        Qh2 = np.zeros((n, 2, 2))
        Qr1 = np.zeros((n, 2, 2))
        Qr2 = np.zeros((n, 2, 2))
        Lh = np.zeros((n, 2))
        Lr1 = np.zeros((n, 2))
        Lr2 = np.zeros((n, 2))
        C1 = np.zeros(n)
        C2 = np.zeros(n)
        Lh[:, 0] = L1h

        C = np.array([[10.0, 0], [0, 0.1]])

        # old
        # C = np.array([[12.5, 0], [0, 0.1]])

        # Dynamics
        Jw = 0.04914830792783059
        Bw = 0.3  # Max = 0.5
        Kw = 0.0  # Max = 2.5
        A = np.array([[0, 1], [- Kw / Jw, - Bw / Jw]])
        B = np.array([[0], [1 / Jw]])
        beta = B[1, 0]
        alpha_1 = A[1, 0]
        alpha_2 = A[1, 1]

        for i in range(n):
            # Old
            # alpha = np.array([[0.1, 0], [0, 1.0]])
            # gamma = np.array([[1.0, 0], [0, -1.0]])
            # zeta = np.array([[1.0, 0], [0, 1.0]])

            alpha = np.array([[0.02, 0], [0, 1.0]])
            gamma = np.array([[1.0, 0], [0, 0.0]])
            zeta = np.array([[1.0, 0], [0, 0.0]])

            for j in range(20):
                Qr1[i, :, :] = C - np.matmul(zeta, Qh1[i, :, :])
                Qr1[i, 0, 0] = min(max(Qr1[i, 0, 0], 0), C[0, 0])
                Qr1[i, 0, 0] = max(Qr1[i, 0, 0], 0)
                Qr2[i, :, :] = np.matmul(alpha, C) + np.matmul(gamma, Qh2[i, :, :])
                Qr2[i, 0, 0] = max(Qr2[i, 0, 0], 0)
                Lr1[i, :] = np.matmul(B.transpose(), cp.solve_continuous_are(A - B * Lh[i, :], B, Qr1[i, :, :], 1))
                try:
                    Lr2[i, :] = np.matmul(B.transpose(), cp.solve_continuous_are(A - B * Lh[i, :], B, Qr2[i, :, :], 1))
                except:
                    Lr2[i, :] = np.array([0, 0])

                Qh1[i, :, :] = self.compute_cost(Lr1[i, :], Lh[i, :], beta, alpha_1, alpha_2)
                Qh2[i, :, :] = self.compute_cost(Lr2[i, :], Lh[i, :], beta, alpha_1, alpha_2)

            C1[i] = (-Lr1[i, 0] + Lh[i, 0]) / (Lr1[i, 0] + Lh[i, 0])
            C2[i] = (-Lr2[i, 0] + Lh[i, 0]) / (Lr2[i, 0] + Lh[i, 0])

            # Old laws


    
        outputs = {
            "cost_human_pos": Qh1,
            "cost_human_neg": Qh2,
            "cost_robot_pos": Qr1,
            "gain_robot_pos": Lr1,
            "gain_human": Lh,
            "cost_robot_neg": Qr2,
            "gain_robot_neg": Lr2,
            "auth_pos": C1,
            "auth_neg": C2,
        }
        return outputs

    def compute_cost(self, Lr, Lhat, beta, alpha_1, alpha_2):
        p = 1/beta * Lhat
        gamma_1 = alpha_1 - beta * Lr[0]
        gamma_2 = alpha_2 - beta * Lr[1]
        q_hhat1 = - 2 * gamma_1 * p[0] + Lhat[0] ** 2
        q_hhat2 = - 2 * p[0] - 2 * gamma_2 * p[1] + Lhat[1] ** 2
        Q_hhat = np.array([[q_hhat1, 0], [0, q_hhat2]])
        return Q_hhat

    def plot_strategies(self, inputs):
        Qh1 = inputs["cost_human_pos"]
        Qh2 = inputs["cost_human_neg"]
        Qr1 = inputs["cost_robot_pos"]
        Lr1 = inputs["gain_robot_pos"]
        Lh = inputs["gain_human"]
        Qr2 = inputs["cost_robot_neg"]
        Lr2 = inputs["gain_robot_neg"]
        C1 = inputs["auth_pos"]
        C2 = inputs["auth_neg"]
        
        plt.figure()
        plt.plot(Qh1[:, 0, 0], Qr1[:, 0, 0], linewidth=self.linewidth)
        plt.title("Negative reinforcement", self.csfont)
        plt.xlabel('Human cost weight (-)', self.hfont)
        plt.ylabel('Robot cost weight (-)', self.hfont)
        # plt.legend(prop={"size": 14}, loc='upper right')
        plt.xlim(-10, 10)
        plt.ylim(0, 20)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(Qh2[:, 0, 0], Qr2[:, 0, 0], linewidth=self.linewidth)
        plt.title("Positive reinforcement", self.csfont)
        plt.xlabel('Human cost weight (-)', self.hfont)
        plt.ylabel('Robot cost weight (-)', self.hfont)
        # plt.legend(prop={"size": 14}, loc='upper right')
        plt.xlim(-10, 10)
        plt.ylim(0, 20)
        # plt.xlim(Qh[0, 0, 0], Qh[-1, 0, 0])
        plt.tight_layout(pad=1)


        labels = ['Human Gain', 'Robot Gain']
        colors = [self.tud_red, self.tud_blue]

        plt.figure()
        plt.stackplot(Lh[:, 0], Lh[:, 0], Lr1[:, 0], labels=labels, colors=colors)
        plt.title("Negative reinforcement", self.csfont)
        plt.xlabel('Human gain (Nm)', self.hfont)
        plt.ylabel('Robot gain (Nm)', self.hfont)
        plt.legend(prop={"size": 14}, loc='upper right')
        plt.xlim(-2, 4)
        plt.ylim(0, 6)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.stackplot(Lh[:, 0], Lh[:, 0], Lr2[:, 0], labels=labels, colors=colors)
        plt.title("Positive reinforcement", self.csfont)
        plt.xlabel('Human gain (Nm)', self.hfont)
        plt.ylabel('Robot gain (Nm)', self.hfont)
        plt.legend(prop={"size": 14}, loc='upper right')
        plt.xlim(-2, 4)
        plt.ylim(0, 6)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(Qh1[:, 0, 0], Qr1[:, 0, 0], self.tud_blue, linewidth=self.linewidth, label="Negative Reinforcement")
        plt.plot(Qh2[:, 0, 0], Qr2[:, 0, 0], self.tud_red, linewidth=self.linewidth, label="Positive Reinforcement")
        plt.title("Interaction strategies: Costs", self.csfont)
        plt.xlabel('Human cost weight (-)', self.hfont)
        plt.ylabel('Robot cost weight (-)', self.hfont)
        plt.legend(prop={"size": 14}, loc='upper right')
        xmin = min(min(Qh1[:, 0, 0]), min(Qh2[:, 0, 0]))
        xmax = max(max(Qh1[:, 0, 0]), max(Qh2[:, 0, 0]))
        ymin = min(min(Qr1[:, 0, 0]), min(Qr2[:, 0, 0])) - 0.1
        ymax = max(max(Qr1[:, 0, 0]), max(Qr2[:, 0, 0])) + 0.1
        # plt.xlim(xmin, xmax)
        # plt.ylim(ymin, ymax)
        plt.xlim(-10, 10)
        plt.ylim(0, 20)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(Lh[:, 0], Lr1[:, 0], self.tud_blue, linewidth=self.linewidth, label="Negative Reinforcement")
        plt.plot(Lh[:, 0], Lr2[:, 0], self.tud_red, linewidth=self.linewidth, label="Positive Reinforcement")
        plt.title("Interaction strategies: Gains", self.csfont)
        plt.xlabel('Human gain (Nm)', self.hfont)
        plt.ylabel('Robot gain (Nm)', self.hfont)
        plt.legend(prop={"size": 14}, loc='upper right')
        # xmin = min(min(Lh[:, 0]), min(Lh[:, 0]))
        # xmax = min(max(Lh[:, 0]), max(Lh[:, 0]))
        # ymin = min(min(Lr1[:, 0]), min(Lr2[:, 0])) - 0.1
        # ymax = max(max(Lr1[:, 0]), max(Lr2[:, 0])) + 0.1
        plt.xlim(-2, 4)
        plt.ylim(0, 6)
        plt.tight_layout(pad=1)

        plt.figure()
        plt.plot(Lh[:, 0], C1, self.tud_blue, linewidth=self.linewidth, label="Negative Reinforcement")
        plt.plot(Lh[:, 0], C2, self.tud_red, linewidth=self.linewidth, label="Positive Reinforcement")

        plt.title("Interaction strategies: Authority", self.csfont)
        plt.xlabel('Human gain (Nm)', self.hfont)
        plt.ylabel('Robot gain (Nm)', self.hfont)
        plt.legend(prop={"size": 14}, loc='upper right')
        xmin = min(min(Lh[:, 0]), min(Lh[:, 0]))
        xmax = min(max(Lh[:, 0]), max(Lh[:, 0]))
        ymin = min(min(C1), min(C2)) - 0.1
        ymax = max(max(C1), max(C2)) + 0.1
        plt.xlim(-2, 4)
        plt.ylim(-1, 1)
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
