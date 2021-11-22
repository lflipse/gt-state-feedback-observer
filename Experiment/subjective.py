import numpy as np
import pandas as pd
import seaborn as sb
import matplotlib.pyplot as plt

class Subjective():
    def __init__(self):
        participant = []
        for i in range(18):
            for j in range(3):
                participant.append(i)

        self.latin_square = ["Positive Reinforcement", "Negative Reinforcement", "Manual Control",
                            "Manual Control", "Positive Reinforcement", "Negative Reinforcement",
                            "Positive Reinforcement", "Negative Reinforcement", "Manual Control",
                            "Manual Control", "Positive Reinforcement", "Negative Reinforcement",
                            "Positive Reinforcement", "Negative Reinforcement", "Manual Control",
                            "Manual Control", "Positive Reinforcement", "Negative Reinforcement",
                            "Manual Control", "Negative Reinforcement", "Positive Reinforcement",
                            "Negative Reinforcement", "Manual Control", "Positive Reinforcement",
                            "Manual Control", "Negative Reinforcement", "Positive Reinforcement",
                            "Negative Reinforcement", "Manual Control", "Positive Reinforcement",
                            "Manual Control", "Negative Reinforcement", "Positive Reinforcement",
                            "Negative Reinforcement", "Manual Control", "Positive Reinforcement",
                            "Negative Reinforcement", "Positive Reinforcement", "Manual Control",
                            "Positive Reinforcement", "Manual Control", "Negative Reinforcement",
                            "Negative Reinforcement", "Positive Reinforcement", "Manual Control",
                            "Positive Reinforcement", "Manual Control", "Negative Reinforcement",
                            "Negative Reinforcement", "Positive Reinforcement", "Manual Control",
                            "Positive Reinforcement", "Manual Control", "Negative Reinforcement",
                            ]

        self.s1 = [
            1, 1, 2,
            1, 1, -2,
            1, 0, 2,
            2, 2, 1,
            2, 2, 1,
            2, 1, 1,
            2, 1, 1,
            0, 2, 2,
            2, 0, 1,
            -1, 2, 1,
            2, 1, 1,
            -1, 2, -1,
            -1, 2, 2,
            1, 2, -1,
            -1, 2, 2,
            1, 2, -1,
            2, 1, 2,
            1, 2, 0,
        ]

        self.s2 = [1, 2, -2,
              -1, 1, 2,
              1, 1, -2,
              -2, 0, 1,
              -2, -2, -1,
              -1, 1, 0,
              -2, 1, 0,
              0, -2, 1,
              -2, 1, 1,
              1, -2, -1,
              -1, 0, 0,
              1, 2, 1,
              2, -1, -2,
              0, -2, 2,
              2, 0, -2,
              -1, -2, 2,
              1, 1, -2,
              2, -1, 2,
              ]

        self.s3 = [
            1, 1, 2,
            2, 1, 1,
            1, 1, 2,
            2, 2, 1,
            2, 2, 1,
            2, 0, 1,
            2, 2, 2,
            1, 2, 1,
            1, 1, 1,
            0, 2, 2,
            2, 2, 0,
            -1, 2, -1,
            -2, 2, 2,
            2, 2, -1,
            1, 2, 2,
            2, 2, 1,
            1, 0, 2,
            0, 2, -1,
        ]

        self.s4 = [
            -1, 1, -2,
            1, 2, 2,
            1, 2, -2,
            -2, 1, 2,
            -2, -2, 1,
            -2, 2, 1,
            -2, 2, 1,
            1, -2, -1,
            -2, 1, 1,
            0, -2, 0,
            -1, 1, 1,
            1, -1, 1,
            2, -2, -2,
            1, -2, 2,
            2, 1, -2,
            1, -1, 0,
            1, 2, -1,
            1, -2, 2
        ]

        self.auth = (np.array(self.s1) - np.array(self.s2)) / (np.array(self.s1) + np.array(self.s2) + 4)
        self.s1_ordered = []
        self.s2_ordered = []
        self.s3_ordered = []
        self.s4_ordered = []
        self.auth_ordered = []

        for i in range(18):
            conds = self.latin_square[i*3:(i+1)*3]
            # put in order manual, negative, positive
            p1 = conds.index("Manual Control")
            p2 = conds.index("Negative Reinforcement")
            p3 = conds.index("Positive Reinforcement")
            s1 = self.s1[i * 3:(i + 1) * 3]
            s2 = self.s2[i * 3:(i + 1) * 3]
            s3 = self.s3[i * 3:(i + 1) * 3]
            s4 = self.s4[i * 3:(i + 1) * 3]
            auth = self.auth[i * 3:(i + 1) * 3]
            self.s1_ordered.extend([s1[p1], s1[p2], s1[p3]])
            self.s2_ordered.extend([s2[p1], s2[p2], s2[p3]])
            self.s3_ordered.extend([s3[p1], s3[p2], s3[p3]])
            self.s4_ordered.extend([s4[p1], s4[p2], s4[p3]])
            self.auth_ordered.extend([auth[p1], auth[p2], auth[p3]])
        # print(self.auth_ordered)

# sub = Subjective()
# questionnaire = {"question_1":sub.s1,
#                  "question_2":sub.s2,
#                  "question_3":sub.s3,
#                  "question_4":sub.s4,
#                  "auth":sub.auth,
#                  "condition":sub.latin_square}
# q_pd = pd.DataFrame(questionnaire)
# # print(q_pd)
#
#
# tud_green = "#00A390"
# tud_orange = "#EB7245"
# tud_otherblue = "#61A4B4"
# colors = {"Manual Control": tud_orange, "Positive Reinforcement": tud_green,
#                        "Negative Reinforcement": tud_otherblue}
#
#
# fig = plt.figure()
# ax = fig.gca()
# sb.scatterplot(data=q_pd, x="auth", y="question_3", hue="condition", style="condition", alpha=0.7, ax=ax, palette=colors, s=100)
# ax.set_xlabel("Percieved control share")
# ax.set_ylabel("Percieved contribution")
# plt.show()