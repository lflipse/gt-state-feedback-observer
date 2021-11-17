import numpy as np
import pandas as pd
import seaborn as sb
import matplotlib.pyplot as plt

participant = []
for i in range(18):
    for j in range(3):
        participant.append(i)

latin_square = ["Positive Reinforcement", "Negative Reinforcement", "Manual Control",
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

s1 = [
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

s2 = [1, 2, -2,
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

s3 = [
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

s4 = [
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

auth = (np.array(s1) - np.array(s2)) / (np.array(s1) + np.array(s2) + 4)

questionnaire = {
    "condition": latin_square,
    "participant": participant,
    "question_1": s1,
    "question_2": s2,
    "question_3": s3,
    "question_4": s4,
    "auth": auth,
}

q_pd = pd.DataFrame(questionnaire)
# print(q_pd)


tud_green = "#00A390"
tud_orange = "#EB7245"
tud_otherblue = "#61A4B4"
colors = {"Manual Control": tud_orange, "Positive Reinforcement": tud_green,
                       "Negative Reinforcement": tud_otherblue}


fig = plt.figure()
ax = fig.gca()
sb.scatterplot(data=q_pd, x="auth", y="question_3", hue="condition", style="condition", alpha=0.7, ax=ax, palette=colors, s=100)
ax.set_xlabel("Percieved control share")
ax.set_ylabel("Percieved contribution")
plt.show()