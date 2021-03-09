# MAIN FILE USED TO DO SIMULATIONS AND CHANGE SETTING #

import numpy as np
import matplotlib.pyplot as plt
import control

# Simulated Human Settings
# True cost values
Qh_e = 150
Qh_v = 0
Qh = np.array([[Qh_e, 0],[0, Qh_v]] )

# Robot dynamics
I = 6 #kg
D = -0.2 #N/m
# TODO: Note that xd is fixed! If xd_dot =/= 0 this does not work!
A = np.array([ [0, 1],[1, -D/I]])
B = np.array([ [0],[1/I]])

print(A, B)
xd = 0.1