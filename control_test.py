# FILE TO TEST THE CONTROL PYTHON PACKAGE #

import numpy as np
import scipy.linalg as cp

# Simulated Human Settings
# True cost values
Qh_e = 150
Qh_v = 0
Qh = np.array([[Qh_e, 0],[0, Qh_v]] )

# Estimated cost value
Qh_e_hat = 100
Qh_v_hat = 4
Qh_hat = np.array([[Qh_e_hat, 0],[0, Qh_v_hat]] )

# Robot cost values
C = np.array([[300, 0],[0, 0]] )
Q = C - Qh_hat
# R = np.eye(2)
R = np.array([[1]])

I = 6 #kg
D = -0.2 #N/m
# TODO: Note that xd is fixed! If xd_dot =/= 0 this does not work!
A = np.array([ [0, 1],[1, -D/I]])
B = np.array([[0], [1/I]])
C = np.eye(2)
D = np.array([[0],[0]])
Gamma = np.array([[100, 0], [0, 1]])

# Initial values
pos0 = 0
vel0 = 0
x_d = 0.1

x = np.array([[pos0-x_d],[vel0]])

Ph = cp.solve_continuous_are(A, B, Qh, R)
Ph_hat = cp.solve_continuous_are(A, B, Qh_hat, R)
P = cp.solve_continuous_are(A, B, Q, R)
print(Ph.shape, P.shape)

Lh = np.matmul(B.transpose(), Ph)
Lh_hat = np.matmul(B.transpose(), Ph_hat)
L = np.matmul(B.transpose(), P)

print(Lh, Lh_hat, L)

uh = np.matmul(-Lh, x)
uh_hat = np.matmul(-Lh_hat, x)
u = np.matmul(-L, x)

print(u)