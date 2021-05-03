import numpy as np
import scipy.linalg as cp
import matplotlib.pyplot as plt
import time

# Functions
def ask_input():
    # OPTION 1: Dynamics
    # Reaching movements
    I = 6  # kg
    D = -0.2  # N/m

    # Steering wheel dynamics
    Jw = 0.22
    Bw = 0.1
    Kw = 1

    # Option for which dynamics to use
    print("Choose which dynamics to use, 0: Mass-damper system, 1: Mass-spring-damper Steering wheel")
    d = input()
    dynamics = ""
    if int(d) == 0:
        dynamics = "Mass-damper system"
        A = np.array([[0, 1], [0, -D / I]])
        B = np.array([[0], [1 / I]])
    elif int(d) == 1:
        dynamics = "Mass-spring-damper steering wheel"
        A = np.array([[0, 1], [- Bw / Jw, -Kw / Jw]])
        B = np.array([[0], [1 / Jw]])
    else:
        exit("That's no option, choose something else!")

    print("You chose wisely, your choice was: ", dynamics)

    # OPTION 2: Controller
    # Option for which controller to use
    print("Choose which controller to use, 0: Normalized gradient cost estimator, 1: Lyapunov cost estimator (Li2019),"
          " 2: Full-information")
    c = input()
    controller = ""
    if int(c) == 0:
        controller = "Normalized gradient cost estimator"

    elif int(c) == 1:
        controller = " Lyapunov cost estimator (Li2019)"

    elif int(c) == 2:
        controller = " Full-information"

    else:
        exit("That's no option, choose something else!")

    print("You chose wisely, your choice was: ", controller)
    return A, B, int(c), int(d), controller, dynamics

def generate_reference(d):
    x_d = 0
    if d == 0:
        x_d = 0.1
    elif d == 1:
        x_d = theta_d = 40*np.pi/180
    else:
        exit("That's no option, choose something else!")

    # Reference signal
    fs1 = 1 / 8
    fs2 = 1 / 20
    fs3 = 1 / 37
    fs4 = 1 / 27
    r = x_d * (np.sin(2 * np.pi * fs1 * T) + np.sin(2 * np.pi * fs2 * T) + np.sin(2 * np.pi * fs3 * T) + np.sin(
        2 * np.pi * fs4 * T))
    return r

A, B, c, d, controller, dynamics = ask_input()
r = generate_reference(d)


alpha = 140
Gamma = alpha * np.array([[4, 0], [0, 1]])
mu = 0.0
sigma = 0.1

# Initial values
pos0 = 0
vel0 = 0


# Simulation
t = 20
h = 0.01
N = round(t/h)
T = np.array(range(N)) * h

# Simulated Human Settings
# True cost values
Qh_e = 100
Qh_v = 50
Qh0 = np.array([[Qh_e, 0], [0, Qh_v]])

# Estimated cost value
Qh_e_hat = 0
Qh_v_hat = 0
Qh_hat = np.array([[Qh_e_hat, 0], [0, Qh_v_hat]])

# Robot cost values
# Cval = np.array([1200 * np.ones(round(0.5*N)), 4000 * np.ones(round(0.5*N))]).flatten()
Cval = np.array([50 * np.ones(round(N))]).flatten()
C = np.array([[Cval[0], 0], [0, 100]])
R = np.array([[1]])
Qr_hat = C - Qh_hat

# Reference signal
fs1 = 1/8
fs2 = 1/20
fs3 = 1/37
fs4 = 1/27
r = theta_d * (np.sin(2*np.pi*fs1*T) + np.sin(2*np.pi*fs2*T) + np.sin(2*np.pi*fs3*T) + np.sin(2*np.pi*fs4*T))

# Robot estimate has an estimator for the human cost
# Human estimate has an estimator for the robot cots
x0 = np.array([pos0, vel0])
x_r_hat0 = x0
x_h_hat0 = x0
x_r_tilde0 = np.array([0, 0])
x_h_tilde0 = np.array([0, 0])
# Ph0 = cp.solve_continuous_are(A, B, Qh_hat, R)
# Pr0 = cp.solve_continuous_are(A - 1/R * B * np.matmul(B.transpose(), Ph0), B, Qr_hat, R)
Ph0 = np.zeros((2, 2))
Pr0 = np.zeros((2, 2))

# y = [x, x_r_hat, x_r_tilde, Ph_hat, x_h_hat, x_h_tilde, Pr_hat]
y0o = np.array([x0, x_h_tilde0, x_h_hat0, Ph0[0]])
y0 = y0o.flatten()

# Initialize model
dynamics_model = DynamicsModel(Jw, Bw, Kw, alpha, Gamma, mu, sigma)

print("Starting simulation")
start = time.time()
# Simulate