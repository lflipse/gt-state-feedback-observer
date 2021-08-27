import numpy as np
import matplotlib.pyplot as plt
import time
import sys
import scipy.linalg as cp

sys.path.insert(1, '..')

from Simulation.Differential_Game_Li2019 import ControllerLi
from Simulation.Differential_Game_Gain_Observer import ControllerNG as ControllerNGObs
from Simulation.Differential_Game_Gain_Descent import ControllerNG
from Simulation.Differential_Game import ControllerDG
from Simulation.Linear_Quadratic import ControllerLQ
from Simulation.plots import PlotStuff

########################################################

#   APOLOGIES BEFOREHAND, THIS SCRIPT HAS BECOME QUITE  #
#   THE MONSTER TO GET ALL FUNCTIONALITIES IN. JUST RUN #
#   THE SCRIPT AND YOU'LL BE FINE                       #

########################################################

# Functions
def ask_input():
    # Option to choose what kind of experiment
    # Option 0, show 1 controller. Option 1, compare 2 controllers, Option 3, sensitivity analysis (Normalized Gradient)
    print("Choose an experiment type, 0: Controller Performance, 1: Compare Controller, 2: Sensitivity analysis")
    e = input()
    experiment = ""
    if int(e) == 0:
        experiment = ""
    elif int(e) == 1:
        experiment = "Comparison"
    elif int(e) == 2:
        experiment = "Analysis"

    # Dynamics to use
    d = 0
    dynamics = "Example System"
    dynamics_name = "A_Simple_System"

    A = np.array([[0, 1], [0, -10]])
    B = np.array([[0], [20]])

    if int(e) < 2:
        # OPTION 2: Controller
        # Option for which controller to use
        print("Choose which controller to use, 0: Full-information Optimal Control, 1: Full-information Differential Game",
              " 2: Differential Game Lyapunov cost estimator (Li2019)",
              " 3: Differential Game Normalized gradient cost estimator")
        c = input()
        controller = ""
        if int(c) == 0:
            controller = "Optimal Control"
            controller_name = "Full-information_Optimal_Control"

        elif int(c) == 1:
            controller = "Differential Game"
            controller_name = "Full-information_Differential_Game"

        elif int(c) == 2:
            controller = "Differential Game Lyapunov cost estimator (Li2019)"
            controller_name = "Differential_Game_Lyapunov_cost_estimator_(Li2019)"

        elif int(c) == 3:
            controller = "Differential Game Normalized gradient cost estimator"
            controller_name = "Differential_Game_Normalized_gradient_cost_estimator"

        else:
            exit("That's no option, choose something else!")

        print("You chose wisely, your choice was: ", controller)

        # For the case we are comparing
        if int(e) == 1:
            print(
                "Choose which controller to compare the ", controller, " with, 0: Full-information Optimal Control, 1: Full-information Differential Game",
                " 2: Differential Game Lyapunov cost estimator (Li2019)",
                " 3: Differential Game Normalized gradient cost estimator")
            c_compare = input()
            controller_compare = ""
            if int(c_compare) == 0:
                controller_compare = "Optimal Control"
                controller_compare_name = "Full-information_Optimal_Control"

            elif int(c_compare) == 1:
                controller_compare = "Differential Game"
                controller_compare_name = "Full-information_Differential_Game"

            elif int(c_compare) == 2:
                controller_compare = "Differential Game Lyapunov cost estimator (Li2019)"
                controller_compare_name = "Differential_Game_Lyapunov_cost_estimator_(Li2019)"

            elif int(c_compare) == 3:
                controller_compare = "Differential Game Normalized gradient cost estimator"
                controller_compare_name = "Differential_Game_Normalized_gradient_cost_estimator"

            else:
                exit("That's no option, choose something else!")

            print("You chose wisely, your choice was: ", controller_compare)
        else:
            c_compare = -1
            controller_compare = ""
            controller_compare_name = ""
    else:
        c = 3
        controller = "Differential Game Normalized gradient cost estimator"
        controller_name = "Differential_Game_Normalized_gradient_cost_estimator"
        c_compare = -1
        controller_compare = ""
        controller_compare_name = ""

    # OPTION 3: Reference signal
    # Option for which reference to use
    print("Choose which reference signal to use, 0: Sine, 1: Multisine")
    s = input()
    r, reference = generate_reference(int(d), T, int(s))

    # OPTION 4: Save figure
    # Option to save the figure
    print("Do you want to save the figure? 0: No, show figure, 1: Yes, show no figure, 2: Yes, show figure")
    save = input()

    return A, B, int(c), int(d), int(s), int(e), int(c_compare), int(save), controller, controller_name, \
           controller_compare, controller_compare_name, dynamics, dynamics_name,  r, reference

def generate_reference(d, T, s):
    x_d = 0
    if d == 0:
        x_d = 0.1
    elif d == 1:
        x_d = 0.1
    elif d == 2:
        x_d = 40*np.pi/180
    else:
        exit("That's no option, choose something else!")

    # Reference signal
    fs1 = 1 / 8
    fs2 = 1 / 20
    fs3 = 1 / 37
    fs4 = 1 / 27

    if s == 0:
        r = x_d * (np.sin(2 * np.pi * fs1 * T))
        rdot = 2 * np.pi * fs1 * x_d * (np.cos(2 * np.pi * fs1 * T))
        print("Excellent choice, a sine it is")
    elif s == 1:
        r = x_d * (np.sin(2 * np.pi * fs1 * T) + np.sin(2 * np.pi * fs2 * T) + np.sin(2 * np.pi * fs3 * T) + np.sin(2 * np.pi * fs4 * T))
        rdot = x_d * (2 * np.pi * fs1 * np.cos(2 * np.pi * fs1 * T) + 2 * np.pi * fs2 * np.cos(2 * np.pi * fs2 * T) +
                   2 * np.pi * fs3 * np.cos(2 * np.pi * fs3 * T) + 2 * np.pi * fs4 * np.cos(2 * np.pi * fs4 * T))
        print("Excellent choice, a multisine it is")
    else:
        exit("That's no option, choose something else!")
    reference = np.array([r, rdot])
    return r, reference.transpose()

def solve_coupled_riccati(it, Qr, Qh, A, B):
    S = B * B.transpose()
    P_it = np.zeros((it + 1, 2, 2))
    Ph_it = np.zeros((it + 1, 2, 2))
    P_it[0, :, :] = cp.solve_continuous_are(A, B, Qr, 1)
    Ph_it[0, :, :] = cp.solve_continuous_are(A, B, Qh, 1)

    for i in range(it):
        Acl = A - np.matmul(S, (Ph_it[i, :, :]))
        Aclh = A - np.matmul(S, (P_it[i, :, :]))
        P_it[i + 1, :, :] = cp.solve_continuous_are(Acl, B, Qr, 1)
        Ph_it[i + 1, :, :] = cp.solve_continuous_are(Aclh, B, Qh, 1)

    Ph = Ph_it[it, :, :]
    Pr = P_it[it, :, :]
    Lr = np.matmul(B.transpose(), Pr)
    Lh = np.matmul(B.transpose(), Ph)
    return Lr, Lh

def generate_controls(c, input_dict, v=None, v_val=1.0):
    inputs = input_dict.copy()
    inputs["c"] = c
    if c == 0:
        # Full-information w/ Newton-Rhapson method
        controls = ControllerLQ(A, B, mu, sigma, False)
        inputs["robot_weight"] = C

    elif c == 1:
        # Full-information w/ Newton-Rhapson method
        controls = ControllerDG(A, B, mu, sigma, False)
        inputs["robot_weight"] = C

    elif c == 2:
        # Lyapunov (Li2019)
        # Different convergence rates for different dynamics
        alpha = 1000
        Gamma = np.array([[2, 0], [0, 2]])
        controls = ControllerLi(A, B, mu, sigma, False)
        inputs["alpha"] = alpha
        inputs["Gamma"] = Gamma
        inputs["sharing_rule"] = C

    elif c == 3:
        # Normalized Gradient Cost Observer
        alpha = 200
        Gamma = 4 * np.array([[-2, 0], [0, -2]])
        K = alpha * np.array([[5, 0], [0, 5]])
        kappa = 1
        controls = ControllerNGObs(A, B, mu, sigma, False)

        inputs["kappa"] = kappa
        inputs["Gamma"] = Gamma
        inputs["K"] = K
        inputs["sharing_rule"] = C

        if v == 0:
            inputs["kappa"] = kappa * v_val
        elif v == 1:
            inputs["K"] = K * v_val
        elif v == 2:
            inputs["gain_bias"] = np.array([0.0, v_val])


    else:
        exit('Something seems to have gone wrong, try again')

    return controls, inputs

# Simulation parameters
t = 20
h = 0.01
N = round(t/h) + 1
T = np.array(range(N)) * h

# Noise parameters
mu = 0.0
sigma = 0.0
bias = np.array([0, 0])

# Initial values
pos0 = 0
vel0 = 0
initial_state = np.array([pos0, vel0])
initial_error = np.array([pos0, vel0])
initial_input = 0

# Simulated Human Settings
# True cost values
Qh = np.array([[20, 0], [0, 0.5]])
C = np.array([[50, 0], [0, 1]])

A, B, c, d, s, e, c_compare, save, controller, controller_name, controller_compare, \
controller_compare_name, dynamics, dynamics_name, r, ref = ask_input()

Lr, Lh = solve_coupled_riccati(40, C-Qh, Qh, A, B)
vhg = np.tile(Lh.transpose(), (1, N))

# Compute inputs for simulation
input_dict = {
    "simulation_steps": N,
    "step_size": h,
    "time_vector": T,
    "reference_signal": r,
    "human_weight": Qh,
    "robot_weight": C,
    "sharing_rule": C,
    "virtual_human_gain": vhg,
    "c": c,
    "d": d,
    "controller_type_name": controller_name,
    "dynamics_type_name": dynamics_name,
    "controller_type": controller,
    "dynamics_type": dynamics,
    "save": save,
    "gain_estimation_bias": bias,
    "initial_state": initial_state,
    "u_initial": initial_input,
    "e_initial": initial_error,
    "reference": ref,
    "time": T,
}

controls, inputs = generate_controls(c, input_dict)
inputs2 = None
inputs3 = None
if e == 1:
    controls2, inputs2 = generate_controls(c_compare, input_dict)
# Sensitivity Analysis
elif e == 2:
    print("Choose which variable to test: 0: kappa, 1: alpha, 2: velocity gain bias")
    v = input()
    if int(v) == 0:
        variable = "kappa"
    elif int(v) == 1:
        variable = "alpha"
    elif int(v) == 2:
        variable = "gain_estimation_bias"
    else:
        exit("Something went wrong, that was no option.")
    controls2, inputs2 = generate_controls(c, input_dict, v=int(v), v_val=0.5)
    controls3, inputs3 = generate_controls(c, input_dict, v=int(v), v_val=2)


# Simulate
outputs = controls.simulate(inputs)
outputs2 = None
outputs3 = None

if e == 1:
    outputs2 = controls2.simulate(inputs2)
if e == 2:
    outputs3 = controls.simulate(inputs3)
    outputs2 = controls.simulate(inputs2)

# Plot
plot_object = PlotStuff()
plot_object.plot_stuff(inputs, outputs, inputs2=inputs2, inputs3=inputs3, outputs2=outputs2, outputs3=outputs3)
