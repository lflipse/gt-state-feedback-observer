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
    dynamics = "Example System"
    dynamics_name = "A_Simple_System"

    A = np.array([[0, 1], [0, -10]])
    B = np.array([[0], [20]])

    if int(e) < 2:
        # OPTION 2: Controller
        # Option for which controller to use
        if int(e) == 1:
            print("Compare 0: Cost observer, 1: Differential Game")
            c = input()
            if int(c) == 1:
                controller = "Differential Game"
                controller_name = "Full-information_Differential_Game"
        else:
            c = 3
            controller = "Differential Game Normalized gradient cost estimator"
            controller_name = "Differential_Game_Normalized_gradient_cost_estimator"

        # For the case we are comparing
        if int(e) == 1:
            print(
                "Compare with, 0: Optimal Control, 1: Differential Game, 2: Li2019")
            c_compare = input()
            controller_compare = ""
            if int(c_compare) == 0:
                controller_compare = "Optimal Control"
                controller_compare_name = "Full-information_Optimal_Control"

            elif int(c_compare) == 1:
                controller_compare = "Differential Game"
                controller_compare_name = "Full-information_Differential_Game"

            elif int(c_compare) == 2:
                controller_compare = "Li et al. (2019) Algorithm"
                controller_compare_name = "Differential_Game_Lyapunov_cost_estimator_(Li2019)"

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

    r, reference = generate_reference(T)

    # OPTION 4: Save figure
    # Option to save the figure
    print("Do you want to save the figure? 0: No, 1: Yes")
    q_save = input()
    if int(q_save) == 0:
        save = False
    else:
        save = True

    return A, B, int(e), int(c), int(c_compare), save, controller, controller_name, \
           controller_compare, controller_compare_name, dynamics, dynamics_name,  r, reference

def generate_reference(T):
    x_d = 0.5

    # Reference signal
    fs1 = 1 / 5

    # Make signals
    r = x_d * (np.sin(2 * np.pi * fs1 * T))
    rdot = 2 * np.pi * fs1 * x_d * (np.cos(2 * np.pi * fs1 * T))

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

def solve_single_riccati(Qr, A, B):
    Pr = cp.solve_continuous_are(A, B, Qr, 1)
    Lr = np.matmul(B.transpose(), Pr)
    return Lr

def generate_controls(controller, input_dict, v=None, v_val=1.0):
    print("controller = ", controller)
    inputs = input_dict.copy()
    inputs["controller_type"] = controller
    if controller == "Optimal Control":
        # Optimal Control
        controls = ControllerLQ(A, B, mu, sigma, False)
        inputs["robot_weight"] = C

    elif controller == "Differential Game":
        # Full-information w/ Newton-Rhapson method
        controls = ControllerDG(A, B, mu, sigma, False)
        inputs["robot_weight"] = C

    elif controller == "Li et al. (2019) Algorithm":
        # Lyapunov (Li2019)
        # Different convergence rates for different dynamics
        alpha = 1000
        Gamma = np.array([[2, 0], [0, 2]])
        controls = ControllerLi(A, B, mu, sigma, False)
        inputs["alpha"] = alpha
        inputs["Gamma"] = Gamma
        inputs["sharing_rule"] = C

    else:
        # Normalized Gradient Cost Observer
        alpha = 100
        Gamma = 4 * np.array([[2, 0], [0, 2]])
        K = alpha * np.array([[8, 0], [0, 3]])
        kappa = 1
        controls = ControllerNGObs(A, B, mu, sigma, False)

        inputs["kappa"] = kappa
        inputs["Gamma"] = Gamma
        inputs["K"] = K
        inputs["sharing_rule"] = C
        inputs["v"] = v

        if v == 0:
            inputs["kappa"] = kappa * v_val**12
        elif v == 1:
            inputs["K"] = K * v_val
        elif v == 2:
            if v_val < 1:
                inputs["gain_estimation_bias"] = np.array([0.0, 2.8])
            else:
                inputs["gain_estimation_bias"] = np.array([0.0, -1.4])

    return controls, inputs

# Simulation parameters
t = 12
h = 0.005
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

A, B, e, c, c_compare, save, controller, controller_name, controller_compare, \
controller_compare_name, dynamics, dynamics_name, r, ref = ask_input()

print("c = ", c, "c_comp = ", c_compare)

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
    "kappa": 1,
}

controls, inputs = generate_controls(controller, input_dict)
inputs2 = None
inputs3 = None
v = 0
variable = "kappa"
if e == 1:
    controls2, inputs2 = generate_controls(controller_compare, input_dict)
# Sensitivity Analysis
elif e == 2:
    print("Choose which variable to 1: 0: kappa, 1: K, 2: velocity gain bias")
    v = input()
    if int(v) == 0:
        variable = "kappa"
    elif int(v) == 1:
        variable = "alpha"
    elif int(v) == 2:
        variable = "gain_estimation_bias"
    else:
        exit("Something went wrong, that was no option.")
    controls2, inputs2 = generate_controls(controller, input_dict, v=int(v), v_val=0.5)
    controls3, inputs3 = generate_controls(controller, input_dict, v=int(v), v_val=2)


# Simulate
large = input("large font/lines? Enter 1 --->  ")
try:
    if int(large) == 1:
        size = "large"
    else:
        size = "small"
except:
    size = "small"


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
plot_object.plot_stuff(inputs, outputs, v=int(v), inputs2=inputs2, inputs3=inputs3,
                       outputs2=outputs2, outputs3=outputs3, save=save, size=size)
