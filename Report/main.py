import numpy as np
import matplotlib.pyplot as plt
import time
from Lyapunov_Li2019 import ControllerLi
from Normalized_Gradient_Flipse import ControllerNG
from Newton_Rhapson import ControllerNR
from LQR import ControllerLQ
from plots import PlotStuff

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

    # Option for which dynamics to use
    print("Choose which dynamics to use, 0: A Simple System, 1: Mass-damper system, 2: Mass-spring-damper Steering wheel")
    d = input()
    dynamics = ""
    if int(d) == 0:
        dynamics = "A Simple System"
        dynamics_name = "A_Simple_System"
        A = np.array([[0, 1], [0.5, 0.2]])
        B = np.array([[0], [0.2]])
    elif int(d) == 1:
        dynamics = "Mass-damper system"
        dynamics_name = "Mass-damper_system"
        A = np.array([[0, 1], [0, -D / I]])
        B = np.array([[0], [1 / I]])
    elif int(d) == 2:
        dynamics = "Mass-spring-damper steering wheel"
        dynamics_name = "Mass-spring-damper_steering_wheel"
        A = np.array([[0, 1], [- Bw / Jw, -Kw / Jw]])
        B = np.array([[0], [1 / Jw]])
    else:
        exit("That's no option, choose something else!")

    print("You chose wisely, your choice was: ", dynamics)

    if int(e) < 2:
        # OPTION 2: Controller
        # Option for which controller to use
        print("Choose which controller to use, 0: Full-information Optimal Control, 1: Full-information Differential Game",
              " 2: Differential Game Lyapunov cost estimator (Li2019)",
              " 3: Differential Game Normalized gradient cost estimator")
        c = input()
        controller = ""
        if int(c) == 0:
            controller = "Full-information Optimal Control"
            controller_name = "Full-information_Optimal_Control"

        elif int(c) == 1:
            controller = "Full-information Differential Game"
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
                controller_compare = "Full-information Optimal Control"
                controller_compare_name = "Full-information_Optimal_Control"

            elif int(c_compare) == 1:
                controller_compare = "Full-information Differential Game"
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
    r = generate_reference(int(d), T, int(s))

    # OPTION 4: Save figure
    # Option to save the figure
    print("Do you want to save the figure? 0: No, show figure, 1: Yes, show no figure, 2: Yes, show figure")
    save = input()

    return A, B, int(c), int(d), int(s), int(e), int(c_compare), int(save), controller, controller_name, \
           controller_compare, controller_compare_name, dynamics, dynamics_name,  r

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
        print("Excellent choice, a sine it is")
    elif s == 1:
        r = x_d * (np.sin(2 * np.pi * fs1 * T) + np.sin(2 * np.pi * fs2 * T) + np.sin(2 * np.pi * fs3 * T) + np.sin(2 * np.pi * fs4 * T))
        print("Excellent choice, a multisine it is")
    else:
        exit("That's no option, choose something else!")
    return r

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

# Simulated Human Settings
# True cost values
Qh = np.array([[100, 0], [0, 10]])
C = np.array([[50, 0], [0, 5]])

A, B, c, d, s, e, c_compare, save, controller, controller_name, controller_compare, \
controller_compare_name, dynamics, dynamics_name, r = ask_input()




# Compute inputs for simulation
inputs = {
    "simulation_steps": N,
    "step_size": h,
    "time_vector": T,
    "reference_signal": r,
    "human_weight": Qh,
    "c": c,
    "d": d,
    "controller_type_name": controller_name,
    "dynamics_type_name": dynamics_name,
    "controller_type": controller,
    "dynamics_type": dynamics,
    "save": save,
    "gain_estimation_bias": bias,
}

# Some options to choose which simulations/algorithms to do/use
if e < 2:
    if c == 0:
        # Full-information w/ Newton-Rhapson method
        controls = ControllerLQ(A, B, mu, sigma)
        inputs["robot_weight"] = 0.5 * Qh

    elif c == 1:
        # Full-information w/ Newton-Rhapson method
        controls = ControllerNR(A, B, mu, sigma)
        inputs["robot_weight"] = 0.5 * Qh

    elif c == 2:
        # Lyapunov (Li2019)
        # Different convergence rates for different dynamics
        if d == 0:
            alpha = 100000
        elif d == 1:
            alpha = 100000
        elif d == 2:
            alpha = 400

        Gamma = np.array([[2, 0], [0, 2]])
        controls = ControllerLi(A, B, mu, sigma)
        inputs["alpha"] = alpha
        inputs["Gamma"] = Gamma
        inputs["sharing_rule"] = C

    elif c == 3:
        # Normalized Gradient Cost Observer
        if d == 0:
            alpha = 8000
        elif d == 1:
            alpha = 25000
        elif d == 2:
            alpha = 50
        Gamma = alpha * np.array([[5, 0], [0, 1]])
        kappa = 2
        controls = ControllerNG(A, B, mu, sigma)
        inputs["kappa"] = kappa
        inputs["Gamma"] = Gamma
        inputs["sharing_rule"] = C

    else:
        exit('Something seems to have gone wrong, try again')

    # If we are comparing:
    # Some options to choose which simulations/algorithms to do/use
    if e == 1:
        inputs2 = {
            "simulation_steps": N,
            "step_size": h,
            "time_vector": T,
            "reference_signal": r,
            "human_weight": Qh,
            "c": c_compare,
            "d": d,
            "controller_type_name": controller_name,
            "dynamics_type_name": dynamics_name,
            "controller_type": controller,
            "dynamics_type": dynamics,
            "save": save,
            "gain_estimation_bias": bias,
        }

        if c_compare == 0:
            # Full-information w/ Newton-Rhapson method
            controls2 = ControllerLQ(A, B, mu, sigma)
            inputs2["robot_weight"] = 0.5 * Qh

        elif c_compare == 1:
            # Full-information w/ Newton-Rhapson method
            controls2 = ControllerNR(A, B, mu, sigma)
            inputs2["robot_weight"] = 0.5 * Qh

        elif c_compare == 2:
            # Lyapunov (Li2019)
            # Different convergence rates for different dynamics
            if d == 0:
                alpha = 50000
            elif d == 1:
                alpha = 100000
            elif d == 2:
                alpha = 400

            Gamma = np.array([[2, 0], [0, 2]])
            controls2 = ControllerLi(A, B, mu, sigma)
            inputs2["alpha"] = alpha
            inputs2["Gamma"] = Gamma
            inputs2["sharing_rule"] = C

        elif c_compare == 3:
            # Normalized Gradient Cost Observer
            if d == 0:
                alpha = 8000
            elif d == 1:
                alpha = 25000
            elif d == 2:
                alpha = 80
            Gamma = alpha * np.array([[5, 0], [0, 2]])
            kappa = 2
            controls2 = ControllerNG(A, B, mu, sigma)
            inputs2["kappa"] = kappa
            inputs2["Gamma"] = Gamma
            inputs2["sharing_rule"] = C

        else:
            exit('Something seems to have gone wrong, try again')
else:
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

    print("You chose ", variable)
    print("Brace yourselve, this will take some time!")

    inputs = {
        "simulation_steps": N,
        "step_size": h,
        "time_vector": T,
        "reference_signal": r,
        "human_weight": Qh,
        "c": c,
        "d": d,
        "controller_type_name": controller_name,
        "dynamics_type_name": dynamics_name,
        "controller_type": controller,
        "dynamics_type": dynamics,
        "save": save,
        "variable": variable,
        "gain_estimation_bias": bias,
    }


    # Normalized Gradient Cost Observer
    if d == 0:
        alpha = 15000
    elif d == 1:
        alpha = 25000
    elif d == 2:
        alpha = 50
    Gamma = alpha * np.array([[5, 0], [0, 1]])
    kappa = 2
    controls = ControllerNG(A, B, mu, sigma)
    inputs["kappa"] = kappa
    inputs["alpha"] = alpha
    inputs["Gamma"] = Gamma
    inputs["sharing_rule"] = C
    inputs2 = inputs.copy()
    inputs3 = inputs.copy()
    # inputs4 = inputs.copy()
    controls = ControllerNG(A, B, mu, sigma)

    # change 1 parameter
    if int(v) == 0:
        # Change kappa
        inputs["kappa"] = 0.01 * kappa
        inputs2["kappa"] = 1 * kappa
        inputs3["kappa"] = 100 * kappa
        # inputs4["kappa"] = 10 * kappa
    elif int(v) == 1:
        # Change alpha
        inputs["alpha"] = 0.2 * alpha
        inputs["Gamma"] = inputs["alpha"] * np.array([[5, 0], [0, 1]])
        inputs2["alpha"] = 1 * alpha
        inputs2["Gamma"] = inputs2["alpha"] * np.array([[5, 0], [0, 1]])
        inputs3["alpha"] = 5 * alpha
        inputs3["Gamma"] = inputs3["alpha"] * np.array([[5, 0], [0, 1]])
        # inputs4["alpha"] = 10 * alpha
        # inputs4["Gamma"] = inputs4["alpha"] * np.array([[5, 0], [0, 1]])
    elif int(v) == 2:
        inputs["gain_estimation_bias"] = np.array([0, -2])
        inputs2["gain_estimation_bias"] = np.array([0, 0])
        inputs3["gain_estimation_bias"] = np.array([0, 2])
    else:
        exit("Something went wrong in line 383")

# Simulate
outputs = controls.simulate(inputs)
if e == 1:
    outputs2 = controls2.simulate(inputs2)

if e == 2:
    outputs3 = controls.simulate(inputs3)
    outputs2 = controls.simulate(inputs2)
    # outputs4 = controls.simulate(inputs4)

# Plot
plot_object = PlotStuff()
if e == 0:
    plot_object.plot_stuff(inputs, outputs)
elif e == 1:
    plot_object.plot_comparison(inputs, inputs2, outputs, outputs2)
elif e == 2:
    # plot_object.plot_sensitivity(inputs, inputs2, inputs3, inputs4, outputs, outputs2, outputs3, outputs4)
    plot_object.plot_sensitivity(inputs, inputs2, inputs3, outputs, outputs2, outputs3)
