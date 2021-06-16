import numpy as np
import multiprocessing as mp
import time

from Experiment.reference_trajectory import Reference
from Experiment.live_plotter import LivePlotter
from Experiment.SensoDrive.SensoDriveMultiprocessing import SensoDriveModule
from Experiment.experiment import Experiment
from Experiment.Controllers.LQR import ControllerLQ
from Experiment.Controllers.Differential_Game import ControllerDG
from Experiment.Controllers.Differential_Game_Gain_Observer import ControllerDG_GObs
from Experiment.plots import PlotStuff
from Report.LQR import ControllerLQ as LQsim
from Report.Differential_Game import ControllerDG as DGsim
from Report.Normalized_Gradient_Flipse import ControllerNG as NGsim

def input_controller():
    # Select controller type
    print("Choose a controller type. 0: Manual control, 1: LQ control, 2: Differential Game, 3: Differential Game w/ obs")
    exp_type = input("Please choose: ")
    if int(exp_type) == 0:
        controller = None
        controller_type = "Manual"
    elif int(exp_type) == 1:
        controller = ControllerLQ(A, B, Qr)
        controller_type = "LQ"
    elif int(exp_type) == 2:
        controller = ControllerDG(A, B, Qr, Qh)
        controller_type = "DG"
    elif int(exp_type) == 3:
        controller = ControllerDG_GObs(A, B, Gamma, kappa, Qr, Qh)
        controller_type = "Gain_observer"
    else:
        print("You chose: ", int(exp_type))
        exit("That's no option, try again.")

    print("Validate using simulation data? No = 0, Yes = 1")
    sim = input("Please choose: ")
    return controller, controller_type, sim, exp_type

def run_simulation(experiment_data, exp_type):
    # Use the correct settings for the simulation
    inputs = {
        "simulation_steps": N,
        "step_size": t_step,
        "time_vector": T,
        "reference_signal": r,
        "human_weight": Qh,
        "robot_weight": Qr,
        "alpha": alpha,
        "Gamma": Gamma,
        "kappa": kappa,
        "initial_state": [experiment_data["position"][0], experiment_data["velocity"][0]],
        "sharing_rule": C,
        "controller_type_name": controller_type,
        "dynamics_type_name": "Steering_dynamics",
        "controller_type": controller_type,
        "dynamics_type": "Steering_dynamics",
        "save": 0,
        "gain_estimation_bias": 0.0,
    }

    if int(exp_type) == 0:
        exit("That's not possible")
    elif int(exp_type) == 1:
        controller_sim = LQsim(A, B, mu=0.0, sigma=0.02)
    elif int(exp_type) == 2:
        controller_sim = DGsim(A, B, mu=0.0, sigma=0.02)
    elif int(exp_type) == 3:
        controller_sim = NGsim(A, B, mu=0.0, sigma=0.02)

    simulation_data = controller_sim.simulate(inputs)
    return simulation_data

# This statement is necessary to allow for multiprocessing
if __name__ == "__main__":

    # Simulation parameters
    t_exp = 10
    t_step = 0.007
    N = round(t_exp/t_step) + 1
    T = np.array(range(N)) * t_step
    fr_min = 1/(20 * np.pi)
    fr_max = 1/(2 * np.pi)
    increments = 20
    reference_generator = Reference()
    r = reference_generator.generate_reference(T, fr_max, fr_min, increments)

    # Dynamics
    Jw = 0.0447
    Bw = 0.7  # Max = 0.5
    Kw = 3.0  # Max = 2.5
    A = np.array([[0, 1], [- Bw / Jw, - Kw / Jw]])
    B = np.array([[0], [1 / Jw]])

    # TODO: verify values
    alpha = 20
    Gamma = alpha * np.array([[5, 0], [0, 1]])
    kappa = 0.7
    Qr = 10*np.array([[10.0, 0.0], [0.0, 0.0]])
    Qh = np.array([[0.0, 0.0], [0.0, 0.0]])
    C = Qr

    screen_width = 1920
    screen_height = 1080
    # screen_width = 720
    # screen_height = 480

    # ask input
    controller, controller_type, sim, exp_type = input_controller()

    # Start the senso drive parallel process
    parent_conn, child_conn = mp.Pipe(True)
    senso_drive_process = SensoDriveModule(Bw, Kw, parent_conn, child_conn)
    senso_drive_process.start()
    print("process started!")

    # Start the data plotting parallel process
    send_conn, receive_conn = mp.Pipe(True)
    live_plotter_process = LivePlotter(receive_conn)
    live_plotter_process.start()
    print("Second process started!")

    # Time to do an experiment!
    full_screen = True
    preview = False
    experiment_handler = Experiment(Bw, Kw, send_conn, parent_conn, child_conn, screen_width, screen_height,
                                    full_screen, controller, controller_type, preview)
    experiment_data = experiment_handler.experiment(r, N, t_step)
    senso_drive_process.join()
    live_plotter_process.join()

    # Plot stuff
    plot_stuff = PlotStuff()

    if int(sim) == 1:
        simulation_data = run_simulation(experiment_data, exp_type)
        plot_stuff.plot_stuff_with_sim_data(experiment_data, simulation_data, controller_type)
    else:
        plot_stuff.plot_stuff(experiment_data, controller_type)

