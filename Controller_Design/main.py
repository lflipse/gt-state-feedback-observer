import numpy as np
import multiprocessing as mp
import time
import wres
import platform
import scipy.linalg as cp
import pandas as pd
import sys

sys.path.insert(1, '..')

# from Controller_Design.Controllers.Linear_Quadratic import ControllerLQ
# from Controller_Design.Controllers.Differential_Game import ControllerDG
# from Controller_Design.Controllers.Differential_Game_Gain_Descent import ControllerDG_GObs
# from Controller_Design.Controllers.Li2019 import ControllerDG_Li

from Controller_Design.reference_trajectory import Reference
from Controller_Design.SensoDrive.SensoDriveMultiprocessing import SensoDriveModule
from Controller_Design.experiment import Experiment
from Controller_Design.Controllers.Differential_Game_Gain_Observer import ControllerDGObs
from Controller_Design.plots import PlotStuff
from Simulation.Linear_Quadratic import ControllerLQ as LQsim
from Simulation.Differential_Game_Gain_Descent import ControllerNG as NGsim
from Simulation.Differential_Game_Gain_Observer import ControllerNG as NG_Obssim

def compute_virtual_gain(Qh1, Qr_end, A, B):
    # Iterative procedure for calculating gains
    Lh = np.matmul(B.transpose(), cp.solve_continuous_are(A, B, Qh1, 1))
    Lr = np.matmul(B.transpose(), cp.solve_continuous_are(A, B, Qr_end, 1))
    for i in range(10):
        Lh = np.matmul(B.transpose(), cp.solve_continuous_are(A - B * Lr, B, Qh1, 1))
        Lr = np.matmul(B.transpose(), cp.solve_continuous_are(A - B * Lh, B, Qr_end, 1))
    return Lh

def to_csv(to_be_saved, file):
    columns = []
    for key in to_be_saved.keys():
        columns.append(key)
    df = pd.DataFrame(data=to_be_saved)
    df.to_csv(file)

def load_data_set(file):
    try:
        df = pd.read_csv(file, index_col=0)
        data_set = df.to_dict(orient='list')
    except:
        exit("Failed to download set")
        data_set = None

    return data_set

def input_controller():
    # Select controller type
    controller = ControllerDGObs(A, B, Gamma, Pi, kappa, Qr_start, Qh1)
    controller_type = "Cost_observer"
    gen_data = input("Generate data_set? No = 0, Yes = 1.   Please choose: ")
    sim = input("Validate using simulation data? No = 0, Yes = 1.   Please choose: ")
    return controller, controller_type, int(sim), int(gen_data)

def run_simulation(experiment_data):
    # Use the correct settings for the simulation
    inputs = {
        # "simulation_steps": N_exp,
        # "step_size": t_step,
        "time": experiment_data["time"],
        "reference_signal": [],
        "human_weight": Qh1,
        "robot_weight": Qr_start,
        "alpha": alpha,
        "Gamma": Gamma,
        "kappa": kappa,
        "e_initial": [experiment_data["angle_error"][0], experiment_data["rate_error"][0]],
        "u_initial": experiment_data["torque"][0],
        "reference": experiment_data["reference"],
        "initial_state": [experiment_data["steering_angle"][0], experiment_data["steering_rate"][0]],
        "sharing_rule": C,
        "controller_type_name": controller_type,
        "dynamics_type_name": "Steering_dynamics",
        "controller_type": controller_type,
        "dynamics_type": "Steering_dynamics",
        "save": 0,
        "gain_estimation_bias": 0.0,
        "virtual_human_gain": experiment_data["virtual_human_gain"],
        "virtual_human_weight":  experiment_data["virtual_human_cost"]
    }

    controller_sim = NG_Obssim(A, B, mu=0.0, sigma=0.0, nonlin=True)
    simulation_data = controller_sim.simulate(inputs)
    return simulation_data

# This statement is necessary to allow for multiprocessing
if __name__ == "__main__":
    if platform.system() == 'Windows':
        import wres

    # Simulation parameters
    t_warmup = 5
    t_cooldown = 5
    t_exp = 90
    duration = t_warmup + t_cooldown + t_exp
    t_step = 0.015
    N = round(t_exp / t_step)
    N_warmup = round(t_warmup / t_step)
    N_cooldown = round(t_cooldown / t_step)
    N_tot = N + N_warmup + N_cooldown
    fr_min = 1/(20 * np.pi)
    fr_max = 1/(1 * np.pi)
    increments = 20
    reference = Reference(duration)
    screen_width = 1920
    screen_height = 1080

    # Dynamics
    Jw = 0.05480475491037145
    Bw = 0.5  # Max = 0.5
    Kw = 0.0  # Max = 2.5
    A = np.array([[0, 1], [- Kw / Jw, - Bw / Jw]])
    B = np.array([[0], [1 / Jw]])

    # TODO: verify values
    alpha = 30
    Gamma = alpha * np.array([[2.5, 0], [0, 0.125]])
    # Pi = -0.1*np.array([[-1, 0.5], [-1.5, 2]])
    Pi = -4 * np.array([[2, 0], [0, 2]])
    kappa = 0.7
    Qr_end = np.array([[10.0, 0.0], [0.0, 0.1]])
    Qr_start = np.array([[10.0, 0.0], [0.0, 0.1]])

    C = Qr_start

    Qh1 = np.array([[25.0, 0.0], [0.0, 0.2]])
    Qh2 = np.array([[12.0, 0.0], [0.0, 0.1]])

    vhg = np.zeros((6, 2))
    vhg[0, :] = compute_virtual_gain(Qh2, Qr_end, A, B)
    vhg[2, :] = compute_virtual_gain(Qh1, Qr_end, A, B)
    vhg[4, :] = -compute_virtual_gain(Qh2, Qr_end, A, B)
    virtual_human_gain = vhg
    Qh = np.zeros((6, 2))
    Qh[0, :] = np.array([Qh2[0, 0], Qh2[1, 1]])
    Qh[2, :] = np.array([Qh1[0, 0], Qh1[1, 1]])
    Qh[4, :] = - np.array([Qh2[0, 0], Qh2[1, 1]])
    virtual_human_weight = Qh

    # Ask for input
    controller, controller_type, sim, gen_dat = input_controller()
    virt = input("Do you want to use a virtual human being? 0. No, 1. Yes. Your answer = ")
    if int(virt) == 0:
        virtual_human = False
    else:
        virtual_human = True

    # Check if we're generating data
    if gen_dat == 1:
        # Let's get cracking and get some data
        # Start the senso drive parallel process
        parent_conn, child_conn = mp.Pipe(True)
        senso_dict = {
            "stiffness": Kw,
            "damping": Bw,
            "alpha_1": A[1, 0],
            "alpha_2": A[1, 1],
            "beta": B[1, 0],
            "controller": controller,
            "controller_type": controller_type,
            "parent_conn": parent_conn,
            "child_conn": child_conn
        }
        senso_drive_process = SensoDriveModule(senso_dict)
        senso_drive_process.start()
        print("process started!")

        # Time to do an experiment!
        full_screen = True
        preview = True
        experiment_input = {
            "damping": Bw,
            "stiffness": Kw,
            "reference": reference,
            "controller_type": controller_type,
            "parent_conn": parent_conn,
            "child_conn": child_conn,
            "senso_drive": senso_drive_process,
            "screen_width": screen_width,
            "screen_height": screen_height,
            "full_screen": full_screen,
            "preview": preview,
            "warm_up_time": t_warmup,
            "experiment_time": t_exp,
            "cooldown_time": t_cooldown,
            "virtual_human": virtual_human,
            "virtual_human_gain": virtual_human_gain,
            "virtual_human_cost": Qh,
            "init_robot_cost": Qr_start,
            "final_robot_cost": Qr_end,
            "sharing_rule": C,
        }
        experiment_handler = Experiment(experiment_input)
        if platform.system() == 'Windows':
            with wres.set_resolution(10000):
                experiment_data = experiment_handler.experiment()

                # Save data
                if virtual_human:
                    string = "data_virtual_human.csv"
                else:
                    string = "data_real_human.csv"
                to_csv(experiment_data, string)

        senso_drive_process.join(timeout=0)

    # Retrieve data
    try:
        virtual_data = load_data_set("data_virtual_human.csv")
        real_data = load_data_set("data_real_human.csv")
    except:
        exit("Missing datasets, first create these")

    sim_data = run_simulation(virtual_data)

    # Analyse stuff
    plot_stuff = PlotStuff()
    plot_stuff.plot(virtual_data, real_data, sim_data)

