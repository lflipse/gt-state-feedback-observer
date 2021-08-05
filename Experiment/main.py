import numpy as np
import multiprocessing as mp
import time
import wres
import platform
import scipy.linalg as cp
import pandas as pd
import random

from Experiment.reference_trajectory import Reference
from Controller_Design.live_plotter import LivePlotter
from Controller_Design.SensoDrive.SensoDriveMultiprocessing import SensoDriveModule
from Experiment.experiment import Experiment
from Controller_Design.Controllers.Linear_Quadratic import ControllerLQ
from Controller_Design.Controllers.Differential_Game_Gain_Observer import ControllerDG_GObsKal
from Experiment.plots import PlotStuff


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

def compute_virtual_gain(Qh1, Qr_end, A, B):
    # Iterative procedure for calculating gains
    Lh = np.matmul(B.transpose(), cp.solve_continuous_are(A, B, Qh1, 1))
    Lr = np.matmul(B.transpose(), cp.solve_continuous_are(A, B, Qr_end, 1))
    for i in range(10):
        Lh = np.matmul(B.transpose(), cp.solve_continuous_are(A - B * Lr, B, Qh1, 1))
        Lr = np.matmul(B.transpose(), cp.solve_continuous_are(A - B * Lh, B, Qr_end, 1))
    return Lh

def input_controller():
    # Select controller type
    print("Choose a controller type. 0: Differential Game Cost Observer, 1: Use a dataset")
    exp_type = input("Please choose: ")
    if int(exp_type) == 0:
        controller = ControllerDG_GObsKal(A, B, Gamma, Pi, kappa, C, None)
        controller_type = "Cost_observer"
    elif int(exp_type) == 1:
        controller = None
        controller_type = "Data_set"
    else:
        print("You chose: ", int(exp_type))
        exit("That's no option, try again.")

    return controller, controller_type, exp_type

# This statement is necessary to allow for multiprocessing
if __name__ == "__main__":
    if platform.system() == 'Windows':
        import wres

    # Simulation parameters
    t_warmup = 5
    t_cooldown = 5
    t_exp = 60
    duration = t_warmup + t_cooldown + t_exp
    t_step = 0.015
    N = round(t_exp / t_step)
    N_warmup = round(t_warmup / t_step)
    N_cooldown = round(t_cooldown / t_step)
    N_tot = N + N_warmup + N_cooldown
    T = np.array(range(N)) * (t_step + 0.001)
    T_ex = np.array(range(N+N_warmup+N_cooldown)) * t_step
    fr_min = 1/(20 * np.pi)
    fr_max = 1/(1 * np.pi)
    increments = 20
    reference = Reference(duration)

    # Dynamics
    Jw = 0.05480475491037145
    Bw = 0.3  # Max = 0.5
    Kw = 0.0  # Max = 2.5
    A = np.array([[0, 1], [- Kw / Jw, - Bw / Jw]])
    B = np.array([[0], [1 / Jw]])

    # TODO: verify values
    alpha = 5
    Gamma = alpha * np.array([[2.5, 0], [0, 0.0]])
    # Pi = -0.1*np.array([[-1, 0.5], [-1.5, 2]])
    Pi = 4 * np.array([[2, 0], [0, 2]])
    kappa = 0.7
    C = np.array([[20.0, 0.0], [0.0, 0.2]])
    conditions_experiment = [3]
    sigma_h = 0.1 * np.array([0.0, 0.0, 0.0, 1, 1, 0.0])
    roles = ["", "", "Leader", "Follower", "Follower", "Leader"]
    # random.shuffle(conditions_experiment)

    print("Observer dynamics", A - Pi)

    screen_width = 1920
    screen_height = 1080
    # screen_width = 720
    # screen_height = 480

    # ask input
    controller, controller_type, exp_type = input_controller()

    if controller_type != "Data_set":

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

        # Start the data plotting parallel process
        # send_conn, receive_conn = mp.Pipe(True)
        # live_plotter_process = LivePlotter(receive_conn)
        # live_plotter_process.start()
        # print("Second process started!")

        # Time to do an experiment!
        full_screen = True
        preview = True
        do_exp = True

        if do_exp == True:
            experiment_input = {
                "damping": Bw,
                "stiffness": Kw,
                "reference": reference,
                "controller_type": controller_type,
                # "send_conn": send_conn,
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
                "virtual_human": False,
                "virtual_human_gain": None,
                "virtual_human_cost": None,
                "init_robot_cost": None,
                "final_robot_cost": None,
                "sharing_rule": C,
                "conditions": conditions_experiment,
                "human_noise": sigma_h,
                "roles": roles,
            }
            experiment_handler = Experiment(experiment_input)
            if platform.system() == 'Windows':
                with wres.set_resolution(10000):
                    experiment_data = experiment_handler.experiment()

                    # Also save data
                    save = input("save data?")
                    if save == "yes":
                        to_csv(experiment_data, "data_set.csv")

            senso_drive_process.join(timeout=0)
            # live_plotter_process.join(timeout=0)

    else:
        # Load data set
        experiment_data = load_data_set("data_set.csv")

    # Plot stuff
    plot_stuff = PlotStuff()
    plot_stuff.plot(experiment_data)

