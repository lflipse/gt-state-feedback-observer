import numpy as np
import multiprocessing as mp
import time
import wres
import platform
import scipy.linalg as cp
import pandas as pd
import random
import os

from Experiment.reference_trajectory import Reference
from Controller_Design.SensoDrive.SensoDriveMultiprocessing import SensoDriveModule
from Experiment.experiment import Experiment
from Controller_Design.Controllers.Differential_Game_Gain_Observer import ControllerDGObs
from Experiment.visuals import Visualize
from Demo.analysis import Analysis

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

def choose_condition():
    print("Choose a controller type")
    a = int(input("0: Manual, 1: Haptic static, 2: Haptic adaptive.  Your answer = "))
    print("Choose a visual condition")
    b = int(input("0: Good, 1: Bad, 2: Mixed.  Your answer = "))
    if b == 0 or b == 1:
        condition = a * 2 + b
    elif b == 2:
        if a == 2:
            condition = -1
        else:
            exit("No option")
    else:
        exit("No option")

    c = int(input("Warmstart? 0: No, Yes: 1.  Your answer = "))
    if c == 0:
        initial_human = np.array([0, 0])
    elif c == 1:
        initial_human = np.matmul(B.transpose(), cp.solve_continuous_are(A, B, C, 1))[0]
        print(initial_human)
    else:
        exit("No option")

    return condition, initial_human * 0.75


# This statement is necessary to allow for multiprocessing
if __name__ == "__main__":
    if platform.system() == 'Windows':
        import wres

    # Simulation parameters
    reference = Reference(duration=None)

    # Dynamics
    Jw = 0.04914830792783059
    Bw = 0.3  # Max = 0.5
    Kw = 0.0  # Max = 2.5
    A = np.array([[0, 1], [- Kw / Jw, - Bw / Jw]])
    B = np.array([[0], [1 / Jw]])

    # TODO: verify values
    Gamma = 4 * np.array([[2, 0], [0, 2]])
    alpha = 1.5
    K = alpha * np.array([[10.0, 0], [0, 0.0]])
    kappa = 1
    C = np.array([[7.5, 0.0], [0.0, 0.2]])

    # Experiment data
    t_warmup = 5
    t_cooldown = 5
    t_period = 60
    sigma = [0.005, 0.035]
    periods = 1
    t_exp = periods * t_period
    t_prev = 1.2
    repetitions = 1
    visual_conditions = 2
    haptic_conditions = 3
    robot_conditions = 1
    conditions = repetitions * visual_conditions * haptic_conditions + robot_conditions
    duration = t_warmup + t_cooldown + t_exp

    # Choose a condition
    condition, initial_human = choose_condition()

    # Visual stuff
    screen_width = 1920
    screen_height = 1080

    # Insert controller
    controller = ControllerDGObs(A, B, K, Gamma, kappa)
    controller_type = "Cost_observer"

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
        "child_conn": child_conn,
        "initial_human": initial_human,
    }
    senso_drive_process = SensoDriveModule(senso_dict)

    # Time to do an experiment!
    full_screen = True
    preview = True
    do_exp = True

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
        "preview_time": t_prev,
        "warm_up_time": t_warmup,
        "experiment_time": t_exp,
        "cooldown_time": t_cooldown,
        "period_time": t_period,
        "virtual_human": False,
        "virtual_human_gain": None,
        "virtual_human_cost": None,
        "init_robot_cost": None,
        "final_robot_cost": None,
        "sharing_rule": C,
        "periods": periods,
        "repetitions": repetitions,
        "visual_conditions": visual_conditions,
        "sigma": sigma,
    }

    # Make folder for the participant
    try:
        dirlist = os.listdir("data")
        new_list = [int(i) for i in dirlist]
        print("last folder: ", max(new_list))
        print("folders: ", new_list)
        participant = max(new_list) + 1
    except:
        participant = 0

    print("now making folder for participant: ", participant)
    folder_name = "data\\" + str(participant)
    os.mkdir(folder_name)
    senso_drive_process.start()
    print("process started!")

    # Initialize pygame visualization
    visualize = Visualize(screen_width, screen_height, full_screen)
    experiment_handler = Experiment(experiment_input, visualize)

    conditions = 1

    for i in range(conditions):
        if platform.system() == 'Windows':
            with wres.set_resolution(10000):
                # Do trial (trial -1 is the robot only run)
                experiment_data = experiment_handler.experiment(condition=condition)

                # Save data
                string = "data\\" + str(participant) + "\\trial_" + str(i) + ".csv"
                to_csv(experiment_data, string)

    visualize.quit()
    senso_drive_process.join(timeout=0)
    # live_plotter_process.join(timeout=0)

    analysis = Analysis()
    analysis.initialize()
    analysis.analyse()


