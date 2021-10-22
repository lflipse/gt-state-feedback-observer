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

def init_experiment():
    dirlist = os.listdir("data")
    new_list = [int(i) for i in dirlist]
    try:
        last_participant = max(new_list)
    except:
        last_participant = -1
    print("last folder: ", last_participant)
    print("folders: ", new_list)


    print("Choose option:")
    option = int(input("0: Do experiment, 1: Resume experiment.   "))
    if option == 1:
        participant = int(input("input participant number:  "))
        trial_number = int(input("input trial number:  "))

        if trial_number > trials:
            exit("trial number exceeds possible trials")

        if participant > last_participant:
            exit("Participant number does not exist")

    else:
        participant = last_participant + 1
        trial_number = -1

    return participant, trial_number


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
    alpha = 10.0
    K = alpha * np.array([[10.0, 0], [0, 0.0]])
    kappa = 1
    C = np.array([[80.0, 0.0], [0.0, 0.1]])

    # Experiment data
    t_warmup = 5
    t_cooldown = 5
    t_period = 77.5
    sigma = [0.005, 0.065]
    periods = 1
    t_exp = periods * t_period
    t_prev = 1.2
    repetitions = 4
    haptic_conditions = 3
    trials = repetitions * haptic_conditions
    duration = t_warmup + t_cooldown + t_exp
    latin_square = [["Manual Control", "Positive Reinforcement", "Negative Reinforcement"],
                    ["Manual Control", "Negative Reinforcement", "Positive Reinforcement"]]

    # Choose a condition
    participant, trial_number = init_experiment()

    # Visual stuff
    screen_width = 1920
    screen_height = 1080

    # Insert controller
    controller = ControllerDGObs(A, B, K, Gamma, kappa)
    controller_type = "Cost_observer"
    initial_human = np.array([0, 0])

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

    order_nr = participant % 2
    order = latin_square[order_nr]
    print(participant, order_nr, order)

    print("now making folder for participant: ", participant)
    folder_name = "data\\" + str(participant)

    # Check if folder already exists
    if not os.path.isdir(folder_name):
        os.mkdir(folder_name)

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
        "trials": trials,
        "sigma": [0.0],
    }


    senso_drive_process.start()
    print("process started!")

    # Initialize pygame visualization
    visualize = Visualize(screen_width, screen_height, full_screen)
    experiment_handler = Experiment(experiment_input, visualize)
    condition_range = range(trial_number, trials)
    if trial_number == -1:
        ready = False

    for i in condition_range:
        # When not ready, do another practice trial
        if i == -1:
            condition = "Practice"
            repetition = random.randint(0, 3)
            condition_number = -1
        else:
            repetition = i % repetitions
            condition_number = int((i - repetition) / haptic_conditions)
            condition = order[condition_number]
        print("Trial: ", i, " Repetition: ", repetition, " Condition nr.: ", condition_number)

        if platform.system() == 'Windows':
            with wres.set_resolution(10000):
                # Do trial (trial -1 is the robot only run)
                if i == -1:
                    while not ready:
                        ready, experiment_data = experiment_handler.experiment(condition=condition,
                                                                               repetition=repetition, trial=i)
                else:
                    ready, experiment_data = experiment_handler.experiment(condition=condition, repetition=repetition,
                                                                           trial=i)

                    # Save data
                    string = "data\\" + str(participant) + "\\trial_" + str(i) + ".csv"
                    to_csv(experiment_data, string)

    visualize.quit()
    senso_drive_process.join(timeout=0)

    # analysis = Analysis()
    # analysis.initialize()
    # analysis.analyse()


