import numpy as np
import multiprocessing as mp
import wres
import platform
import pandas as pd
import os
import sys

sys.path.insert(1, '..')
from Experiment.reference_trajectory import Reference
from Controller_Design.SensoDrive.SensoDriveMultiprocessing import SensoDriveModule
from Experiment.experiment import Experiment
from Controller_Design.Controllers.Linear_Quadratic import ControllerLQ
from Experiment.visuals import Visualize

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

    # Experiment data
    t_warmup = 0.1
    t_cooldown = 0.1
    t_period = 30
    sigma = [0.005, 0.065]
    periods = 1
    t_exp = periods * t_period
    t_prev = 0.1
    repetitions = 1
    visual_conditions = 1
    haptic_conditions = 1
    robot_conditions = 1
    conditions = 10
    duration = t_warmup + t_cooldown + t_exp
    trials = conditions
    q = np.logspace(-1, 3, num=conditions)
    print("Checking following costs: ", q)

    # Visual stuff
    screen_width = 1920
    screen_height = 1080

    cond = "Linear Quadratic"
    i = 1
    Q = np.array([[q[i], 0], [0, 0.1]])

    # Insert controller
    controller = ControllerLQ(A, B, Q)
    controller_type = "Linear Quadratic"
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
    senso_drive_process.start()

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
        "sharing_rule": None,
        "periods": periods,
        "trials": trials,
        "repetitions": repetitions,
        "visual_conditions": visual_conditions,
        "sigma": sigma,
    }

    # Initialize pygame visualization
    visualize = Visualize(screen_width, screen_height, full_screen)

    experiment_input["sharing_rule"] = Q
    experiment_handler = Experiment(experiment_input, visualize)

    for i in range(conditions):


        if platform.system() == 'Windows':
            with wres.set_resolution(10000):
                # Do trial (trial -1 is the robot only run)
                ready, experiment_data = experiment_handler.experiment(condition=cond, repetition=0, trial=i)

                # Save data
                string = "data_robot\\trial_" + str(i) + ".csv"
                to_csv(experiment_data, string)

        visualize.quit()
        senso_drive_process.join(timeout=0)

    # live_plotter_process.join(timeout=0)


