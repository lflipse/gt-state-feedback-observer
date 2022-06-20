import numpy as np
import multiprocessing as mp
import time
import wres
import platform
import scipy.linalg as cp
import pandas as pd
import random
import os
import sys

from reference_trajectory import Reference
from SensoDrive.SensoDriveMultiprocessing import SensoDriveModule
from experiment import Experiment
from Differential_Game_Gain_Observer import ControllerDGObs
from visuals import Visualize
from analysis import Analysis
from PyQt5 import QtWidgets
from PyQt5.QtGui import *
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QCoreApplication

class MyWindow(QMainWindow):
    def __init__(self, optionHandler):
        super(MyWindow, self).__init__()
        self.initUI()
        self.setGeometry(100, 100, 600, 400)
        self.setWindowTitle("Demonstrator")
        self.option = -2
        self.optionHandler = optionHandler

    def initUI(self):
        self.label = QtWidgets.QLabel(self)
        self.label.setFont(QFont("Arial", 18))
        self.label.setText("Please choose one of \n the following demos")
        self.label.setGeometry(200, 50, 100, 40)
        self.label.adjustSize()

        self.button1 = QtWidgets.QPushButton(self)
        self.button1.setFont(QFont("Arial", 18))
        self.button1.setText("Mini experiment")
        self.button1.move(25, 150)
        self.button1.clicked.connect(self.clicked_4)
        self.button1.adjustSize()

        self.button2 = QtWidgets.QPushButton(self)
        self.button2.setFont(QFont("Arial", 18))
        self.button2.setText("Manual Control")
        self.button2.move(300, 150)
        self.button2.clicked.connect(self.clicked_1)
        self.button2.adjustSize()

        self.button3 = QtWidgets.QPushButton(self)
        self.button3.setFont(QFont("Arial", 18))
        self.button3.setText("Positive Reinforcement")
        self.button3.move(25, 300)
        self.button3.clicked.connect(self.clicked_2)
        self.button3.adjustSize()

        self.button4 = QtWidgets.QPushButton(self)
        self.button4.setFont(QFont("Arial", 18))
        self.button4.setText("Negative Reinforcement")
        self.button4.move(300, 300)
        self.button4.clicked.connect(self.clicked_3)
        self.button4.adjustSize()

    def clicked_1(self):
        self.option = 0
        self.optionHandler.update_option(self.option)
        run_demo()
        self.close()

    def clicked_2(self):
        self.option = 1
        self.optionHandler.update_option(self.option)
        run_demo()
        self.close()

    def clicked_3(self):
        self.option = 2
        self.optionHandler.update_option(self.option)
        run_demo()
        self.close()

    def clicked_4(self):
        self.option = -1
        self.optionHandler.update_option(self.option)
        run_demo()
        self.close()

    def closeEvent(self, *args, **kwargs):
        super(MyWindow, self).closeEvent(*args, **kwargs)

class optionHandler:
    def __init__(self):
        self.option = -2

    def update_option(self, option):
        self.option = option

    def return_option(self):
        return self.option

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

def choose_demo(option_handler):
    print("Choose an experimental condition")
    app = QApplication(sys.argv)
    win = MyWindow(option_handler)
    win.show()
    sys.exit(app.exec_())

def run_demo():
    # Simulation parameters
    reference = Reference(duration=None)

    # Dynamics
    Jw = 0.04914830792783059
    Bw = 0.3  # Max = 0.5
    Kw = 0.0  # Max = 2.5
    A = np.array([[0, 1], [- Kw / Jw, - Bw / Jw]])
    B = np.array([[0], [1 / Jw]])

    # Controller settings
    Gamma = 4 * np.array([[2, 0], [0, 2]])
    alpha = 2.5
    K = alpha * np.array([[10.0, 0], [0, 0.0]])
    kappa = 1
    C = np.array([[15, 0.0], [0.0, 0.1]])

    # Experiment data
    t_warmup = 5
    t_cooldown = 5
    t_period = 40
    sigma = [0.005, 0.065]
    periods = 1
    t_exp = periods * t_period
    t_prev = 0.1
    repetitions = 1
    visual_conditions = 1
    haptic_conditions = 3
    robot_conditions = 1
    conditions = repetitions * visual_conditions * haptic_conditions + robot_conditions
    conditions = 3
    duration = t_warmup + t_cooldown + t_exp

    # Choose a condition

    condition = option_handler.return_option()
    if condition < 0:
        trials = conditions
    else:
        trials = 1

    # Visual stuff
    # automatically find correct height and width
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
        "trials": trials,
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

    # conditions = 3
    if condition >= 0:
        conditions = 1

    condits = ["Manual Control", "Positive Reinforcement", "Negative Reinforcement", "Mixed Reinforcement"]

    for i in range(conditions):
        if condition < 0:
            cond = condits[i]
        else:
            cond = condits[condition]

        if platform.system() == 'Windows':
            with wres.set_resolution(10000):
                # Do trial (trial -1 is the robot only run)
                ready, experiment_data = experiment_handler.experiment(condition=cond, repetition=0, trial=i)

                # Save data
                string = "data\\" + str(participant) + "\\trial_" + str(i) + ".csv"
                to_csv(experiment_data, string)

    visualize.quit()
    senso_drive_process.join(timeout=0)
    # live_plotter_process.join(timeout=0)

    analysis = Analysis()
    analysis.initialize()
    analysis.analyse()

# This statement is necessary to allow for multiprocessing
if __name__ == "__main__":
    if platform.system() == 'Windows':
        import wres

    option_handler = optionHandler()
    choose_demo(option_handler)
