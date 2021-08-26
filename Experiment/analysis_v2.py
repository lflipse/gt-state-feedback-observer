import sys
import os
sys.path.insert(1, '..')
import pandas as pd
import numpy as np
from Experiment.plots import PlotStuff
import matplotlib.pyplot as plt


class Analysis():
    def __init__(self):
        # Unpack data
        self.raw_data = {}
        self.filtered_data = {}
        self.metrics = {}
        self.plot_stuff = None
        self.trials = 0
        self.participants = 0
        self.conditions = 6 # TODO: softcode

    def initialize(self):
        self.unpack_data()
        self.plot_stuff = PlotStuff()

    def unpack_data(self):
        path = "data"
        list_dir = os.listdir(path)
        self.participants = len(list_dir)
        for i in range(self.participants):
            path_participant = path + "\\" + list_dir[i]
            participant = int(list_dir[i])
            list_dir_par = os.listdir(path_participant)
            self.trials = len(list_dir_par)
            for j in range(self.trials):
                path_trial = path_participant + "\\" + list_dir_par[j]
                try:
                    df = pd.read_csv(path_trial, index_col=0)
                    self.raw_data[participant, j] = df.to_dict(orient='list')
                    print("loaded ", path_trial)
                except:
                    exit("Something went wrong")

    def analyse(self):

        # First, build some metrics
        print(self.participants)
        for i in range(self.participants):
            for j in range(self.trials):
                self.cut_data(i, j)

        self.build_metrics()

        # Some some individual data
        # self.plot_stuff.plot_trial(self.raw_data[0, 2])
        # self.plot_stuff.plot_trial(self.raw_data[1, 1])

        # General experiment data
        self.plot_stuff.plot_experiment(self.metrics, True)
        plt.show()

    def build_metrics(self):
        # Check metrics per participant: sorted per (participant, condition)
        # RMSE (performance), RMSU (effort), Gains

        # Pre-define metrics
        self.metrics["rmse"] = {}
        self.metrics["rmsu"] = {}
        self.metrics["cost"] = {}
        self.metrics["rmse_index"] = {}
        self.metrics["rmsu_index"] = {}
        self.metrics["cost_index"] = {}

        # RMS
        rms_angle_error = []
        rms_rate_error = []
        rms_human_torque = []
        rms_robot_torque = []

        # Costs
        human_angle_cost = []
        robot_angle_cost = []

        # Info
        conditions = []
        participant = []
        condition_name = []
        condition_names = ["", "C1: Implicit Leader", "C2: Explicit Inconsistent Follower", "C3: Explicit Consistent Leader",
                           "C4: Implicit Follower", "C5: Explicit Consistent Follower", "C6: Explicit Inconsistent Leader"]

        for i in range(self.participants):
            for j in range(self.trials):
                # Find condition
                cond = self.filtered_data[i, j]["condition"]
                condition = cond[10]  # Not very nicely done this
                conditions.append(condition)
                condition_name.append(condition_names[condition])
                participant.append(i)

                # RMSE
                angle_error = self.filtered_data[i, j]["angle_error"]
                rate_error = self.filtered_data[i, j]["rate_error"]
                rms_angle_error.append(np.sqrt(1 / (len(angle_error)) * np.inner(angle_error, angle_error)))
                rms_rate_error.append(np.sqrt(1 / (len(rate_error)) * np.inner(rate_error, rate_error)))

                # RMSU
                human_torque = self.filtered_data[i, j]["estimated_human_input"]
                robot_torque = self.filtered_data[i, j]["torque"]
                rms_human_torque.append(np.sqrt(1 / (len(rate_error)) * np.inner(human_torque, human_torque)))
                rms_robot_torque.append(np.sqrt(1 / (len(rate_error)) * np.inner(robot_torque, robot_torque)))

                # Average gains
                human_angle_cost.append(np.mean(self.filtered_data[i, j]["estimated_human_cost_1"]))
                robot_angle_cost.append(np.mean(self.filtered_data[i, j]["robot_cost_pos"]))

        # Save to metrics dictionary
        self.metrics["condition"] = np.append(conditions, conditions)
        self.metrics["participant"] = np.append(participant, participant)
        self.metrics["rmse"] = np.append(rms_angle_error, rms_rate_error)
        self.metrics["rmse_index"] = np.append(np.tile("Angle error", self.trials * self.participants),
                                               np.tile("Rate error", self.trials * self.participants))
        self.metrics["rmsu"] = np.append(rms_human_torque, rms_rate_error)
        self.metrics["rmsu_index"] = np.append(np.tile("Human torque", self.trials * self.participants),
                                               np.tile("Robot torque", self.trials * self.participants))
        self.metrics["cost"] = np.append(human_angle_cost, robot_angle_cost)
        self.metrics["cost_index"] = np.append(np.tile("Human cost", self.trials * self.participants),
                                               np.tile("Robot cost", self.trials * self.participants))



    def cut_data(self, participant, trial):
        # Find start and end indices
        # print(self.raw_data[participant, trial]["condition"])
        time = np.array(self.raw_data[participant, trial]['time'])
        start_time = 0.25 * time[-1]
        end_time = 0.75 * time[-1]
        # Find index where to cut the data
        # print(start_time, end_time)
        start_index = np.argmax(time > start_time)
        end_index = np.argmax(time > end_time)

        # Cut the data from the trial
        filtered_data = {}
        for key in self.raw_data[participant, trial].keys():
            data = self.raw_data[participant, trial][key]
            filtered_data[key] = data[start_index:end_index]

        self.filtered_data[participant, trial] = filtered_data


# analysis = Analysis()
# analysis.initialize()
# analysis.analyse()
