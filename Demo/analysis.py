import sys
import os
sys.path.insert(1, '..')
import pandas as pd
import numpy as np
from Demo.plots import PlotStuff
import matplotlib.pyplot as plt


class Analysis():
    def __init__(self):
        # Unpack data
        self.raw_data = {}
        self.filtered_data = {}
        self.average_data = {}
        self.metrics = {}
        self.metrics_individuals = {}
        self.plot_stuff = None
        self.trials = 0
        self.participants = 0
        self.periods = 4
        self.conditions = 4

    def initialize(self):
        self.unpack_data()
        self.plot_stuff = PlotStuff()

    def unpack_data(self):
        path = "data"
        # path = "first_trial"
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
        self.build_individual_metrics()
        self.plot_stuff.plot_trial(self.raw_data[self.participants-1, 0])
        self.plot_stuff.plot_trial(self.raw_data[self.participants-1, 3])
        self.plot_stuff.plot_trial(self.raw_data[self.participants-1, 4])

        # General experiment data
        self.plot_stuff.plot_experiment(self.metrics, self.metrics_individuals, False)
        plt.show()

    def build_metrics(self):
        # Check metrics per participant: sorted per (participant, condition)
        # RMSE (performance), RMSU (effort), Gains

        # Pre-define metrics
        self.metrics["rms_angle_error"] = {}
        self.metrics["rms_rate_error"] = {}
        self.metrics["rms_human_torque"] = {}
        self.metrics["rms_robot_torque"] = {}
        self.metrics["human_angle_cost"] = {}
        self.metrics["robot_angle_cost"] = {}
        self.metrics["condition"] = {}
        self.metrics["repetition"] = {}

        # RMS
        rms_angle_error = []
        rms_rate_error = []
        rms_human_torque = []
        rms_robot_torque = []

        # Costs
        human_angle_cost = []
        robot_angle_cost = []

        # Info
        repetitions = []
        conditions = []
        participant = []
        settings = []

        for i in range(self.participants):
            for j in range(self.trials):
                rep = self.filtered_data[i, j]["repetition"]
                repetition = rep[10]  # Not very nicely done this

                # Omit the first trial per condition
                cond = self.filtered_data[i, j]["condition"]
                condition = cond[10]  # Not very nicely done this
                conditions.append(condition)
                repetitions.append(repetition)
                participant.append(i)
                set = self.filtered_data[i, j]["setting"]
                setting = set[10]  # Not very nicely done this
                settings.append(setting)

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

                # Average cost functions
                human_angle_cost.append(np.mean(self.filtered_data[i, j]["estimated_human_cost_1"]))
                robot_angle_cost.append(np.mean(self.filtered_data[i, j]["robot_cost_pos"]))


        # Save to metrics dictionary
        self.metrics["rms_angle_error"] = rms_angle_error
        self.metrics["rms_rate_error"] = rms_rate_error
        self.metrics["rms_human_torque"] = rms_human_torque
        self.metrics["rms_robot_torque"] = rms_robot_torque
        self.metrics["human_angle_cost"] = human_angle_cost
        self.metrics["robot_angle_cost"] = robot_angle_cost
        self.metrics["repetition"] = repetitions
        self.metrics["settings"] = settings
        self.metrics["condition"] = conditions
        self.metrics["participant"] = participant
        # print(self.metrics)

    def build_individual_metrics(self):
        conditions = []
        settings = []
        participants = []
        rms_angle_error = []
        rms_rate_error = []
        human_angle_cost = []
        robot_angle_cost = []

        metrics = pd.DataFrame.from_dict(self.metrics)
        for i in range(self.participants):
            participant = metrics.loc[metrics['participant'] == i]
            manual_control = participant.loc[metrics['condition'] == "Manual Control"]
            shared_control = participant.loc[metrics['condition'] == "Shared Control"]
            MC_GV = manual_control.loc[metrics['settings'] == "Good Visuals"]
            MC_BV = manual_control.loc[metrics['settings'] == "Bad Visuals"]
            SC_GV = shared_control.loc[metrics['settings'] == "Good Visuals"]
            SC_BV = shared_control.loc[metrics['settings'] == "Bad Visuals"]

            participants.append([i, i, i, i])
            conditions.append(["Manual Control", "Manual Control", "Shared Control", "Shared Control"])
            settings.append(["Good Visuals", "Bad Visuals", "Good Visuals", "Bad Visuals"])
            rms_angle_error.append([MC_GV["rms_angle_error"].mean(), MC_BV["rms_angle_error"].mean(),
                               SC_GV["rms_angle_error"].mean(), SC_BV["rms_angle_error"].mean()])
            rms_rate_error.append([MC_GV["rms_rate_error"].mean(), MC_BV["rms_rate_error"].mean(),
                               SC_GV["rms_rate_error"].mean(), SC_BV["rms_rate_error"].mean()])
            human_angle_cost.append([MC_GV["human_angle_cost"].mean(), MC_BV["human_angle_cost"].mean(),
                               SC_GV["human_angle_cost"].mean(), SC_BV["human_angle_cost"].mean()])
            robot_angle_cost.append([MC_GV["robot_angle_cost"].mean(), MC_BV["robot_angle_cost"].mean(),
                              SC_GV["robot_angle_cost"].mean(), SC_BV["robot_angle_cost"].mean()])

        self.metrics_individuals["participants"] = [item for sublist in participants for item in sublist]
        self.metrics_individuals["conditions"] = [item for sublist in conditions for item in sublist]
        self.metrics_individuals["settings"] = [item for sublist in settings for item in sublist]
        self.metrics_individuals["rms_angle_error"] = [item for sublist in rms_angle_error for item in sublist]
        self.metrics_individuals["rms_rate_error"] = [item for sublist in rms_rate_error for item in sublist]
        self.metrics_individuals["human_angle_cost"] = [item for sublist in human_angle_cost for item in sublist]
        self.metrics_individuals["robot_angle_cost"] = [item for sublist in robot_angle_cost for item in sublist]


    def cut_data(self, participant, trial):
        # Cut data
        filtered_data = {}
        for key in self.raw_data[participant, trial].keys():
            data = self.raw_data[participant, trial][key]
            time = np.array(self.raw_data[participant, trial]['time'])
            start_time = 0.2 * time[-1]
            end_time = 0.8 * time[-1]
            # Find index where to cut the data
            # print(start_time, end_time)
            start_index = np.argmax(time > start_time)
            end_index = np.argmax(time > end_time)
            filtered_data[key] = data[start_index:end_index]

        self.filtered_data[participant, trial] = filtered_data
