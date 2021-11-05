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
        self.average_data = {}
        self.metrics = {}
        self.metrics_individuals = {}
        self.plot_stuff = None
        self.trials = 16
        self.participants = 0
        self.periods = 4
        self.conditions = 4

    def initialize(self):
        self.unpack_data()
        self.plot_stuff = PlotStuff()

    def unpack_data(self):
        path = "..\\Demo\\data"
        # path = "first_trial"
        list_dir = os.listdir(path)
        self.participants = len(list_dir)
        for i in range(self.participants):
            path_participant = path + "\\" + list_dir[i]
            participant = int(list_dir[i])
            self.raw_data[participant] = {}
            list_dir_par = os.listdir(path_participant)
            self.trials = len(list_dir_par)

            for j in range(self.trials):
                path_trial = path_participant + "\\" + list_dir_par[j]
                try:
                    df = pd.read_csv(path_trial, index_col=0)
                    self.raw_data[participant][j] = df.to_dict(orient='list')
                    print("loaded ", path_trial)
                except:
                    exit("Something went wrong")

    def analyse(self):
        # First, build some metrics
        for i in range(self.participants):
            for j in range(self.trials):
                self.cut_data(i, j)

        self.build_metrics()
        self.build_individual_metrics()

        # Plot individual data
        self.plot_stuff.plot_participant(self.raw_data, trials=self.trials, participant=self.participants - 1)
        # self.plot_stuff.plot_data(self.raw_data, trials=self.trials, participant=self.participants - 2)

        # Plot metrics
        # self.plot_stuff.plot_experiment(self.metrics, self.metrics_individuals, self.metrics_individuals, self.participants, False)
        plt.show()

    def build_metrics(self):
        # Check metrics per participant: sorted per (participant, condition)
        # RMSE (performance), RMSU (effort), Gains

        # Pre-define metrics
        self.metrics["rms_angle_error"] = {}
        self.metrics["rms_rate_error"] = {}
        self.metrics["rms_human_torque"] = {}
        self.metrics["rms_robot_torque"] = {}
        self.metrics["rms_human_power"] = {}
        self.metrics["rms_robot_power"] = {}
        self.metrics["human_angle_cost"] = {}
        self.metrics["robot_angle_cost"] = {}
        self.metrics["human_angle_gain"] = {}
        self.metrics["robot_angle_gain"] = {}
        self.metrics["system_angle_cost"] = {}
        self.metrics["system_angle_gain"] = {}
        self.metrics["authority"] = {}
        self.metrics["condition"] = {}
        self.metrics["condition_number"] = {}
        self.metrics["repetition"] = {}
        self.metrics["gain_variability"] = {}


        # RMS
        rms_angle_error = []
        rms_rate_error = []
        rms_estimated_human_torque = []
        rms_human_torque = []
        rms_robot_torque = []
        rms_human_power = []
        rms_robot_power = []

        # Costs
        human_angle_cost = []
        robot_angle_cost = []
        system_angle_cost = []

        # Gains
        human_angle_gain = []
        robot_angle_gain = []
        system_angle_gain = []

        # Robot
        robot_rms = []
        robot_gains = []

        # Co-adaptation
        authority = []
        gain_variability = []

        conflicts = []

        # Info
        repetitions = []
        conditions = []
        condition_numbers = []
        participant = []
        settings = []


        for i in range(self.participants):
            for j in range(self.trials):
                rep = self.filtered_data[i][j]["repetition"]
                repetition = rep[10]  # Not very nicely done this

                # Omit the first trial per condition
                cond = self.filtered_data[i][j]["condition"]
                condition = cond[10]  # Not very nicely done this
                if condition == "Manual Control":
                    condit_nr = 0
                elif condition == "Positive Reinforcement":
                    condit_nr = 1
                elif condition == "Negative Reinforcement":
                    condit_nr = 2
                else:
                    condit_nr = -1
                    print("nope")
                conditions.append(condition)
                condition_numbers.append(condit_nr)
                repetitions.append(repetition)
                participant.append(i)
                set = self.filtered_data[i][j]["setting"]
                setting = set[10]  # Not very nicely done this
                settings.append(setting)

                # RMSE
                angle_error = self.filtered_data[i][j]["angle_error"]
                rate_error = self.filtered_data[i][j]["rate_error"]
                rms_angle_error.append(np.sqrt(1 / (len(angle_error)) * np.inner(angle_error, angle_error)))
                rms_rate_error.append(np.sqrt(1 / (len(rate_error)) * np.inner(rate_error, rate_error)))

                # RMSU
                estimated_human_torque = self.filtered_data[i][j]["estimated_human_input"]
                estimation_error_human_torque = self.filtered_data[i][j]["input_estimation_error"]
                real_human_torque = np.array(estimated_human_torque) - np.array(estimation_error_human_torque)
                robot_torque = self.filtered_data[i][j]["torque"]
                steering_rate = self.filtered_data[i][j]["steering_rate"]

                # Power
                human_power = np.array(estimated_human_torque) * np.array(steering_rate)
                robot_power = np.array(robot_torque) * np.array(steering_rate)

                rms_estimated_human_torque.append(
                    np.sqrt(1 / (len(rate_error)) * np.inner(estimated_human_torque, estimated_human_torque)))
                rms_human_torque.append(np.sqrt(1 / (len(rate_error)) * np.inner(real_human_torque, real_human_torque)))
                rms_robot_torque.append(np.sqrt(1 / (len(rate_error)) * np.inner(robot_torque, robot_torque)))
                rms_human_power.append(np.sqrt(1 / (len(rate_error)) * np.inner(human_power, human_power)))
                rms_robot_power.append(np.sqrt(1 / (len(rate_error)) * np.inner(robot_power, robot_power)))

                # Force conflicts
                conflict = self.compute_conflict(robot_torque, real_human_torque)
                conflicts.append(conflict)

                # Average cost functions
                human_cost = self.filtered_data[i][j]["estimated_human_cost_1"]
                robot_cost = self.filtered_data[i][j]["robot_cost_pos"]
                system_cost = np.array(human_cost) + np.array(robot_cost)
                human_angle_cost.append(np.median(human_cost))
                robot_angle_cost.append(np.median(robot_cost))
                system_angle_cost.append(np.median(system_cost))

                # Average gains
                human_gain = self.filtered_data[i][j]["estimated_human_gain_pos"]
                robot_gain = self.filtered_data[i][j]["robot_gain_pos"]
                system_gain = np.array(human_gain) + np.array(robot_gain)
                human_angle_gain.append(np.median(human_gain))
                robot_angle_gain.append(np.median(robot_gain))
                system_angle_gain.append(np.median(system_gain))

                # C = (avg_robot_gain - avg_human_gain) / (avg_robot_gain + avg_human_gain)
                C = 1

                gain_var = np.var(human_gain)
                gain_variability.append(gain_var)
                authority.append(C)

        # Save to metrics dictionary
        self.metrics["rms_angle_error"] = rms_angle_error
        self.metrics["rms_rate_error"] = rms_rate_error
        self.metrics["rms_human_torque"] = rms_human_torque
        self.metrics["rms_estimated_human_torque"] = rms_estimated_human_torque
        self.metrics["rms_robot_torque"] = rms_robot_torque
        self.metrics["human_angle_cost"] = human_angle_cost
        self.metrics["robot_angle_cost"] = robot_angle_cost
        self.metrics["system_angle_cost"] = system_angle_cost
        self.metrics["human_angle_gain"] = human_angle_gain
        self.metrics["robot_angle_gain"] = robot_angle_gain
        self.metrics["system_angle_gain"] = system_angle_gain
        self.metrics["authority"] = authority
        self.metrics["repetition"] = repetitions
        self.metrics["settings"] = settings
        self.metrics["condition"] = conditions
        self.metrics["condition_number"] = condition_numbers
        self.metrics["participant"] = participant
        self.metrics["gain_variability"] = gain_variability
        self.metrics["conflict"] = conflicts
        self.metrics["rms_human_power"] = rms_human_power
        self.metrics["rms_robot_power"] = rms_robot_power


    def build_individual_metrics(self):
        conditions = []
        participants = []
        increase = []
        performance = []


        metrics = pd.DataFrame.from_dict(self.metrics)
        for i in range(self.participants):
            participant = metrics.loc[metrics['participant'] == i]
            manual_control = participant.loc[metrics['condition'] == "Manual Control"]

            negative_control = participant.loc[metrics['condition'] == "Negative Reinforcement"]
            mixed_control = participant.loc[metrics['condition'] == "Mixed Reinforcement"]
            positive_control = participant.loc[metrics['condition'] == "Positive Reinforcement"]

            participants.append([i, i, i, i])
            conditions.append(
                ["Manual Control", "Negative Reinforcement", "Mixed Reinforcement", "Positive Reinforcement"])
            m = manual_control["rms_angle_error"].mean()
            increase.append(
                [100*m/m, 100*m/negative_control["rms_angle_error"].mean(),
                 100*m/mixed_control["rms_angle_error"].mean(), 100*m/positive_control["rms_angle_error"].mean(),])
            performance.append(
                [m, negative_control["rms_angle_error"].mean(), mixed_control["rms_angle_error"].mean(), positive_control["rms_angle_error"].mean(),])

        self.metrics_individuals["participant"] = [item for sublist in participants for item in sublist]
        self.metrics_individuals["condition"] = [item for sublist in conditions for item in sublist]
        self.metrics_individuals["increase"] = [item for sublist in increase for item in sublist]
        self.metrics_individuals["performance"] = [item for sublist in performance for item in sublist]

    def cut_data(self, participant, trial):
        # Cut data
        filtered_data = {}
        if trial == 0:
            self.filtered_data[participant] = {}
        try:
            for key in self.raw_data[participant][trial].keys():
                data = self.raw_data[participant][trial][key]
                time = np.array(self.raw_data[participant][trial]['time'])
                start_time = 0.01 * time[-1]
                end_time = 0.99 * time[-1]
                # Find index where to cut the data
                # print(start_time, end_time)
                start_index = np.argmax(time > start_time)
                end_index = np.argmax(time > end_time)
                filtered_data[key] = data[start_index:end_index]
        except:
            print("no data found")

        self.filtered_data[participant][trial] = filtered_data

    def compute_conflict(self, ur, uh):
        conflicts = 0
        n = len(ur)
        for i in range(n):
            if ur[i]*uh[i] < 0:
                conflicts += 1
        conflict = conflicts/n
        return conflict