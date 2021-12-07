import sys
import os
import pickle
sys.path.insert(1, '..')
import pandas as pd
import numpy as np
from Experiment.plots import PlotStuff
import matplotlib.pyplot as plt
import scipy.stats as stat
# from statsmodels.stats.anova import AnovaRM
from matplotlib.backends.backend_pdf import PdfPages
from Experiment.subjective import Subjective

class Analysis():
    def __init__(self):
        # Unpack data
        self.a = None
        self.raw_data = {}
        self.raw_data_robot = {}
        self.filtered_data = {}
        self.average_data = {}
        self.metrics = {}
        self.metrics_robot = {}
        self.metrics_averaged = {}
        self.plot_stuff = None
        self.trials = 12
        self.participants = 0
        self.periods = 4
        self.conditions = 3
        self.robot_trials = 8
        self.statistics = {
            "title": [],
            "test_type": [],
            "p": [],
        }
        self.subjective = Subjective()

    def initialize(self):
        self.a = input("Create metrics? 0: No, Yes: 1.  Your answer = ")
        if int(self.a) == 1:
            self.unpack_data()
        else:
            print("pickling stuff")
            pickle_off = open("data_experiment", 'rb')
            self.raw_data = pickle.load(pickle_off)
            pickle_robot = open("data_robot", 'rb')
            self.raw_data_robot = pickle.load(pickle_robot)
            print("done pickling")
            self.trials = 12
            self.robot_trials = 10
            self.participants = 3
            # print(self.raw_data)
        self.plot_stuff = PlotStuff()

    def unpack_data(self):
        # path = "..\\Experiment\\data"
        path_robot = "..\\Controller_Design\\data_robot"
        path = "..\\Experiment\\data"
        # path = "first_trial"
        list_dir_robot = os.listdir(path_robot)
        self.robot_trials = len(list_dir_robot)
        for j in range(self.robot_trials):
            path_trial_robot = path_robot + "\\" + list_dir_robot[j]
            try:
                df = pd.read_csv(path_trial_robot, index_col=0)
                self.raw_data_robot[j] = df.to_dict(orient='list')
                print("loaded ", path_trial_robot)
            except:
                exit("Something went wrong")

        # Save as pickle element
        pickling_robot = open("data_robot", "wb")
        pickle.dump(self.raw_data_robot, pickling_robot)
        pickling_robot.close()

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

        # Save as pickle element
        pickling_on = open("data_experiment", "wb")
        pickle.dump(self.raw_data, pickling_on)
        pickling_on.close()

    def analyse(self):
        # First, build some metrics
        for i in range(self.participants):
            for j in range(self.trials):
                self.cut_data(i, j)
        print(self.a)
        if int(self.a) == 1:
            print("we here")
            self.build_metrics()
            self.build_individual_metrics()
            # Save as pickle element
            pickling_on1 = open("metrics", "wb")
            pickling_on2 = open("metrics_ind", "wb")
            pickling_on3 = open("metrics_robot", "wb")
            pickle.dump(self.metrics, pickling_on1)
            pickle.dump(self.metrics_averaged, pickling_on2)
            pickle.dump(self.metrics_robot, pickling_on3)
            pickling_on1.close()
            pickling_on2.close()
            pickling_on3.close()
        else:
            print("we not here")
            pickle_off1 = open("metrics", 'rb')
            pickle_off2 = open("metrics_ind", 'rb')
            pickle_off3 = open("metrics_robot", 'rb')
            self.metrics = pickle.load(pickle_off1)
            self.metrics_averaged = pickle.load(pickle_off2)
            self.metrics_robot = pickle.load(pickle_off3)

        # Plot individual data
        print("number of participants = ", self.participants)
        self.plot_stuff.plot_participant(self.raw_data, trials=self.trials, participant=self.participants - 1)
        # self.plot_stuff.plot_participant(self.raw_data, trials=self.trials, participant=self.participants - 4)
        # self.plot_stuff.plot_participant(self.raw_data, trials=self.trials, participant=self.participants - 7)
        self.plot_stuff.plot_metrics(self.metrics, conditions=self.conditions, participant=self.participants - 1)

        # Plot metrics
        self.plot_stuff.plot_experiment(self.metrics_averaged, self.metrics_robot, self.participants, self.conditions)

        # plt.show()
        self.save_all_figures()
        # plt.close()

    def save_all_figures(self):
        pp = PdfPages('..\\Experiment\\figures\\test.pdf')
        figs = None
        if figs is None:
            figs = [plt.figure(n) for n in plt.get_fignums()]
        for fig in figs:
            fig.savefig(pp, format='pdf')
        pp.close()



    def save_data(self, data, file):
        df = pd.DataFrame(data=data)
        df.to_csv(file)

    def build_metrics(self):
        # Check metrics per participant: sorted per (participant, condition)
        # RMSE (performance), RMSU (effort), Gains

        # Pre-define metrics
        self.metrics["rms_angle_error"] = {}
        self.metrics["rms_angle_error_rad"] = {}
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
        self.metrics["cost_variability"] = {}
        self.metrics["trial"] = {}
        # self.metrics["label"] = {}
        # self.metrics["gains_normalized"] = {}
        # self.metrics["error_normalized"] = {}

        # Robot RMS and gains
        self.metrics_robot["rms_angle_error"] = []
        self.metrics_robot["rms_angle_error_rad"] = []
        self.metrics_robot["robot_angle_gain"] = []


        # RMS
        rms_angle_error = []
        rms_angle_error_rad = []
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
        robot_rms_rad = []
        robot_gains = []

        # Co-adaptation
        authority = []
        cost_variability = []
        conflicts = []

        # Info
        repetitions = []
        conditions = []
        condition_numbers = []
        participant = []
        settings = []
        trial_order = [0, 1, 10, 11, 2, 3, 4, 5, 6, 7, 8, 9]
        trials = []

        for k in range(self.robot_trials):
            # Robot
            robot_error = self.raw_data_robot[k]["angle_error"]
            robot_error_deg = np.array(robot_error) * 180 / np.pi
            robot_rms_rad.append(np.sqrt(1 / (len(robot_error)) * np.inner(robot_error, robot_error)))
            robot_rms.append(np.sqrt(1 / (len(robot_error_deg)) * np.inner(robot_error_deg, robot_error_deg)))
            robot_solo_gain = self.raw_data_robot[k]["robot_gain_pos"]
            robot_gains.append(np.median(robot_solo_gain))

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
                repetitions.append(repetition+1)
                participant.append(i)
                set = self.filtered_data[i][j]["setting"]
                setting = set[10]  # Not very nicely done this
                settings.append(setting)

                # RMSE
                angle_error = self.filtered_data[i][j]["angle_error"]
                rate_error = self.filtered_data[i][j]["rate_error"]
                error_deg = np.array(angle_error) * 180 / np.pi
                rms_angle_error_rad.append(np.sqrt(1 / (len(angle_error)) * np.inner(angle_error, angle_error)))
                rms_angle_error.append(np.sqrt(1 / (len(error_deg)) * np.inner(error_deg, error_deg)))
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

                rms_estimated_human_torque.append(np.sqrt(1 / (len(rate_error)) * np.inner(estimated_human_torque, estimated_human_torque)))
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

                auth = (human_power - robot_power) / (human_power + robot_power)
                C = np.median(auth)

                cost_var = np.var(human_angle_cost)
                cost_variability.append(cost_var)
                authority.append(C)
            trials.extend(trial_order)

        # Save to metrics dictionary
        self.metrics["rms_angle_error_rad"] = rms_angle_error_rad
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
        self.metrics["cost_variability"] = cost_variability
        self.metrics["conflict"] = conflicts
        self.metrics_robot["rms_angle_error"] = robot_rms
        self.metrics_robot["rms_angle_error_rad"] = robot_rms_rad
        self.metrics_robot["robot_angle_gain"] = robot_gains
        self.metrics["rms_human_power"] = rms_human_power
        self.metrics["rms_robot_power"] = rms_robot_power
        self.metrics["trial"] = trials

    def build_individual_metrics(self):
        conditions = []
        condition_nr = []
        participants = []
        increase = []
        performance = []

        # Costs
        cost_human = []
        cost_robot = []
        cost_system = []

        # Gains
        gain_human = []
        gain_robot = []
        gain_system = []

        # Inputs
        inputs_human = []
        inputs_robot = []

        # Powerr
        human_power = []
        robot_power = []

        # authority
        auth = []

        # labels
        labels = []

        error_normalized = []
        gains_normalized = []

        metrics = pd.DataFrame.from_dict(self.metrics)
        for i in range(self.participants):
            participant = metrics.loc[metrics['participant'] == i]
            manual_control = participant.loc[metrics['condition'] == "Manual Control"]
            negative_control = participant.loc[metrics['condition'] == "Negative Reinforcement"]
            positive_control = participant.loc[metrics['condition'] == "Positive Reinforcement"]

            participants.append([i, i, i])
            conditions.append(
                ["Manual Control", "Negative Reinforcement", "Positive Reinforcement"])
            condition_nr.append([0, 1, 2])
            m = manual_control["rms_angle_error"].mean()
            increase.append([100*m/m, 100*m/negative_control["rms_angle_error"].mean(),
                             100*m/positive_control["rms_angle_error"].mean()])
            performance.append([m, negative_control["rms_angle_error"].mean(),
                                positive_control["rms_angle_error"].mean()])

            # Costs
            cost_human.append([manual_control["human_angle_cost"].mean(), negative_control["human_angle_cost"].mean(),
                                positive_control["human_angle_cost"].mean()])
            cost_robot.append([manual_control["robot_angle_cost"].mean(), negative_control["robot_angle_cost"].mean(),
                               positive_control["robot_angle_cost"].mean()])
            cost_system.append([manual_control["system_angle_cost"].mean(), negative_control["system_angle_cost"].mean(),
                               positive_control["system_angle_cost"].mean()])

            # Gains
            gain_human.append([manual_control["human_angle_gain"].mean(), negative_control["human_angle_gain"].mean(),
                               positive_control["human_angle_gain"].mean()])
            gain_robot.append([manual_control["robot_angle_gain"].mean(), negative_control["robot_angle_gain"].mean(),
                               positive_control["robot_angle_gain"].mean()])
            gain_system.append([manual_control["system_angle_gain"].mean(), negative_control["system_angle_gain"].mean(),
                               positive_control["system_angle_gain"].mean()])

            participant_now = metrics.loc[metrics['participant'] == i]
            skill = manual_control["system_angle_gain"].mean()
            error_skill = manual_control["rms_angle_error"].mean()

            negative_gains = negative_control["human_angle_gain"].values
            trials_neg = negative_control["repetition"].values
            trials_ordered = participant_now["repetition"].values
            gains_ordered_neg = negative_gains[trials_neg - 1]
            # print(trials_ordered)
            # print(participant_now["human_angle_gain"].values[trials_ordered - 1])

            gains_normalized.extend(participant_now["human_angle_gain"].values - skill)
            error_normalized.extend(participant_now["rms_angle_error"].values - error_skill)


            # print(negative_gains)
            # print(trials)
            # print(negative_gains[trials-1])

            if gains_ordered_neg[0] - gains_ordered_neg[3] > 1 or gains_ordered_neg[0] - gains_ordered_neg[2] > 1:
                label = "Slacker"
            elif negative_gains[0] < 0 and negative_gains[1] < 0 and negative_gains[2] < 0 and negative_gains[3] < 0:
                label = "Fighter"
            else:
                label = "Cooperator"

            labels.extend([label] * self.trials)


            # Inputs
            inputs_human.append([manual_control["rms_estimated_human_torque"].mean(),
                                 negative_control["rms_estimated_human_torque"].mean(),
                                 positive_control["rms_estimated_human_torque"].mean()])
            inputs_robot.append([manual_control["rms_robot_torque"].mean(), negative_control["rms_robot_torque"].mean(),
                                 positive_control["rms_robot_torque"].mean()])

            # Power
            human_power.append([manual_control["rms_human_power"].mean(),
                                 negative_control["rms_human_power"].mean(),
                                 positive_control["rms_human_power"].mean()])
            robot_power.append([manual_control["rms_robot_power"].mean(), negative_control["rms_robot_power"].mean(),
                                 positive_control["rms_robot_power"].mean()])

            # auth
            auth.append([manual_control["authority"].mean(),
                                 negative_control["authority"].mean(),
                                 positive_control["authority"].mean()])

        self.metrics_averaged["participant"] = [item for sublist in participants for item in sublist]
        self.metrics_averaged["condition"] = [item for sublist in conditions for item in sublist]
        self.metrics_averaged["condition_nr"] = [item for sublist in condition_nr for item in sublist]
        self.metrics_averaged["increase"] = [item for sublist in increase for item in sublist]
        self.metrics_averaged["performance"] = [item for sublist in performance for item in sublist]
        self.metrics_averaged["cost_human"] = [item for sublist in cost_human for item in sublist]
        self.metrics_averaged["cost_robot"] = [item for sublist in cost_robot for item in sublist]
        self.metrics_averaged["cost_system"] = [item for sublist in cost_system for item in sublist]
        self.metrics_averaged["gain_human"] = [item for sublist in gain_human for item in sublist]
        self.metrics_averaged["gain_robot"] = [item for sublist in gain_robot for item in sublist]
        self.metrics_averaged["gain_system"] = [item for sublist in gain_system for item in sublist]
        self.metrics_averaged["inputs_human"] = [item for sublist in inputs_human for item in sublist]
        self.metrics_averaged["inputs_robot"] = [item for sublist in inputs_robot for item in sublist]
        self.metrics_averaged["human_power"] = [item for sublist in human_power for item in sublist]
        self.metrics_averaged["robot_power"] = [item for sublist in robot_power for item in sublist]
        self.metrics_averaged["authority"] = [item for sublist in auth for item in sublist]

        # Subjective measures
        self.metrics_averaged["subjective_1"] = self.subjective.s1_ordered[0:self.participants*3]
        self.metrics_averaged["subjective_2"] = self.subjective.s2_ordered[0:self.participants*3]
        self.metrics_averaged["subjective_3"] = self.subjective.s3_ordered[0:self.participants*3]
        self.metrics_averaged["subjective_4"] = self.subjective.s4_ordered[0:self.participants*3]
        self.metrics_averaged["subjective_authority"] = self.subjective.auth_ordered[0:self.participants*3]

        print("made it here")
        # Label participants
        self.metrics["label"] = labels
        self.metrics["error_normalized"] = error_normalized
        self.metrics["gains_normalized"] = gains_normalized
        # print(self.metrics["error_normalized"] )

    def cut_data(self, participant, trial):
        # Cut data
        if trial == 0:
            self.filtered_data[participant] = {}
        filtered_data = {}
        try:
            for key in self.raw_data[participant][trial].keys():
                data = self.raw_data[participant][trial][key]
                time = np.array(self.raw_data[participant][trial]['time'])
                start_time = 0.1 * time[-1]
                end_time = 0.9 * time[-1]
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