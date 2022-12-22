import time
import numpy as np
import wres
import platform
import keyboard
import random
import math



if platform.system() == 'Windows':
    import wres

class Experiment:
    def __init__(self, input, visualize):
        # 2 OBJECTS TO DRIVE THE EXPERIMENT
        # 1. Sensodrive, can be called to drive some torque and get sensor data
        super().__init__()
        self.visualize = visualize
        self.parent_conn = input["parent_conn"]
        self.child_conn = input["child_conn"]
        # self.data_plotter_conn = input["send_conn"]
        self.senso_process = input["senso_drive"]
        self.t_warmup = input["warm_up_time"]
        self.t_cooldown = input["cooldown_time"]
        self.experiment_time = input["experiment_time"]
        self.t_exp = self.experiment_time
        self.virtual_human_cost = input["virtual_human_cost"]
        self.sharing_rule = input["sharing_rule"]
        self.preview_time = input["preview_time"]
        self.trials = input["trials"]
        self.repetition = 0
        self.sigma = input["sigma"]
        self.duration = self.t_warmup + self.t_exp + self.t_cooldown
        # self.t_exp = 77.5*2
        self.t_now = 0
        self.t_last = 0
        self.t0 = 0
        self.time = 0
        self.cond = 0
        self.computed = False
        self.RMSE = 0
        self.visual_setting = 0
        self.reference = input["reference"]
        self.store = False
        self.start_measurements = False
        self.virtual_human_gain = input["virtual_human_gain"]
        self.send_dict = {
            "ref": None,
            "factor": 0,
            "exit": False,
            "robot_cost": None,
            "experiment": False,
            'manual': True,
            'condition': False,
        }
        self.visualize_dict = {
            "time_stamp": 0,
            "position": 0,
            "velocity": 0,
            "torque": 0,
            "exit": False
        }
        self.controller_type = input["controller_type"]
        self.preview = input["preview"]
        self.damping = input["damping"]
        self.stiffness = input["stiffness"]
        self.virtual_human = input["virtual_human"]
        self.damp = 0
        self.stiff = 0
        self.factor = 0
        self.new_states = {"steering_angle": 0, "measured_torque": 0}
        self.quit = False
        self.preview_positions = np.array([0.0])
        self.preview_speed = np.array([0.0])
        self.preview_times = np.array([0.0])
        self.ref_sig = 0
        self.estimated_human_cost = np.array([[0, 0], [0, 0]])

        # Initialize message structures
        self.states = {}
        self.variables = dict()

        print("sleeping for 2 seconds to see if motor is active")
        self.visualize.visualize_experiment(0, [], 0, text="Initializing", top_text="", bottom_text="", sigma=0)
        time.sleep(1.5)

        print("init complete!")

    def experiment(self, condition, repetition, trial):
        # reset values
        ready = False
        self.send_dict["exit"] = False
        self.send_dict["ref"] = [0, 0]
        self.preview_times = [0]
        self.preview_positions = [0]
        self.preview_speed = [0]
        self.time = 0
        self.cond = 0
        self.variables = dict()
        self.repetition = repetition

        # First x trials are manual control
        sigma_h = self.sigma[int(self.visual_setting)]
        setting = "Good Visuals"

        self.send_dict["condition"] = condition
        if condition == "Manual Control":
            self.send_dict["manual"] = True
            self.send_dict["sharing_rule"] = self.estimated_human_cost
        elif condition == "Practice":
            self.send_dict["condition"] = "Manual Control"
            self.send_dict["manual"] = True
            self.send_dict["sharing_rule"] = self.estimated_human_cost
        else:
            self.send_dict["manual"] = False
            self.send_dict["sharing_rule"] = self.sharing_rule

        print(condition)

        # Press spacebar to start trial
        trial_nr = trial + 1
        top_text = "Trial " + str(trial_nr) + "/" + str(self.trials)
        while not ready:
            if self.senso_process.is_alive():
                self.quit = self.visualize.check_quit()
                if self.quit:
                    self.send_dict["exit"] = True
                self.factor = 0
                self.send_dict["reset"] = True
                self.send_dict["experiment"] = False
                self.send_dict["ref"] = np.array([0.0, 0.0])
                self.parent_conn.send(self.send_dict)  # Child is for sending
                new_states = self.parent_conn.recv()  # Receive from child
                self.computed = False
            else:
                print("sensodrive process was killed")
                return -1
            time.sleep(0.005)
            if keyboard.is_pressed(' '):
                break
            self.visualize.visualize_experiment(0, [], new_states["steering_angle"],
                                                text="Press spacebar to start the trial", top_text=top_text, bottom_text="", sigma=0, )

        self.t0 = time.perf_counter_ns()
        self.t_last = time.perf_counter_ns()
        self.send_dict["reset"] = False
        self.time = 0
        self.cond = 0
        self.time = 0
        self.factor = 0.25
        top_text = ""

        if condition == "Practice":
            self.t_exp = 0.5 * self.experiment_time
        else:
            self.t_exp = self.experiment_time

        self.duration = self.t_warmup + self.t_exp + self.t_cooldown

        # Loop over 1 trial
        while self.time < 1.0*self.duration:
            # Check whether the process has not been quited
            self.quit = self.visualize.check_quit()
            if self.quit:
                self.send_dict["exit"] = True

            # Calculate timings
            self.t_now = time.perf_counter_ns()
            h = (self.t_now - self.t_last) * 1e-9
            self.t_last = self.t_now
            self.time = (self.t_last - self.t0) * 1e-9
            self.time_exp = self.time - self.t_warmup

            # Compute reference
            delay = 0.180
            ref = self.reference.generate_reference(self.time - delay, sigma=0, player="robot", ref_sign=self.repetition)
            self.reference_preview(self.time, h, sigma_h=sigma_h)

            # WARM_UP
            if self.time < self.t_warmup:
                # 3 second start screen
                self.store = False
                self.factor = math.tanh(self.time)
                self.send_dict["factor"] = self.factor
                self.send_dict["ref"] = ref
                self.send_dict["experiment"] = False

                # Text to display
                dt = self.t_warmup - self.time
                top_text = ""
                bottom_text = ""
                if dt > 3:
                    text = "Trial starts in:"
                else:
                    text = str(round(dt, 1))

            # EXPERIMENT
            elif self.t_warmup <= self.time < (self.t_warmup + self.t_exp):
                self.store = True
                self.factor = 1
                self.send_dict["factor"] = self.factor
                self.send_dict["ref"] = ref
                self.send_dict["experiment"] = True
                estimated_gain_pos = self.states["estimated_human_gain"].flatten()[0]
                robot_gain_pos = self.states["robot_gain"][0, 0]
                # print(self.states["robot_gain"])


                if self.virtual_human:
                    Qh, vhg = self.smooth_virtual_human()
                    self.send_dict["virtual_human_gain"] = vhg
                    virtual_gain = vhg[0]
                    bottom_text = "Gains; Human:" + str(round(estimated_gain_pos, 2)) + " Robot: " + str(
                        round(robot_gain_pos, 2)) \
                                  + " Virtual H: " + str(round(virtual_gain, 2))
                else:
                    bottom_text = "Gains; Human:" + str(round(estimated_gain_pos, 2)) + " Robot: " + str(
                        round(robot_gain_pos, 2))


                top_text = "Time = " + str(round(self.time - self.t_warmup, 1))
                text = ""

                # Uncomment to remove text
                # top_text = ""
                # bottom_text = ""

            # COOLDOWN
            else:
                self.compute_error()
                self.store = False
                self.factor = -math.tanh(self.time - self.duration)
                self.send_dict["factor"] = self.factor
                self.send_dict["ref"] = ref
                self.send_dict["experiment"] = False
                text = "Finished trial"
                bottom_text = "Performance Score (RMSE) = " + str(round(self.RMSE, 4))

            # Send data to the child
            if self.senso_process.is_alive():
                self.parent_conn.send(self.send_dict)  # Child is for sending
                new_states = self.parent_conn.recv()  # Receive from child
            else:
                print("sensodrive process was killed")
                return None

            if self.new_states == None:
                print("Double check with error")
            else:
                self.states = new_states

            self.estimated_human_cost = np.array(self.states["estimated_human_cost"])

            # Visualize experiment
            self.visualize.visualize_experiment(self.send_dict["ref"][0], angle=self.states["steering_angle"],
                                                r_prev=self.preview_positions, text=text, top_text=top_text,
                                                bottom_text=bottom_text, sigma=sigma_h)

            if self.store:
                output = {
                    "time": self.time - self.t_warmup,
                    "steering_angle": self.states["steering_angle"],
                    "steering_rate": self.states["steering_rate"],
                    "angle_error": self.states["angle_error"],
                    "rate_error": self.states["rate_error"],
                    "acceleration": self.states["steering_acc"],
                    "reference_angle": ref[0],
                    "reference_rate": ref[1],
                    "measured_input": self.states["measured_input"],
                    "execution_time": h,
                    "repetition": self.repetition,
                    "condition": condition,
                    "human_noise": sigma_h,
                    "setting": setting,
                    "robot_gain_pos": self.states["robot_gain"][0, 0],
                    "robot_gain_vel": self.states["robot_gain"][0, 1],
                }
                try:
                    output['torque'] = self.states["torque"][0, 0]
                except:
                    output["torque"] = 0

                if self.controller_type == "Gain_observer" or self.controller_type == "Cost_observer":
                    try:
                        output["measured_human_input"] = self.states["measured_human_input"][0, 0]
                    except:
                        output["measured_human_input"] = self.states["measured_human_input"][0][0]


                    output["estimated_human_gain_pos"] = self.states["estimated_human_gain"].flatten()[0]
                    output["estimated_human_gain_vel"] = self.states["estimated_human_gain"].flatten()[1]

                    # print(self.states["robot_gain"])


                    output["state_estimate_pos"] = self.states["estimated_state"][0][0]
                    output["state_estimate_vel"] = self.states["estimated_state"][1][0]
                    output["input_estimation_error"] = self.states["input_estimation_error"]
                    output["state_estimate_derivative"] = self.states["state_estimate_derivative"][1][0]

                    if self.virtual_human:
                        output["virtual_human_gain_pos"] = vhg[0]
                        output["virtual_human_gain_vel"] = vhg[1]
                        output["virtual_human_cost_pos"] = Qh[0]
                        output["virtual_human_cost_vel"] = Qh[1]
                        try:
                            output["virtual_human_torque"] = self.states["virtual_human_torque"][0, 0]
                        except:
                            output["virtual_human_torque"] = self.states["virtual_human_torque"][0]

                    try:
                        output["estimated_human_input"] = self.states["estimated_human_torque"][0, 0]
                    except:
                        output["estimated_human_input"] = self.states["estimated_human_torque"][0]


                    output["robot_cost_pos"] = self.states["robot_cost_calc"][0, 0]
                    output["robot_cost_vel"] = self.states["robot_cost_calc"][1, 1]

                    output["estimated_human_cost_1"] = self.states["estimated_human_cost"][0, 0]
                    output["estimated_human_cost_2"] = self.states["estimated_human_cost"][1, 1]

                # print(output)
                self.store_variables(output)

        ready = False

        print(round(self.RMSE, 4))

        if condition == "Practice":
            # Press spacebar to start trial
            while not ready:
                if self.senso_process.is_alive():
                    self.quit = self.visualize.check_quit()
                    if self.quit:
                        self.send_dict["exit"] = True
                    self.factor = 0
                    self.send_dict["reset"] = True
                    self.send_dict["experiment"] = False
                    self.send_dict["ref"] = np.array([0.0, 0.0])
                    self.parent_conn.send(self.send_dict)  # Child is for sending
                    new_states = self.parent_conn.recv()  # Receive from child
                    self.computed = False
                else:
                    print("sensodrive process was killed")
                    return -1
                time.sleep(0.005)
                if keyboard.is_pressed('S'):
                    ready_for_experiment = True
                    break
                if keyboard.is_pressed('P'):
                    ready_for_experiment = False
                    break
                self.visualize.visualize_experiment(0, [], new_states["steering_angle"],
                                                    text="To do an extra practice run press P, To start experiment press S",
                                                    top_text="", bottom_text="", sigma=0, )

        else:
            ready_for_experiment = True

        # Time to close off
        # self.send_dict["exit"] = True
        # self.parent_conn.send(self.send_dict)

        return ready_for_experiment, self.variables

    def ask_agency(self):
        while True:
            if self.senso_process.is_alive():
                self.quit = self.visualize.check_quit()
                if self.quit:
                    self.send_dict["exit"] = True
                self.factor = 0
                self.send_dict["reset"] = True
                self.send_dict["experiment"] = False
                self.send_dict["ref"] = np.array([0.0, 0.0])
                self.parent_conn.send(self.send_dict)  # Child is for sending
                new_states = self.parent_conn.recv()  # Receive from child
                self.computed = False
            else:
                print("sensodrive process was killed")
                return -1
            time.sleep(0.005)
            text = "Please fill in the questionnaire"

            if keyboard.is_pressed('enter'):
                break

            self.visualize.visualize_experiment(0, [], new_states["steering_angle"],
                                                text=text, top_text="",
                                                bottom_text="", sigma=0, )


    def store_variables(self, output):
        for key in output.keys():
            self.variables.setdefault(key, []).append(output[key])

    def compute_error(self):
        if not self.computed:
            e = self.variables["angle_error"]
            self.RMSE = np.sqrt(1/len(e) * np.inner(e, e))
            self.computed = True

    def reference_preview(self, t_now, h, sigma_h):
        steps = 25
        dt = self.preview_time / steps

        # Remove past points
        i = 0
        N = len(self.preview_positions)

        while i < N:
            if self.preview_times[i] < t_now:
                self.preview_times = np.delete(self.preview_times, i)
                self.preview_positions = np.delete(self.preview_positions, i)
                self.preview_speed = np.delete(self.preview_speed, i)
            else:
                break
            N = len(self.preview_positions)

        # Append until again back at enough steps
        while N < steps:
            try:
                t_new = self.preview_times[-1] + dt
            except:
                t_new = dt
            ref = self.reference.generate_reference(t_new, sigma_h, player="human", ref_sign=self.repetition)
            self.preview_times = np.append(self.preview_times, t_new)
            self.preview_positions = np.append(self.preview_positions, ref[0])
            self.preview_speed = np.append(self.preview_speed, ref[1])
            N = len(self.preview_positions)

        # Move points according to their velocity
        self.preview_positions += h * np.array(self.preview_speed)

    def smooth_virtual_human(self):
        if self.time - self.t_warmup < 1 / 6 * self.t_exp:
            dL1 = self.virtual_human_gain[0, :] - self.virtual_human_gain[1, :]
            L1 = self.virtual_human_gain[1, :]
            dL2 = self.virtual_human_gain[1, :] - self.virtual_human_gain[0, :]
            vhg = self.compute_tanh(0, 1/6, dL1, dL2, L1)
            dQh1 = self.virtual_human_cost[0, :]
            Qh1 = self.virtual_human_cost[1, :]
            dQh2 = self.virtual_human_cost[1, :] - self.virtual_human_cost[0, :]
            Qh = self.compute_tanh(0, 1 / 6, dQh1, dQh2, Qh1)
        elif 2 / 6 * self.t_exp > self.time - self.t_warmup > 1 / 6 * self.t_exp:
            dL1 = self.virtual_human_gain[1, :] - self.virtual_human_gain[0, :]
            L1 = self.virtual_human_gain[0, :]
            dL2 = self.virtual_human_gain[2, :] - self.virtual_human_gain[1, :]
            vhg = self.compute_tanh(1/6, 2/6, dL1, dL2, L1)
            dQh1 = self.virtual_human_cost[1, :] - self.virtual_human_cost[0, :]
            Qh1 = self.virtual_human_cost[0, :]
            dQh2 = self.virtual_human_cost[2, :] - self.virtual_human_cost[1, :]
            Qh = self.compute_tanh(1 / 6, 2 / 6, dQh1, dQh2, Qh1)
        elif 3 / 6 * self.t_exp > self.time - self.t_warmup > 2 / 6 * self.t_exp:
            dL1 = self.virtual_human_gain[2, :] - self.virtual_human_gain[1, :]
            L1 = self.virtual_human_gain[1, :]
            dL2 = self.virtual_human_gain[3, :] - self.virtual_human_gain[2, :]
            vhg = self.compute_tanh(2/6, 3/6, dL1, dL2, L1)
            dQh1 = self.virtual_human_cost[2, :] - self.virtual_human_cost[1, :]
            Qh1 = self.virtual_human_cost[1, :]
            dQh2 = self.virtual_human_cost[3, :] - self.virtual_human_cost[2, :]
            Qh = self.compute_tanh(2 / 6, 3 / 6, dQh1, dQh2, Qh1)
        elif 4 / 6 * self.t_exp > self.time - self.t_warmup > 3 / 6 * self.t_exp:
            dL1 = self.virtual_human_gain[3, :] - self.virtual_human_gain[2, :]
            L1 = self.virtual_human_gain[2, :]
            dL2 = self.virtual_human_gain[4, :] - self.virtual_human_gain[3, :]
            vhg = self.compute_tanh(3/6, 4/6, dL1, dL2, L1)
            dQh1 = self.virtual_human_cost[3, :] - self.virtual_human_cost[2, :]
            Qh1 = self.virtual_human_cost[2, :]
            dQh2 = self.virtual_human_cost[4, :] - self.virtual_human_cost[3, :]
            Qh = self.compute_tanh(3/ 6, 4 / 6, dQh1, dQh2, Qh1)
        elif 5 / 6 * self.t_exp > self.time - self.t_warmup > 4 / 6 * self.t_exp:
            dL1 = self.virtual_human_gain[4, :] - self.virtual_human_gain[3, :]
            L1 = self.virtual_human_gain[3, :]
            dL2 = self.virtual_human_gain[5, :] - self.virtual_human_gain[4, :]
            vhg = self.compute_tanh(4/6, 5/6, dL1, dL2, L1)
            dQh1 = self.virtual_human_cost[4, :] - self.virtual_human_cost[3, :]
            Qh1 = self.virtual_human_cost[3, :]
            dQh2 = self.virtual_human_cost[5, :] - self.virtual_human_cost[4, :]
            Qh = self.compute_tanh(4 / 6, 5 / 6, dQh1, dQh2, Qh1)
        else:
            dL1 = self.virtual_human_gain[5, :] - self.virtual_human_gain[4, :]
            L1 = self.virtual_human_gain[4, :]
            dL2 = - self.virtual_human_gain[5, :]
            vhg = self.compute_tanh(5/6, 6/6, dL1, dL2, L1)
            dQh1 = self.virtual_human_cost[5, :] - self.virtual_human_cost[4, :]
            Qh1 = self.virtual_human_cost[4, :]
            dQh2 = - self.virtual_human_cost[5, :]
            Qh = self.compute_tanh(5 / 6, 6 / 6, dQh1, dQh2, Qh1)

        return Qh, vhg

    def compute_tanh(self, f1, f2, dL1, dL2, L1):
        value = (0.5 * dL1 * np.tanh(5 * (self.time - self.t_warmup - f1 * self.t_exp)) + 0.5 * dL1 + L1) + \
                  (0.5 * dL2 * np.tanh(5 * (self.time - self.t_warmup - f2 * self.t_exp)) + 0.5 * dL2)
        return value