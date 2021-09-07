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
        self.t_exp = input["experiment_time"]
        self.periods = input["periods"]
        self.t_period = input["period_time"]
        self.Qr_start = input["init_robot_cost"]
        self.Qr_end = input["final_robot_cost"]
        self.virtual_human_cost = input["virtual_human_cost"]
        self.sharing_rule = input["sharing_rule"]
        self.preview_time = input["preview_time"]
        self.repetitions = input["repetitions"]
        self.sigma = input["sigma"]
        self.duration = self.t_warmup + self.t_exp + self.t_cooldown
        self.t_now = 0
        self.t_last = 0
        self.t0 = 0
        self.time = 0
        self.cond = 0
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
        self.visualize.visualize_experiment(0, [], 0, text="Initializing", top_text="", sigma=0)
        time.sleep(1.5)

        print("init complete!")

    def experiment(self, condition):
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

        # First x trials are manual control
        if condition < self.repetitions:
            self.send_dict["manual"] = True
        else:
            self.send_dict["manual"] = False

        # Press spacebar to start trial
        while not ready:
            if self.senso_process.is_alive():
                self.quit = self.visualize.check_quit()
                if self.quit:
                    self.send_dict["exit"] = True
                self.send_dict["factor"] = 0
                self.send_dict["reset"] = True
                self.send_dict["experiment"] = False
                self.send_dict["ref"] = np.array([0.0, 0.0])
                self.send_dict["robot_cost"] = self.Qr_start
                self.send_dict["sharing_rule"] = self.sharing_rule
                self.parent_conn.send(self.send_dict)  # Child is for sending
                new_states = self.parent_conn.recv()  # Receive from child
            else:
                print("sensodrive process was killed")
                return -1
            time.sleep(0.005)
            if keyboard.is_pressed(' '):
                break
            self.visualize.visualize_experiment(0, [], new_states["steering_angle"],
                                                text="Press spacebar to start the trial", top_text="", sigma=0, )

        self.t0 = time.perf_counter_ns()
        self.t_last = time.perf_counter_ns()
        self.send_dict["reset"] = False
        self.time = 0
        self.cond = 0
        self.time = 0

        # Loop over 1 trial
        while self.time < self.duration:
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

            # Compute conditions
            c = math.floor(self.time_exp/self.t_period)
            if c >= 0:
                sigma_h = self.sigma[c]
            else:
                sigma_h = self.sigma[0]

            # Compute reference
            ref = self.reference.generate_reference(self.time, sigma=0, player="robot", ref_sign=self.ref_sig)
            self.reference_preview(self.time, h, sigma_h=sigma_h)

            if condition < self.repetitions:
                self.send_dict["sharing_rule"] = self.estimated_human_cost
            else:
                self.send_dict["sharing_rule"] = self.sharing_rule

            # WARM_UP
            if self.time < self.t_warmup:
                # 3 second start screen
                self.store = False
                self.send_dict["factor"] = 1 / (1 + np.exp(-2 * self.time)) - 1 / (1 + np.exp(2 * self.time))
                self.send_dict["ref"] = ref * self.send_dict["factor"]
                self.send_dict["experiment"] = False


                # Text to display
                dt = self.t_warmup - self.time
                top_text = ""
                if dt > 3:
                    text = "Trial starts in:"
                else:
                    text = str(round(dt, 1))

            # EXPERIMENT
            elif self.t_warmup <= self.time < (self.t_warmup + self.t_exp):
                self.store = True
                self.send_dict["factor"] = 1
                self.send_dict["ref"] = ref
                self.send_dict["experiment"] = True

                # Determine condition
                text = ""
                top_text = "Time = " + str(round(self.time - self.t_warmup, 1))

            # COOLDOWN
            else:
                self.store = False
                self.send_dict["factor"] = 1 / (1 + np.exp(-2 * (self.time - self.t_warmup - self.t_exp))) - 1 / \
                                           (1 + np.exp(2 * (self.time - self.t_warmup - self.t_exp)))
                self.send_dict["ref"] = ref
                self.send_dict["experiment"] = False
                text = "Finished trial"
                top_text = ""

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

            # self.estimated_human_cost = np.array(self.states["estimated_human_gain"])

            # Visualize experiment
            self.visualize.visualize_experiment(self.send_dict["ref"][0], angle=self.states["steering_angle"],
                                                r_prev=self.preview_positions, text=text, top_text=top_text, sigma=sigma_h)

            if self.store:
                output = {
                    "time": self.time - self.t_warmup,
                    "steering_angle": self.states["steering_angle"],
                    "steering_rate": self.states["steering_rate"],
                    "angle_error": self.states["angle_error"][0],
                    "rate_error": self.states["rate_error"][0],
                    "acceleration": self.states["steering_acc"],
                    "reference_angle": ref[0],
                    "reference_rate": ref[1],
                    "measured_input": self.states["measured_input"],
                    "execution_time": h,
                    "condition": self.cond,
                    "human_noise": sigma_h,
                }
                try:
                    output['torque'] = self.states["torque"][0, 0]
                except:
                    output["torque"] = 0
                if self.controller_type == "Gain_observer" or self.controller_type == "Cost_observer":
                    output["estimated_human_gain_pos"] = self.states["estimated_human_gain"].flatten()[0]
                    output["estimated_human_gain_vel"] = self.states["estimated_human_gain"].flatten()[1]

                    # print(self.states["robot_gain"])
                    output["robot_gain_pos"] = self.states["robot_gain"][0, 0]
                    output["robot_gain_vel"] = self.states["robot_gain"][0, 1]

                    output["state_estimate_pos"] = self.states["state_estimate"][0][0]
                    output["state_estimate_vel"] = self.states["state_estimate"][1][0]
                    try:
                        output["estimated_human_input"] = self.states["estimated_human_torque"][0, 0]
                        output["input_estimation_error"] = self.states["input_estimation_error"][0, 0]
                    except:
                        output["estimated_human_input"] = self.states["estimated_human_torque"][0]
                        output["input_estimation_error"] = self.states["input_estimation_error"][0]

                    output["robot_cost_pos"] = self.states["robot_cost_calc"][0, 0]
                    output["robot_cost_vel"] = self.states["robot_cost_calc"][1, 1]
                    if self.controller_type == "Cost_observer":
                        output["estimated_human_cost_1"] = self.states["estimated_human_cost"][0, 0]
                        output["estimated_human_cost_2"] = self.states["estimated_human_cost"][1, 1]

                # print(output)
                self.store_variables(output)

        # Time to close off
        # self.send_dict["exit"] = True
        # self.parent_conn.send(self.send_dict)

        return self.variables

    def store_variables(self, output):
        for key in output.keys():
            self.variables.setdefault(key, []).append(output[key])

    def reference_preview(self, t_now, h, sigma_h):
        steps = 250
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
            ref = self.reference.generate_reference(t_new, sigma_h, player="human", ref_sign=self.ref_sig)
            self.preview_times = np.append(self.preview_times, t_new)
            self.preview_positions = np.append(self.preview_positions, ref[0])
            self.preview_speed = np.append(self.preview_speed, ref[1])
            N = len(self.preview_positions)


        # Move points according to their velocity
        self.preview_positions += h * np.array(self.preview_speed)


