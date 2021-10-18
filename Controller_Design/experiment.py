import time
import numpy as np
import wres
import platform
import math

from Experiment.visuals import Visualize

if platform.system() == 'Windows':
    import wres

class Experiment:
    def __init__(self, input):
        # 2 OBJECTS TO DRIVE THE EXPERIMENT
        # 1. Sensodrive, can be called to drive some torque and get sensor data
        super().__init__()
        self.parent_conn = input["parent_conn"]
        self.child_conn = input["child_conn"]
        self.senso_process = input["senso_drive"]
        self.t_warmup = input["warm_up_time"]
        self.t_cooldown = input["cooldown_time"]
        self.t_exp = input["experiment_time"]
        self.virtual_human_cost = input["virtual_human_cost"]
        self.sharing_rule = input["sharing_rule"]
        self.duration = self.t_warmup + self.t_exp + self.t_cooldown
        self.t_now = 0
        self.t_last = 0
        self.t0 = 0
        self.time = 0
        self.reference = input["reference"]
        self.store = False
        self.start_measurements = False
        self.virtual_human_gain = input["virtual_human_gain"]
        self.send_dict = {
            "ref": None,
            "factor": 0,
            "exit": False,
            "reset": False,
            "manual": input["manual"],
            "condition": 3,
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

        # Initialize message structures
        self.states = {}
        self.variables = dict()

        # Initialize pygame visualization
        self.visualize = Visualize(input["screen_width"], input["screen_height"], input["full_screen"])

        print("init complete!")

    def experiment(self):
        print("sleeping for 1.5 seconds to see if motor is active")
        bottom_text = ""
        self.visualize.visualize_experiment(0, [], 0, text="Initializing", top_text="", sigma=0, bottom_text=bottom_text)
        time.sleep(1.5)

        self.t0 = time.perf_counter_ns()
        self.t_last = time.perf_counter_ns()

        # Let's make this 1 loop
        while self.time < self.duration:

            # Check whether the process has not been quited
            self.visualize.check_quit()

            # Calculate timings
            self.t_now = time.perf_counter_ns()
            h = (self.t_now - self.t_last) * 1e-9
            self.t_last = self.t_now
            self.time = (self.t_last - self.t0) * 1e-9
            self.time_exp = self.time - self.t_warmup

            # Compute reference
            ref = self.reference.generate_reference(self.time, sigma=0, player="robot", ref_sign=0)
            # self.reference_preview(self.time, h, sigma_h=0.0)

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

                Qh, vhg = self.smooth_virtual_human()
                if self.virtual_human:
                    self.send_dict["virtual_human_gain"] = vhg

                estimated_gain_pos = self.states["estimated_human_gain"].flatten()[0]
                robot_gain_pos = self.states["robot_gain"][0, 0]
                virtual_gain = vhg[0]
                bottom_text = "Gains; H:" + str(round(estimated_gain_pos, 2)) + " R: " + str(round(robot_gain_pos, 2)) \
                        + " Virtual H: " + str(round(virtual_gain, 2))

                # Determine condition
                text = ""
                top_text = ""
                # bottom_text = ""
                # top_text = "Time = " + str(round(self.time - self.t_warmup, 1))

            # COOLDOWN
            else:
                # self.compute_error()
                self.store = False
                self.factor = -math.tanh(self.time - self.duration)
                self.send_dict["factor"] = self.factor
                self.send_dict["ref"] = ref
                self.send_dict["experiment"] = False
                text = "Finished trial"
                # bottom_text = "Performance Score (RMSE) = " + str(round(self.RMSE, 4))
                top_text = ""
                bottom_text = ""

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
                                                r_prev=[], text=text, top_text=top_text,
                                                bottom_text=bottom_text, sigma=0.0)

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
                    "repetition": 0,
                    "condition": "Robot only",
                    "human_noise": 0.0,
                    "setting": 0,
                }
                try:
                    output['torque'] = self.states["torque"][0, 0]
                except:
                    output["torque"] = 0

                output["estimated_human_gain_pos"] = self.states["estimated_human_gain"].flatten()[0]
                output["estimated_human_gain_vel"] = self.states["estimated_human_gain"].flatten()[1]

                # print(self.states["robot_gain"])
                output["robot_gain_pos"] = self.states["robot_gain"][0, 0]
                output["robot_gain_vel"] = self.states["robot_gain"][0, 1]

                output["virtual_human_gain_pos"] = vhg[0]
                output["virtual_human_gain_vel"] = vhg[1]
                output["virtual_human_cost_pos"] = Qh[0]
                output["virtual_human_cost_vel"] = Qh[1]

                output["state_estimate_pos"] = self.states["state_estimate"][0][0]
                output["state_estimate_vel"] = self.states["state_estimate"][1][0]
                output["input_estimation_error"] = self.states["input_estimation_error"]
                output["state_estimate_derivative"] = self.states["state_estimate_derivative"][1][0]

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


        # Time to close off
        self.send_dict["exit"] = True
        self.parent_conn.send(self.send_dict)
        self.visualize.quit()

        return self.variables

    def store_variables(self, output):
        for key in output.keys():
            self.variables.setdefault(key, []).append(output[key])

    def reference_preview(self, t_now, t_prev):
        steps = 200
        r = np.zeros(steps)
        t = t_now + np.array(range(steps))/steps * t_prev
        for i in range(steps):
            ref = self.reference.generate_reference(t[i], 0.0, "Robot", 0)
            r[i] = ref[0]
        return r

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
