import time
import numpy as np
import wres
import platform

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
        # self.data_plotter_conn = input["send_conn"]
        self.senso_process = input["senso_drive"]
        self.t_warmup = input["warm_up_time"]
        self.t_cooldown = input["cooldown_time"]
        self.t_exp = input["experiment_time"]
        self.Qr_start = input["init_robot_cost"]
        self.Qr_end = input["final_robot_cost"]
        self.virtual_human_cost = input["virtual_human_cost"]
        self.sharing_rule = input["sharing_rule"]
        self.conditions = input["conditions"]
        print("conditions --> ", self.conditions)
        self.human_noise = input["human_noise"]
        self.role = input["roles"]
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
        self.visualize = Visualize(input["screen_width"], input["screen_height"], self.preview, input["full_screen"])

        print("init complete!")

    def experiment(self):
        print("sleeping for 2 seconds to see if motor is active")
        self.visualize.visualize_experiment(0, [], 0, text="Initializing", top_text="", sigma=0, role="")
        time.sleep(2)

        self.t0 = time.perf_counter_ns()
        self.t_last = time.perf_counter_ns()

        # Loop over x conditions
        for i in range(len(self.conditions)):

            # First check if participant is ready
            # TODO
            self.visualize.visualize_experiment(0, [], 0, text="Initializing", top_text="", sigma=0, role="")

            # Loop over 1 trial
            while self.time < self.duration:

                # Check whether the process has not been quited
                self.visualize.check_quit()

                self.t_now = time.perf_counter_ns()
                h = (self.t_now - self.t_last) * 1e-9
                self.t_last = self.t_now
                self.time = (self.t_last - self.t0) * 1e-9
                ref = self.reference.generate_reference(self.time, sigma=0, player="robot")

                # WARM_UP
                if self.time < self.t_warmup:
                    # 3 second start screen
                    self.store = False
                    self.send_dict["factor"] = 1 / (1 + np.exp(-2 * self.time)) - 1 / (1 + np.exp(2 * self.time))
                    self.send_dict["ref"] = ref * self.send_dict["factor"]
                    self.send_dict["experiment"] = False
                    self.send_dict["robot_cost"] = self.Qr_start
                    self.send_dict["sharing_rule"] = self.sharing_rule
                    r_prev = self.reference_preview(self.time, t_prev=1, sigma_h=self.human_noise[self.cond])
                    dt = self.t_warmup - self.time
                    top_text = ""
                    if dt > 3:
                        text = "Experiment starts in:"
                    else:
                        text = str(round(dt, 1))

                # EXPERIMENT
                elif self.t_warmup <= self.time < (self.t_warmup + self.t_exp):
                    self.store = True
                    self.send_dict["factor"] = 1
                    self.send_dict["ref"] = ref
                    self.send_dict["experiment"] = True
                    self.send_dict["robot_cost"] = self.sharing_rule
                    self.send_dict["sharing_rule"] = self.sharing_rule
                    r_prev = self.reference_preview(self.time, t_prev=1, sigma_h=self.human_noise[self.cond])

                    if self.time - self.t_warmup > 0.2 * self.t_exp:
                        if self.time - self.t_warmup < 0.8 * self.t_exp:
                            self.cond = self.conditions[i]
                        else:
                            self.cond = 0

                    text = ""
                    top_text = "Time = " + str(round(self.time - self.t_warmup, 1))

                # COOLDOWN
                else:
                    self.cond = 0
                    self.store = False
                    self.send_dict["factor"] = 0.96 * self.send_dict["factor"]
                    self.send_dict["ref"] = 0.96 * self.send_dict["ref"]
                    text = "Finished Experiment"
                    self.send_dict["experiment"] = False
                    self.send_dict["sharing_rule"] = self.sharing_rule
                    r_prev = []
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

                # Visualize experiment
                self.visualize.visualize_experiment(self.send_dict["ref"][0], angle=self.states["steering_angle"],
                                                    r_prev=r_prev, text=text, top_text=top_text,
                                                    role=self.role[self.cond], sigma=self.human_noise[self.cond])

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
                        "torque": self.states["torque"][0, 0],
                        "measured_input": self.states["measured_input"],
                        "execution_time": h,
                        "condition": self.cond,
                        "human_noise": self.human_noise[self.cond],
                        "role": self.role[self.cond],
                    }
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
        self.send_dict["exit"] = True
        self.parent_conn.send(self.send_dict)
        self.visualize.quit()

        return self.variables

    def store_variables(self, output):
        for key in output.keys():
            self.variables.setdefault(key, []).append(output[key])

    def reference_preview(self, t_now, t_prev, sigma_h):
        steps = 200
        r = np.zeros(steps)
        t = t_now + np.array(range(steps))/steps * t_prev
        for i in range(steps):
            ref = self.reference.generate_reference(t[i], sigma_h, player="human")
            r[i] = ref[0]
        return r

