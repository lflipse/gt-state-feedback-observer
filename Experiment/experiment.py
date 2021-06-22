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
        self.data_plotter_conn = input["send_conn"]
        self.senso_process = input["senso_drive"]
        self.t_warmup = input["warm_up_time"]
        self.t_cooldown = input["cooldown_time"]
        self.t_exp = input["experiment_time"]
        self.Qr_start = input["init_robot_cost"]
        self.Qr_end = input["final_robot_cost"]
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
        print("sleeping for 1.5 seconds to see if motor is active")
        self.visualize.visualize_experiment(0, [], 0, text="Initializing", top_text="")
        time.sleep(1.5)

        self.t0 = time.perf_counter_ns()
        self.t_last = time.perf_counter_ns()

        # Let's make this 1 loop
        while self.time < self.duration:
            # Check whether the process has not been quited
            self.visualize.check_quit()

            self.t_now = time.perf_counter_ns()
            h = (self.t_now - self.t_last) * 1e-9
            self.t_last = self.t_now
            self.time = (self.t_last - self.t0) * 1e-9
            ref = self.reference.generate_reference(self.time)

            # WARM_UP
            if self.time < self.t_warmup:
                # 3 second start screen
                self.store = False
                self.send_dict["factor"] = 1 / (1 + np.exp(-2 * self.time)) - 1 / (1 + np.exp(2 * self.time))
                self.send_dict["ref"] = ref * self.send_dict["factor"]
                self.send_dict["experiment"] = False
                self.send_dict["robot_cost"] = self.Qr_start
                r_prev = self.reference_preview(self.time, t_prev=1)
                dt = self.t_warmup - self.time
                top_text = ""
                if dt > 3:
                    text = "Experiment starts in:"
                else:
                    text = str(round(dt, 1))

            # EXPERIMENT
            elif self.time >= self.t_warmup and self.time < (self.t_warmup + self.t_exp):
                self.store = True
                self.send_dict["factor"] = 1
                self.send_dict["ref"] = ref
                self.send_dict["experiment"] = True
                self.send_dict["robot_cost"] = 0.5 * ((self.Qr_start + self.Qr_end) + np.tanh(self.time - self.t_warmup - 0.5 * self.t_exp) * (self.Qr_end - self.Qr_start))
                r_prev = self.reference_preview(self.time, t_prev=1)

                if self.time - self.t_warmup < 1/6 * self.t_exp:
                    vhg = np.array(self.virtual_human_gain[0, :])
                elif 2 / 6 * self.t_exp > self.time - self.t_warmup > 1 / 6 * self.t_exp:
                    vhg = np.array(self.virtual_human_gain[1, :])
                elif 3 / 6 * self.t_exp > self.time - self.t_warmup > 2 / 6 * self.t_exp:
                    vhg = np.array(self.virtual_human_gain[2, :])
                elif 4 / 6 * self.t_exp > self.time - self.t_warmup > 3 / 6 * self.t_exp:
                    vhg = np.array(self.virtual_human_gain[3, :])
                elif 5 / 6 * self.t_exp > self.time - self.t_warmup > 4 / 6 * self.t_exp:
                    vhg = np.array(self.virtual_human_gain[4, :])
                else:
                    vhg = np.array(self.virtual_human_gain[5, :])

                if self.virtual_human:
                    self.send_dict["virtual_human_gain"] = vhg

                text = ""
                top_text = "Time = " + str(round(self.time - self.t_warmup, 1))
                # Save variables

            # COOLDOWN
            else:
                self.store = False
                self.send_dict["factor"] = 0.96 * self.send_dict["factor"]
                self.send_dict["ref"] = 0.96 * self.send_dict["ref"]
                text = "Finished Experiment"
                self.send_dict["experiment"] = False
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
                                                r_prev=r_prev, text=text, top_text=top_text)

            if self.store:
                output = {
                    "steering_angle": self.states["steering_angle"],
                    "steering_rate": self.states["steering_rate"],
                    "angle_error": self.states["angle_error"],
                    "rate_error": self.states["rate_error"],
                    "acceleration": self.states["steering_acc"],
                    "cost": self.states["cost"],
                    "reference": ref,
                    "torque": self.states["torque"].flatten(),
                    "measured_input": self.states["measured_input"],
                    "execution_time": h,
                    "time": self.time - self.t_warmup,
                }
                if self.controller_type == "Gain_observer":
                    output["estimated_human_gain_pos"] = self.states["estimated_human_gain"][0, 0]
                    output["estimated_human_gain_vel"] = self.states["estimated_human_gain"][0, 1]
                    # print(self.states["robot_gain"])
                    output["robot_gain_pos"] = self.states["robot_gain"][0, 0]
                    output["robot_gain_vel"] = self.states["robot_gain"][0, 1]
                    output["state_estimate"] = self.states["state_estimate"]
                    output["estimated_human_input"] = self.states["estimated_human_torque"].flatten()
                    output["input_estimation_error"] = self.states["input_estimation_error"].flatten()
                    output["virtual_human_torque"] = self.states["virtual_human_torque"]
                    output["xi_gamma"] = self.states["xi_gamma"]
                    output["state_estimate_derivative"] = self.states["state_estimate_derivative"]
                    output["virtual_human_gain_pos"] = vhg[0]
                    output["virtual_human_gain_vel"] = vhg[1]

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
            ref = self.reference.generate_reference(t[i])
            r[i] = ref[0]
        return r

