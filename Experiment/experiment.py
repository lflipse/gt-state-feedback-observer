# import pygame
import multiprocessing as mp
import time
import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as cp


from Experiment.visuals import Visualize

class Experiment:
    def __init__(self, Bw, Kw, parent_conn, child_conn, screen_width, screen_height, controller, controller_type):
        # 2 OBJECTS TO DRIVE THE EXPERIMENT
        # 1. Sensodrive, can be called to drive some torque and get sensor data
        super().__init__()
        self.parent_conn = parent_conn
        self.child_conn = child_conn
        self.send_dict = {"torque": 0,
            "damping": Bw,
            "stiffness": Kw,
            "exit": False}

        self.damping = Bw
        self.stiffness = Kw
        self.damp = 0
        self.stiff = 0
        self.new_states = {"steering_angle": 0, "measured_torque": 0}

        # 2. Controller, which manages the inputs to the sensodrive
        self.controller = controller
        self.controller_type = controller_type

        # Initialize message structures
        self.variables = dict()
        self.angle_array = []
        self.xddot_array = []

        # Filter values


        # Initialize pygame visualization
        # self.visualize = Visualize(screen_width, screen_height)

        # States
        self.states = {"steering_angle": 0.0,
                       "steering_rate": 0.0,}
        self.x = np.array([0.0, 0.0])
        self.estimated_human_gain = np.array([0.0, 0.0])
        self.torque = 0
        self.ref = 0

        print("init complete!")

    def experiment(self, T, r, N, t_step):
        # Initialize the senso_drive
        self._time_step_in_ns = t_step * 1e9
        self.states['steering_angle'] = 0.0
        self.states['steering_rate'] = 0.0
        self.estimated_human_gain = np.array([0.0, 0.0])

        # 3 second start screen
        self.warm_up(duration=3.0, t_step=t_step)

        self.angle_array = [self.states["steering_rate"]]
        self.xdot_array = [0]

        for i in range(N):
            t0 = time.perf_counter_ns()

            # self.visualize.check_quit()

            # Let's first start by reading out some values
            x = np.array([self.states["steering_angle"], self.states["steering_rate"]])

            # Calculate derivative(s) of reference
            h = t_step
            if i > 1:
                ref = np.array([r[i], (r[i] - r[i - 1]) / h])
            else:
                ref = np.array([r[i], (r[i]) / (2 * h)])

            # Compute error states
            xi = x - ref

            x_ddot = (x[1] - self.x[1]) / t_step
            x_ddot_filt, self.xddot_array = self.filter(x_ddot, self.xddot_array)
            self.x = x

            # TODO: Mooier maken!
            max_torque = 20.0
            if self.controller_type != "Manual":
                if self.controller_type == "Gain_observer":
                    torque, estimated_human_torque, robot_gain, estimated_human_gain, uhtilde, cost = \
                        self.controller.compute_control_input(xi, x, x_ddot_filt, self.estimated_human_gain, t_step)
                    self.estimated_human_gain = estimated_human_gain
                    if float(torque) > max_torque:
                        # print(torque)
                        torque = max_torque
                        print("torque is too high")
                    elif float(torque) < -max_torque:
                        print(torque)
                        torque = -max_torque
                        print("torque is too high")
                else:
                    torque, cost = self.controller.compute_control_input(xi)
                    if torque > max_torque:
                        torque = max_torque
                        print("torque is too high")
                    elif float(torque) < -max_torque:
                        print(torque)
                        torque = -max_torque
                        print("torque is too high")
            else:
                torque = 0
                cost = 0

            self.torque = torque

            # Update states
            self.send_dict["torque"] = torque
            self.send_dict["damping"] = self.damping
            self.send_dict["stiffness"] = self.stiffness
            self.parent_conn.send(self.send_dict)  # Child is for sending
            new_states = self.parent_conn.recv()  # Receive from child

            if new_states != None:
                self.new_states = new_states
            new_speed = (self.new_states["steering_angle"] - self.states["steering_angle"]) / t_step
            self.states["steering_rate"], self.angle_array = self.filter(new_speed, self.angle_array)
            self.states["steering_angle"] = self.new_states["steering_angle"]
            real_torque = self.new_states["measured_torque"]


            # Visualize and rest
            # self.visualize.visualize_experiment(r[i], x[0])
            self.ref = r[i]
            execution_time = time.perf_counter_ns() - t0
            # print(execution_time * 1e-9)
            time.sleep(max(0.0, (self._time_step_in_ns - execution_time) * 1e-9))

            output = {
                "position": x[0],
                "velocity": x[1],
                "error": xi[0],
                "error_vel": xi[1],
                "acceleration": x_ddot_filt,
                "cost": cost,
                "reference": ref[0],
                "reference_vel": ref[1],
                "input": torque,
                "measured_input": real_torque,
                "execution_time": execution_time * 1e-9,
                "time": i*t_step,
            }
            if self.controller_type == "Gain_observer":
                # print(self.estimated_human_gain)
                output["estimated_human_gain_pos"] = self.estimated_human_gain[0, 0]
                output["estimated_human_gain_vel"] = self.estimated_human_gain[0, 1]
                output["robot_gain_pos"] = robot_gain[0, 0]
                output["robot_gain_vel"] = robot_gain[0, 1]
                output["estimated_human_input"] = estimated_human_torque
                output["input_estimation_error"] = uhtilde[0]
            self.store_variables(output)

        # Cool down for 3 seconds
        self.cool_down(duration=3.0, t_step=t_step)
        self.send_dict["exit"] = True
        self.parent_conn.send(self.send_dict)  # Child is for s

        # self.visualize.quit()

        return self.variables

    def store_variables(self, output):
        for key in output.keys():
            self.variables.setdefault(key, []).append(output[key])

    def filter(self, value_new, value_array):
        # TODO: moet beter dit!
        max_len = 50
        value_array.append(value_new)
        t = range(len(value_array))
        if len(value_array) > max_len:
            del value_array[0]
        if len(value_array) > 27:
            fc = 10  # Cut-off frequency of the filter
            fs = 1 / 0.015
            w = fc / (fs / 2)  # Normalize the frequency
            b, a = cp.butter(8, w, 'low')
            v_array_filt = cp.filtfilt(b, a, value_array)
        else:
            v_array_filt = value_array

        return v_array_filt[-1], value_array

    def warm_up(self, duration, t_step):
        # Slowly activate the damping and stiffness to desired values and show countdown screen
        N = int(duration/t_step)
        for j in range(N):
            # First three seconds countdown
            t0 = time.perf_counter_ns()
            # self.visualize.check_quit()
            torque = 0
            damp = self.damping * j * t_step / 1
            stiff = self.stiffness * j * t_step / 1
            self.damp = min(damp, self.damping)
            self.stiff = min(stiff, self.stiffness)

            # Update states
            self.send_dict["torque"] = torque
            self.send_dict["damping"] = self.damp
            self.send_dict["stiffness"] = self.stiff
            self.parent_conn.send(self.send_dict)  # Child is for sending
            new_states = self.parent_conn.recv()  # Receive from child

            if self.new_states == None:
                print("Double check with error")
                self.states = self.states
            else:
                self.states["steering_rate"] = (self.new_states["steering_angle"] - self.states["steering_angle"]) / t_step
                self.states["steering_angle"] = self.new_states["steering_angle"]

            # self.visualize.visualize_experiment(0, angle=self.new_states["steering_angle"])
            execution_time = time.perf_counter_ns() - t0
            # print(execution_time * 1e-9)
            time.sleep(max(0.0, (self._time_step_in_ns - execution_time) * 1e-9))


    def cool_down(self, duration, t_step):
        # Slowly deactivate the damping and stiffness to zero values and show countdown screen
        N = int(duration / t_step)
        for j in range(N):
            # First three seconds countdown
            t0 = time.perf_counter_ns()
            # self.visualize.check_quit()
            torque = 0
            damp = self.damping - self.damping * j * t_step / duration
            stiff = self.stiffness - self.stiffness * j * t_step / duration
            self.damp = min(damp, self.damping)
            self.stiff = min(stiff, self.stiffness)

            # Update states
            self.send_dict["torque"] = torque
            self.send_dict["damping"] = self.damp
            self.send_dict["stiffness"] = self.stiff
            self.parent_conn.send(self.send_dict)  # Child is for sending
            new_states = self.parent_conn.recv()  # Receive from child

            if self.new_states == None:
                print("Double check with error")
                self.states = self.states
            else:
                self.states["steering_rate"] = (self.new_states["steering_angle"] - self.states[
                    "steering_angle"]) / t_step
                self.states["steering_angle"] = self.new_states["steering_angle"]
            ref = 0.98*self.ref
            self.ref = ref
            # self.visualize.visualize_experiment(ref, angle=self.new_states["steering_angle"])
            execution_time = time.perf_counter_ns() - t0
            # print(execution_time * 1e-9)
            time.sleep(max(0.0, (self._time_step_in_ns - execution_time) * 1e-9))

