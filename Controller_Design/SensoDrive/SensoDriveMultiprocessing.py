import time
from Controller_Design.SensoDrive.PCANBasic import *
from Controller_Design.SensoDrive.Lowpassfilter_Biquad import LowPassFilterBiquad
import math
import multiprocessing as mp
import numpy as np



class SensoDriveModule(mp.Process):
    def __init__(self, senso_dict):

        super().__init__(daemon=True)
        # Establish communication
        self.parent_channel = senso_dict["parent_conn"]
        self.child_channel = senso_dict["child_conn"]
        self.message = TPCANMsg()
        self._pcan_channel = None
        self.exit = False
        self.frequency = 500  # Hz
        self.time_step = 1 / self.frequency
        self._time_step_in_ns = self.time_step * 1e9
        self._bq_filter_velocity = LowPassFilterBiquad(fc=60, fs=self.frequency)
        self._bq_filter_acc = LowPassFilterBiquad(fc=15, fs=self.frequency)
        self.controller = senso_dict["controller"]
        self.controller_type = senso_dict["controller_type"]
        self.now = 0
        self.last_time = 0

        self.count_loop = 0
        self.count_senso = 0
        self.received_ok = 1

        # Steering wheel setting
        self.settings = {
            'factor': 0,
            'mp_friction': 0,
            'mp_damping': senso_dict["damping"],
            'mp_spring_stiffness': senso_dict["stiffness"],
            'manual': True,
            'static': False,
        }

        # Cost parameters
        self.beta = senso_dict["beta"]
        self.alpha_1 = senso_dict["alpha_1"]
        self.alpha_2 = senso_dict["alpha_2"]
        self.reset_states = False

        try:
            self.initial_human = senso_dict["initial_human"]
        except:
            senso_dict["initial_human"] = np.array([0, 0])

        print(self.initial_human)

        # States
        self.states = {
            "ref": np.array([0, 0]),
            "steering_angle": 0,
            "steering_rate": 0,
            "steering_rate_unf": 0,
            "steering_acc": 0,
            "state": np.array([[0], [0]]),
            "state_derivative": np.array([[0], [0]]),
            "state_estimate": np.array([[0], [0]]),
            "state_estimate_derivative": np.array([[0], [0]]),
            "angle_error": 0,
            "rate_error": 0,
            "error_state": np.array([[0], [0]]),
            "torque": 0,
            "estimated_human_torque": 0,
            "cost": 0,
            "measured_input": 0,
            "estimated_human_gain": self.initial_human,
            "estimated_human_gain_derivative": np.array([0, 0]),
            "virtual_human_gain": np.array([0.0, 0.0]),
            "virtual_human_torque": 0,
            "robot_gain": np.array([[0, 0]]),
            "input_estimation_error": 0,
            "xi_gamma": np.array([0, 0]),
            "experiment": False,
            "xdot_test": np.array([[0], [0]]),
            "robot_cost": np.array([[0, 0], [0, 0]]),
            "estimated_human_cost": np.array([[0, 0], [0, 0]]),
            "robot_P": np.array([[0, 0], [0, 0]]),
            "sharing_rule": np.array([[0, 0], [0, 0]]),
            "robot_cost_calc": np.array([[0, 0], [0, 0]])
        }

        # Hex Codes
        self.INITIALIZATION_MESSAGE_ID = 0x200
        self.INITIALIZATION_MESSAGE_LENGTH = 8
        self.STATE_MESSAGE_RECEIVE_ID = 0x210
        self.STEERINGWHEEL_MESSAGE_SEND_ID = 0x201
        self.STEERINGWHEEL_MESSAGE_RECEIVE_ID = 0x211
        self.STEERINGWHEEL_MESSAGE_LENGTH = 8

        endstops_bytes = int.to_bytes(int(math.degrees(180)), 2, byteorder='little',
                                      signed=True)
        torque_limit_between_endstops_bytes = int.to_bytes(120, 1,
                                                           byteorder='little',
                                                           signed=False)
        torque_limit_beyond_endstops_bytes = int.to_bytes(100, 1,
                                                          byteorder='little',
                                                          signed=False)

        self.settings_operation = {}
        self.settings_operation[0] = 0x1F
        self.settings_operation[1] = 0
        # Endstop position
        self.settings_operation[2] = endstops_bytes[0]
        self.settings_operation[3] = endstops_bytes[1]
        # reserved
        self.settings_operation[4] = 0
        self.settings_operation[5] = 0
        # Torque between endstops:
        self.settings_operation[6] = torque_limit_between_endstops_bytes[0]
        # Torque beyond endstops:
        self.settings_operation[7] = torque_limit_beyond_endstops_bytes[0]

    # Function to send torque and read out states
    def run(self):
        print("running process has started")
        self.initialize()
        self.last_time = time.perf_counter_ns()

        while not self.exit:
            if self.reset_states:
                self.reset()
            self.count_senso += 1

            # Compute control input using states, read out data and update states
            sensor_data = self.write_and_read(msgtype="201", data=self.settings)

            # Update timestamp, calculate interval and update states
            self.now = time.perf_counter_ns()
            delta = (self.now - self.last_time) * 1e-9
            self.last_time = self.now
            self.update_states(sensor_data, delta)

            # When data is available, receive new reference value
            # Feedback states to main loop
            data_available = self.child_channel.poll()
            if data_available is True:
                self.count_loop += 1
                msg = self.child_channel.recv()
                self.states["ref"] = msg["ref"]

                try:
                    self.states["virtual_human_gain"] = msg["virtual_human_gain"]
                except:
                    self.states["virtual_human_gain"] = self.states["virtual_human_gain"]

                try:
                    self.states["sharing_rule"] = msg["sharing_rule"]
                except:
                    self.states["sharing_rule"] = self.states["sharing_rule"]
                    print("sharing rule not succesfully recieved")

                try:
                    self.states["robot_cost"] = msg["robot_cost"]
                except:
                    self.states["robot_cost"] = self.states["sharing_rule"] - self.states["estimated_human_cost"]

                self.settings["factor"] = msg["factor"]
                self.settings["manual"] = msg["manual"]
                self.settings["static"] = msg["static"]
                self.states["experiment"] = msg["experiment"]
                self.exit = msg["exit"]
                self.reset_states = msg["reset"]
                self.child_channel.send(self.states)

        print("sent ", self.count_senso, " messages to sensodrive")
        print("received ", self.count_loop, " messages from controller")
        self.uninitialize()


    # Function to send torque and read out states
    def read(self):
        sensor_data = self.write_and_read(msgtype="201", data=None)
        return sensor_data

    def initialize(self):
        """
        Initializes the PCAN Dongle and sends appropriate initialization messages to the SensoDrive.
        :return:
        """
        self._pcan_channel = PCAN_USBBUS1
        # Here we can initialize our PCAN Communication (WE HAVE TO DO THIS HERE ELSE WE WONT HAVE THE PCAN
        # OBJECT IN OUR DESIRED PROCESS)

        self.pcan_object = PCANBasic()
        self.pcan_initialization_result = self.pcan_object.Initialize(self._pcan_channel, PCAN_BAUD_1M)

        # Start the controller
        self.write_and_read(msgtype="2001F", data=None)
        self.write_and_read(msgtype="20012", data=None)
        self.write_and_read(msgtype="20014", data=None)
        print("Initialization succesful!")

        # TODO : FIXX ---->> when motor is not turned on stop process and give warning.

        # TODO: Referencing mode to align steering wheel to 0 position!

    def uninitialize(self):
        """
        Initializes the PCAN Dongle and sends appropriate initialization messages to the SensoDrive.
        :return:
        """

        # Stop the controller
        self.write_and_read(msgtype="20014", data=None)
        self.write_and_read(msgtype="20012", data=None)
        self.write_and_read(msgtype="2001F", data=None)
        self.pcan_initialization_result = self.pcan_object.Uninitialize(self._pcan_channel)

        print("Uninitialization succesful!")

        # TODO: Referencing mode to align steering wheel to 0 position!

    def reset(self):
        self.states = {
            "ref": np.array([0, 0]),
            "steering_angle": 0,
            "steering_rate": 0,
            "steering_rate_unf": 0,
            "steering_acc": 0,
            "state": np.array([[0], [0]]),
            "state_derivative": np.array([[0], [0]]),
            "state_estimate": np.array([[0], [0]]),
            "state_estimate_derivative": np.array([[0], [0]]),
            "angle_error": 0,
            "rate_error": 0,
            "error_state": np.array([[0], [0]]),
            "torque": 0,
            "estimated_human_torque": 0,
            "cost": 0,
            "measured_input": 0,
            "estimated_human_gain": self.initial_human,
            "estimated_human_gain_derivative": np.array([0, 0]),
            "virtual_human_gain": np.array([0.0, 0.0]),
            "virtual_human_torque": 0,
            "robot_gain": np.array([[0, 0]]),
            "input_estimation_error": 0,
            "xi_gamma": np.array([0, 0]),
            "experiment": False,
            "xdot_test": np.array([[0], [0]]),
            "robot_cost": np.array([[0, 0], [0, 0]]),
            "estimated_human_cost": np.array([[0, 0], [0, 0]]),
            "robot_P": np.array([[0, 0], [0, 0]]),
            "sharing_rule": np.array([[0, 0], [0, 0]]),
            "robot_cost_calc": np.array([[0, 0], [0, 0]]),
        }


    def update_states(self, sensor_data, delta):
        steering_angle = sensor_data["steering_angle"]
        steering_rate = (steering_angle - self.states["steering_angle"]) / delta
        steering_acc = (steering_rate - self.states["steering_rate_unf"]) / delta
        self.states["steering_rate"] = self._bq_filter_velocity.step(steering_rate)
        self.states["steering_rate_unf"] = steering_rate
        self.states["steering_acc"] = self._bq_filter_acc.step(steering_acc)
        self.states["steering_angle"] = steering_angle
        self.states["state"] = np.array([[self.states["steering_angle"]],
                                        [self.states["steering_rate"]]])
        self.states["error_state"] = np.array([[self.states["steering_angle"] - self.states["ref"][0]],
                                               [self.states["steering_rate"] - self.states["ref"][1]]])
        self.states["angle_error"] = self.states["error_state"][0]
        self.states["rate_error"] = self.states["error_state"][1]

        estimate_derivative = np.array(self.states["state_estimate_derivative"])
        old_estimated_state = np.array(self.states["state_estimate"])
        new_estimated_state = old_estimated_state + estimate_derivative * delta
        self.states["state_estimate"] = new_estimated_state

        old_estimated_gain = np.array(self.states["estimated_human_gain"])
        gain_derivative = np.array(self.states["estimated_human_gain_derivative"])
        new_estimated_gain = old_estimated_gain + gain_derivative * delta
        self.states["estimated_human_gain"] = new_estimated_gain

        self.states["state_derivative"] = np.array([[self.states["steering_rate"]],
                                                    [self.states["steering_acc"]]])

        self.states["estimated_human_cost"] = self.compute_cost()

        if not self.states["experiment"]:
            self.states["estimated_human_gain"] = self.initial_human
            self.states["estimated_human_cost"] = np.array([[0.0, 0.0], [0.0, 0.0]])

        # Cap for safety reasons
        try:
            self.states["estimated_human_gain"][0, 1] = max(-0.5, min(2, self.states["estimated_human_gain"][0, 1]))
            self.states["estimated_human_gain"][0, 0] = max(-4, min(8, self.states["estimated_human_gain"][0, 0]))
        except:
            self.states["estimated_human_gain"][1] = max(-0.5, min(2, self.states["estimated_human_gain"][1]))
            self.states["estimated_human_gain"][0] = max(-4, min(8, self.states["estimated_human_gain"][0]))

    def compute_cost(self):
        Lr = self.states["robot_gain"][0]
        Lhat = self.states["estimated_human_gain"][0]
        p = 1 / self.beta * Lhat
        gamma_1 = self.alpha_1 - self.beta * Lr[0]
        gamma_2 = self.alpha_2 - self.beta * Lr[1]
        q_hhat1 = - 2 * gamma_1 * p[0] + Lhat[0] ** 2
        q_hhat2 = - 2 * p[0] - 2 * gamma_2 * p[1] + Lhat[1] ** 2
        Q_hhat = np.array([[q_hhat1, 0], [0, q_hhat2]])
        return Q_hhat

    def write_and_read(self, msgtype, data):
        # Check message type
        if msgtype == "2001F":
            # State message (QUIT_ERROR)
            self.message.ID = 0x200
            message = self.settings_operation
            message[0] = 0x1F
        elif msgtype == "20012":
            # State message (READY)
            self.message.ID = 0x200
            message = self.settings_operation
            message[0] = 0x12
        elif msgtype == "20014":
            # State message (ON)
            self.message.ID = 0x200
            message = self.settings_operation
            message[0] = 0x14
        elif msgtype == "20011":
            # State message (ON)
            self.message.ID = 0x200
            message = self.settings_operation
            message[0] = 0x11
        elif msgtype == "201":
            # State message
            self.message.ID = 0x201
            message = self.map_si_to_sensodrive(data)
        else:
            exit("Something went wrong in sending a message")
        self.message.LEN = 8
        self.message.TYPE = PCAN_MESSAGE_STANDARD

        # Create message
        self.message.DATA[0] = message[0]
        self.message.DATA[1] = message[1]
        self.message.DATA[2] = message[2]
        self.message.DATA[3] = message[3]
        self.message.DATA[4] = message[4]
        self.message.DATA[5] = message[5]
        self.message.DATA[6] = message[6]
        self.message.DATA[7] = message[7]

        self.pcan_object.Write(self._pcan_channel, self.message)

        self.received_ok = 1
        t0 = time.time()
        while self.received_ok > 0:
            received = self.pcan_object.Read(self._pcan_channel)
            self.received_ok = received[0]
            if time.time() - t0 > 1.0:
                # TODO: Fix that main process also quits
                exit("This should not take this long. Check if the motor is connected!")
                return -1

        output = self.map_sensodrive_to_si(received)

        return output

    def virtual_human(self):
        self.states["virtual_human_torque"] = np.matmul(-self.states["virtual_human_gain"], self.states["error_state"])
        self.states["torque"] += self.states["virtual_human_torque"]


    def map_si_to_sensodrive(self, settings):
        """
        Converts settings to sensodrive message
        """

        # Compute the control input
        output = self.controller.compute_control_input(self.states, self.settings["manual"], self.settings["static"])
        self.update_controller_states(output)
        if self.settings["manual"]:
            self.states["torque"] = 0

        self.virtual_human()

        # Limit torque
        if settings != None:
            # print(settings['mp_torque'])
            # Limit the torque manually if it goes over a barrier of 10 Nm and output warning!
            if self.states["torque"] > 15:
                self.states["torque"] = 15
                print("Torque is too high, what happend?")
            elif self.states["torque"] < -15:
                self.states["torque"] = -15
                print("Torque is too high, what happend?")

            a = self.settings["factor"]
            torque = int(a * self.states["torque"] * 1000.0)
            friction = int(settings['mp_friction'] * 1000.0)
            damping = int(settings['mp_damping'] * 1000.0 * (2.0 * math.pi) / 60.0)
            spring_stiffness = int(settings['mp_spring_stiffness'] * 1000.0 / (180.0 / math.pi))
            torque_bytes = int.to_bytes(torque, 2, byteorder='little', signed=True)
            friction_bytes = int.to_bytes(friction, 2, byteorder='little', signed=True)
            damping_bytes = int.to_bytes(damping, 2, byteorder='little', signed=True)
            spring_stiffness_bytes = int.to_bytes(spring_stiffness, 2, byteorder='little', signed=True)
        else:
            torque_bytes = int.to_bytes(0, 2, byteorder='little', signed=True)
            friction_bytes = int.to_bytes(0, 2, byteorder='little', signed=True)
            damping_bytes = int.to_bytes(0, 2, byteorder='little', signed=True)
            spring_stiffness_bytes = int.to_bytes(0, 2, byteorder='little', signed=True)

        data = {}
        data[0] = torque_bytes[0]
        data[1] = torque_bytes[1]
        data[2] = friction_bytes[0]
        data[3] = friction_bytes[1]
        data[4] = damping_bytes[0]
        data[5] = damping_bytes[1]
        data[6] = spring_stiffness_bytes[0]
        data[7] = spring_stiffness_bytes[1]

        return data

    def update_controller_states(self, output):
        self.states["torque"] = output["torque"]

        # Update gains
        if self.controller_type == "Gain_observer" or self.controller_type == "Cost_observer":
            # Update human gain estimate and calculate torque
            if not self.states["experiment"]:
                self.states["estimated_human_gain"] = np.array([0, 0])
                self.states["estimated_human_cost"] = np.array([[0, 0], [0, 0]])
                self.states["state_estimate"] = self.states["state"]
                self.states["estimated_human_gain_derivative"] = np.array([0, 0])
            self.states["state_estimate_derivative"] = output["state_estimate_derivative"]
            self.states["estimated_human_gain_derivative"] = output["estimated_human_gain_derivative"]
            self.states["estimated_human_torque"] = output["estimated_human_torque"]
            self.states["input_estimation_error"] = output["input_estimation_error"]
            self.states["robot_gain"] = output["robot_gain"]
            self.states["robot_P"] = output["robot_P"]
            self.states["robot_cost_calc"] = output["robot_cost"]


    def map_sensodrive_to_si(self, received):
        """
        Converts sensodrive data to actual SI Units
        :param received: sensodrive unit filled dictionary
        :return:
        """
        message = {}
        if received[1].ID == self.STEERINGWHEEL_MESSAGE_RECEIVE_ID:
            # steering wheel

            # steering angle
            increments = int.from_bytes(received[1].DATA[0:4], byteorder='little', signed=True)
            message['steering_angle'] = math.radians(
                float(increments) * 0.009)  # we get increments, convert to deg, convert to rad

            # steering rate
            steering_rate = int.from_bytes(received[1].DATA[4:6], byteorder='little', signed=True)
            message['steering_rate'] = float(steering_rate) * (
                    2.0 * math.pi) / 60.0  # we get rev/min, convert to rad/s

            # torque
            torque = int.from_bytes(received[1].DATA[6:], byteorder='little', signed=True)
            message['measured_torque'] = float(torque) / 1000.0  # we get mNm convert to Nm

        elif received[1].ID == self.STATE_MESSAGE_RECEIVE_ID:
            self._current_state_hex = received[1].DATA[0]
            message['sensodrive_motorstate'] = received[1].DATA[0]

        else:
            print('Missing values')
            message['steering_angle'] = 0
            message['steering_rate'] = 0

        return message
