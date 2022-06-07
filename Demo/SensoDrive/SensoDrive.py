import time
from Controller_Design.SensoDrive.PCANBasic import *
import math


class SensoDriveModule:
    def __init__(self, Bw, Kw):
        # Establish communication
        self.message = TPCANMsg()
        self._pcan_channel = None

        # Steering wheel setting
        self.settings = {}
        self.settings['mp_torque'] = 0
        self.settings['mp_friction'] = 0
        self.settings['mp_damping'] = Bw
        self.settings['mp_spring_stiffness'] = Kw

        # Hex Codes
        self.INITIALIZATION_MESSAGE_ID = 0x200
        self.INITIALIZATION_MESSAGE_LENGTH = 8
        self.STATE_MESSAGE_RECEIVE_ID = 0x210
        self.STEERINGWHEEL_MESSAGE_SEND_ID = 0x201
        self.STEERINGWHEEL_MESSAGE_RECEIVE_ID = 0x211
        self.STEERINGWHEEL_MESSAGE_LENGTH = 8

        # TODO: Fix settings this way!
        endstops_bytes = int.to_bytes(int(math.degrees(180)), 2, byteorder='little',
                                      signed=True)
        torque_limit_between_endstops_bytes = int.to_bytes(100, 1,
                                                           byteorder='little',
                                                           signed=False)
        torque_limit_beyond_endstops_bytes = int.to_bytes(100, 1,
                                                          byteorder='little',
                                                          signed=False)

        self.settings_operation = {}
        self.settings_operation[0] = 0x1F
        self.settings_operation[1] = 0
        # Endstop position
        self.settings_operation[2] = 0
        self.settings_operation[3] = 0xB4
        # reserved
        self.settings_operation[4] = 0
        self.settings_operation[5] = 0
        # Torque between endstops:
        self.settings_operation[6] = 0x14
        # Torque beyond endstops:
        self.settings_operation[7] = 0x14

    # Function to send torque and read out states
    def drive(self, torque, Bw, Kw):
        self.settings['mp_damping'] = Bw
        self.settings['mp_spring_stiffness'] = Kw
        self.settings['mp_torque'] = torque
        sensor_data = self.write_and_read(msgtype="201", data=self.settings)
        # (Note that we get these sensor values for free each time we send data)
        return sensor_data

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
        # OBJECT IN OUR DESIRED PROCESS

        self.pcan_object = PCANBasic()
        self.pcan_initialization_result = self.pcan_object.Initialize(self._pcan_channel, PCAN_BAUD_1M)

        # Start the controller
        self.write_and_read(msgtype="2001F", data=None)
        self.write_and_read(msgtype="20012", data=None)
        self.write_and_read(msgtype="20014", data=None)

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

        received = {0: 1}
        while received[0] > 0:
            received = self.pcan_object.Read(self._pcan_channel)

        output = self.map_sensodrive_to_si(received)

        return output

    @staticmethod
    def map_si_to_sensodrive(settings):
        """
        Converts settings to sensodrive message
        """
        if settings != None:
            torque = int(settings['mp_torque'] * 1000.0)
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
        # print("gives: ",data[0], data[1])
        data[2] = friction_bytes[0]
        data[3] = friction_bytes[1]
        data[4] = damping_bytes[0]
        data[5] = damping_bytes[1]
        data[6] = spring_stiffness_bytes[0]
        data[7] = spring_stiffness_bytes[1]

        return data

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
