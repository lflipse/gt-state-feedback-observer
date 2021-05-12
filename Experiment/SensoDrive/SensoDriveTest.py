# import pygame
import time
import numpy as np
import matplotlib.pyplot as plt
import multiprocessing as mp
from PCANBasic import *
import math

class SensoDriveTest:
    def __init__(self):
        # Initialize message structures
        self.steering_wheel_message = TPCANMsg()
        self.state_message = TPCANMsg()
        self.pedal_message = TPCANMsg()
        self.sensodrive_message = TPCANMsg()
        self.sensodrive_initialization_message = TPCANMsg()
        self.state_change_message = TPCANMsg()
        self._pcan_channel = None
        self._time_step_in_ns = None
        self._time = None
        self.settings_dict = None
        self.pcan_object = None
        self.pcan_initialization_result = None
        self.values_from_sensodrive = {}

        self.settings= dict()
        self.settings['mp_torque'] = 0
        self.settings['mp_friction'] = 0
        self.settings['mp_damping'] = 0.1
        self.settings['mp_spring_stiffness'] = 0.5

        self.state_change_message.ID = INITIALIZATION_MESSAGE_ID
        self.state_change_message.LEN = INITIALIZATION_MESSAGE_LENGTH
        self.state_change_message.TYPE = PCAN_MESSAGE_STANDARD
        # mode of operation
        self.state_change_message.DATA[0] = 0x11
        self.state_change_message.DATA[1] = 0x00
        # End stop position
        self.state_change_message.DATA[2] = 0xB4
        self.state_change_message.DATA[3] = 0x00
        # Torque beyond end stops:
        self.state_change_message.DATA[6] = 0x14
        self.state_change_message.DATA[7] = 0x14

        self._current_state_hex = 0x00

    def do(self):
        self._pcan_channel = PCAN_USBBUS1
        self._time_step_in_ns = 10000000
        # Here we can initialize our PCAN Communication (WE HAVE TO DO THIS HERE ELSE WE WONT HAVE THE PCAN
        # OBJECT IN OUR DESIRED PROCESS
        self.initialize()

        for i in range(10):
            t0 = time.perf_counter_ns()

            # Let's first start by reading out some values
            data = self._map_si_to_sensodrive(self.settings)
            torque_bytes = int.to_bytes(data['torque'], 2, byteorder='little', signed=True)
            friction_bytes = int.to_bytes(data['friction'], 2, byteorder='little', signed=True)
            damping_bytes = int.to_bytes(data['damping'], 2, byteorder='little', signed=True)
            spring_stiffness_bytes = int.to_bytes(data['spring_stiffness'], 2, byteorder='little', signed=True)

            self.sensodrive_message.ID = STEERINGWHEEL_MESSAGE_SEND_ID
            self.sensodrive_message.LEN = STEERINGWHEEL_MESSAGE_LENGTH
            self.sensodrive_message.TYPE = PCAN_MESSAGE_STANDARD
            self.sensodrive_message.DATA[0] = torque_bytes[0]
            self.sensodrive_message.DATA[1] = torque_bytes[1]
            self.sensodrive_message.DATA[2] = friction_bytes[0]
            self.sensodrive_message.DATA[3] = friction_bytes[1]
            self.sensodrive_message.DATA[4] = damping_bytes[0]
            self.sensodrive_message.DATA[5] = damping_bytes[1]
            self.sensodrive_message.DATA[6] = spring_stiffness_bytes[0]
            self.sensodrive_message.DATA[7] = spring_stiffness_bytes[1]

            self.pcan_object.Write(self._pcan_channel, self.sensodrive_message)

            # receive data from Sensodrive (wheel, pedals)
            received = self.pcan_object.Read(self._pcan_channel)
            received_msg = self._sensodrive_data_to_si(received)
            print(self.values_from_sensodrive)


            execution_time = time.perf_counter_ns() - t0
            time.sleep(max(0.0, (self._time_step_in_ns - execution_time) * 1e-9))

    def initialize(self):
        """
        Initializes the PCAN Dongle and sends appropriate initialization messages to the SensoDrive.
        :return:
        """
        self.pcan_object = PCANBasic()
        self.pcan_initialization_result = self.pcan_object.Initialize(self._pcan_channel, PCAN_BAUD_1M)

        # Convert our shared settings to bytes
        endstops_bytes = int.to_bytes(int(math.degrees(180)), 2, byteorder='little',
                                      signed=True)
        torque_limit_between_endstops_bytes = int.to_bytes(100, 1,
                                                           byteorder='little',
                                                           signed=False)
        torque_limit_beyond_endstops_bytes = int.to_bytes(100, 1,
                                                          byteorder='little',
                                                          signed=False)

        # We need to have our init message here as well
        self.sensodrive_initialization_message.ID = INITIALIZATION_MESSAGE_ID
        self.sensodrive_initialization_message.LEN = INITIALIZATION_MESSAGE_LENGTH
        self.sensodrive_initialization_message.TYPE = PCAN_MESSAGE_STANDARD
        # mode of operation
        self.sensodrive_initialization_message.DATA[0] = 0x11
        # reserved
        self.sensodrive_initialization_message.DATA[1] = 0
        # Endstop position
        self.sensodrive_initialization_message.DATA[2] = endstops_bytes[0]
        self.sensodrive_initialization_message.DATA[3] = endstops_bytes[1]
        # reserved
        self.sensodrive_initialization_message.DATA[4] = 0
        self.sensodrive_initialization_message.DATA[5] = 0
        # Torque between endstops:
        self.sensodrive_initialization_message.DATA[6] = torque_limit_between_endstops_bytes[0]
        # Torque beyond endstops:
        self.sensodrive_initialization_message.DATA[7] = torque_limit_beyond_endstops_bytes[0]

        self.pcan_object.Write(self._pcan_channel, self.sensodrive_initialization_message)
        # time.sleep(0.002)
        msg = self.pcan_object.Read(self._pcan_channel)

        # do not switch mode
        self.state_message = self.sensodrive_initialization_message
        self.state_message.DATA[0] = 0x11

        # Set the data structure for the steeringwheel message with the just applied values
        self.steering_wheel_parameters = self._map_si_to_sensodrive(self.settings)

        self.pcan_object.Write(self._pcan_channel, self.sensodrive_initialization_message)
        # time.sleep(0.02)
        response = self.pcan_object.Read(self._pcan_channel)

        self._current_state_hex = response[1].DATA[0]

        self.state_message = self.sensodrive_initialization_message
        self.state_message.DATA[0] = 0x11

    def write_message_steering_wheel(self, pcan_object, pcan_message, data):
        """
        Writes a CAN message to the sensodrive containing information regarding torque, friction and damping. Also
        returns the current state of the wheel (angle, force etc).
        :param pcan_object:
        :param pcan_message:
        :param data:
        :return:
        """
        torque_bytes = int.to_bytes(data['torque'], 2, byteorder='little', signed=True)
        friction_bytes = int.to_bytes(data['friction'], 2, byteorder='little', signed=True)
        damping_bytes = int.to_bytes(data['damping'], 2, byteorder='little', signed=True)
        spring_stiffness_bytes = int.to_bytes(data['spring_stiffness'], 2, byteorder='little', signed=True)

        pcan_message.ID = STEERINGWHEEL_MESSAGE_SEND_ID
        pcan_message.LEN = STEERINGWHEEL_MESSAGE_LENGTH
        pcan_message.TYPE = PCAN_MESSAGE_STANDARD
        pcan_message.DATA[0] = torque_bytes[0]
        pcan_message.DATA[1] = torque_bytes[1]
        pcan_message.DATA[2] = friction_bytes[0]
        pcan_message.DATA[3] = friction_bytes[1]
        pcan_message.DATA[4] = damping_bytes[0]
        pcan_message.DATA[5] = damping_bytes[1]
        pcan_message.DATA[6] = spring_stiffness_bytes[0]
        pcan_message.DATA[7] = spring_stiffness_bytes[1]

        pcan_object.Write(self._pcan_channel, pcan_message)

    @staticmethod
    def _map_si_to_sensodrive(settings):
        """
        Converts settings to sensodrive units
        :param settings:
        :return: converted settings
        """

        out = {
            'torque': int(settings['mp_torque'] * 1000.0),
            'friction': int(settings['mp_friction'] * 1000.0),
            'damping': int(settings['mp_damping'] * 1000.0 * (2.0 * math.pi) / 60.0),
            'spring_stiffness': int(settings['mp_spring_stiffness'] * 1000.0 / (180.0 / math.pi))
        }

        return out

    def _sensodrive_data_to_si(self, received):
        """
        Converts sensodrive data to actual SI Units
        :param received: sensodrive unit filled dictionary
        :return:
        """
        print(received[1].ID)
        if received[1].ID == STEERINGWHEEL_MESSAGE_RECEIVE_ID:
            # steering wheel

            # steering angle
            increments = int.from_bytes(received[1].DATA[0:4], byteorder='little', signed=True)
            self.values_from_sensodrive['steering_angle'] = math.radians(
                float(increments) * 0.009)  # we get increments, convert to deg, convert to rad

            # steering rate
            steering_rate = int.from_bytes(received[1].DATA[4:6], byteorder='little', signed=True)
            self.values_from_sensodrive['steering_rate'] = float(steering_rate) * (
                    2.0 * math.pi) / 60.0  # we get rev/min, convert to rad/s

            # torque
            torque = int.from_bytes(received[1].DATA[6:], byteorder='little', signed=True)
            self.values_from_sensodrive['measured_torque'] = float(torque) / 1000.0  # we get mNm convert to Nm

        elif received[1].ID == PEDAL_MESSAGE_RECEIVE_ID:
            # pedals
            self.values_from_sensodrive['throttle'] = float(
                int.from_bytes(received[1].DATA[2:4], byteorder='little') - 1100) / 2460.0
            self.values_from_sensodrive['brake'] = float(
                int.from_bytes(received[1].DATA[4:6], byteorder='little') - 1) / 500

        elif received[1].ID == STATE_MESSAGE_RECEIVE_ID:
            #
            self._current_state_hex = received[1].DATA[0]
            self.values_from_sensodrive['sensodrive_motorstate'] = received[1].DATA[0]


"""
These global parameters are used to make the message ID's more identifiable than just the hex nr.
"""

INITIALIZATION_MESSAGE_ID = 0x200
INITIALIZATION_MESSAGE_LENGTH = 8
STATE_MESSAGE_RECEIVE_ID = 0x210

STEERINGWHEEL_MESSAGE_SEND_ID = 0x201
STEERINGWHEEL_MESSAGE_RECEIVE_ID = 0x211
STEERINGWHEEL_MESSAGE_LENGTH = 8

PEDAL_MESSAGE_SEND_ID = 0x20C
PEDAL_MESSAGE_RECEIVE_ID = 0x21C
PEDAL_MESSAGE_LENGTH = 2

tester_class = SensoDriveTest()
tester_class.do()