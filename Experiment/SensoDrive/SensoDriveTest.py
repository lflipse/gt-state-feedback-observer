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
        self._pcan_channel = None

        self.settings= dict()
        self.settings['mp_torque'] = 0
        self.settings['mp_friction'] = 0
        self.settings['mp_damping'] = 0.6
        self.settings['mp_spring_stiffness'] = 2


    def initialize(self):
        """
        Initializes the PCAN Dongle and sends appropriate initialization messages to the SensoDrive.
        :return:
        """
        self._pcan_channel = PCAN_USBBUS1
        self._time_step_in_ns = 10000000
        # Here we can initialize our PCAN Communication (WE HAVE TO DO THIS HERE ELSE WE WONT HAVE THE PCAN
        # OBJECT IN OUR DESIRED PROCESS

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

        sensodrive_comm_message = TPCANMsg()
        sensodrive_comm_message.ID = 0x200
        sensodrive_comm_message.LEN = 8
        sensodrive_comm_message.TYPE = PCAN_MESSAGE_STANDARD
        # mode of operation
        sensodrive_comm_message.DATA[0] = 0x10
        # reserved
        sensodrive_comm_message.DATA[1] = 0
        # Endstop position
        sensodrive_comm_message.DATA[2] = 0
        sensodrive_comm_message.DATA[3] = 0
        # reserved
        sensodrive_comm_message.DATA[4] = 0
        sensodrive_comm_message.DATA[5] = 0
        # Torque between endstops:
        sensodrive_comm_message.DATA[6] = 0
        # Torque beyond endstops:
        sensodrive_comm_message.DATA[7] = 0

        # Write and read
        self.pcan_object.Write(self._pcan_channel, sensodrive_comm_message)
        time.sleep(0.005)
        msg = self.pcan_object.Read(self._pcan_channel)

        if msg[1].ID == STATE_MESSAGE_RECEIVE_ID:
            print("Communication Okay!")
        else:
            print("Nothing received")

        # We need to have our init message here as well
        sensodrive_initialization_message = TPCANMsg()

        sensodrive_initialization_message.ID = 0x201
        sensodrive_initialization_message.LEN = 8
        sensodrive_initialization_message.TYPE = PCAN_MESSAGE_STANDARD
        # mode of operation
        sensodrive_initialization_message.DATA[0] = 0
        # reserved
        sensodrive_initialization_message.DATA[1] = 0
        # Endstop position
        sensodrive_initialization_message.DATA[2] = 0
        sensodrive_initialization_message.DATA[3] = 0
        # reserved
        sensodrive_initialization_message.DATA[4] = 0
        sensodrive_initialization_message.DATA[5] = 0
        # Torque between endstops:
        sensodrive_initialization_message.DATA[6] = 0
        # Torque beyond endstops:
        sensodrive_initialization_message.DATA[7] = 0

        self.pcan_object.Write(self._pcan_channel, sensodrive_initialization_message)
        time.sleep(0.002)
        msg = self.pcan_object.Read(self._pcan_channel)
        msg_si = self.map_sensodrive_to_si(msg)
        print("Checking initial states")
        print(msg_si)

        sensodrive_init_message = TPCANMsg()
        sensodrive_init_message.ID = 0x200
        sensodrive_init_message.LEN = 8
        sensodrive_init_message.TYPE = PCAN_MESSAGE_STANDARD
        # mode of operation
        sensodrive_init_message.DATA[0] = 0x1F
        # reserved
        sensodrive_init_message.DATA[1] = 0
        # Endstop position
        sensodrive_init_message.DATA[2] = 0
        sensodrive_init_message.DATA[3] = 0xB4
        # reserved
        sensodrive_init_message.DATA[4] = 0
        sensodrive_init_message.DATA[5] = 0
        # Torque between endstops:
        sensodrive_init_message.DATA[6] = 0x14
        # Torque beyond endstops:
        sensodrive_init_message.DATA[7] = 0x14

        self.pcan_object.Write(self._pcan_channel, sensodrive_init_message)
        time.sleep(0.005)
        msg = self.pcan_object.Read(self._pcan_channel)
        print("Checking initialization: ", msg[1].ID)

        sensodrive_ready_message = sensodrive_init_message
        sensodrive_ready_message.DATA[0] = 0x12
        self.pcan_object.Write(self._pcan_channel, sensodrive_ready_message)
        time.sleep(0.005)
        msg = self.pcan_object.Read(self._pcan_channel)
        print("Checking ready state?", msg[1].ID)
        print(msg[1].ID)

        sensodrive_on_message = sensodrive_init_message
        sensodrive_ready_message.DATA[0] = 0x14
        self.pcan_object.Write(self._pcan_channel, sensodrive_ready_message)
        time.sleep(0.005)
        msg = self.pcan_object.Read(self._pcan_channel)
        print("Checking on state (528): ", msg[1].ID)


    def do(self):
        self.initialize()
        sensodrive_message = TPCANMsg()

        for i in range(1000):
            t0 = time.perf_counter_ns()

            # Let's first start by reading out some values
            data = self.map_si_to_sensodrive(self.settings)
            torque_bytes = int.to_bytes(data['torque'], 2, byteorder='little', signed=True)
            friction_bytes = int.to_bytes(data['friction'], 2, byteorder='little', signed=True)
            damping_bytes = int.to_bytes(data['damping'], 2, byteorder='little', signed=True)
            spring_stiffness_bytes = int.to_bytes(data['spring_stiffness'], 2, byteorder='little', signed=True)

            sensodrive_message.ID = 0x201
            sensodrive_message.LEN = 8
            sensodrive_message.TYPE = PCAN_MESSAGE_STANDARD
            sensodrive_message.DATA[0] = torque_bytes[0]
            sensodrive_message.DATA[1] = torque_bytes[1]
            sensodrive_message.DATA[2] = friction_bytes[0]
            sensodrive_message.DATA[3] = friction_bytes[1]
            sensodrive_message.DATA[4] = damping_bytes[0]
            sensodrive_message.DATA[5] = damping_bytes[1]
            sensodrive_message.DATA[6] = spring_stiffness_bytes[0]
            sensodrive_message.DATA[7] = spring_stiffness_bytes[1]

            self.pcan_object.Write(self._pcan_channel, sensodrive_message)
            time.sleep(0.005)

            # receive data from Sensodrive (wheel, pedals)
            received = self.pcan_object.Read(self._pcan_channel)
            msg = self.map_sensodrive_to_si(received)  # Values output to self.values_from_sensodrive
            # print(msg)

            execution_time = time.perf_counter_ns() - t0
            time.sleep(max(0.0, (self._time_step_in_ns - execution_time) * 1e-9))



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
    def map_si_to_sensodrive(settings):
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

    def map_sensodrive_to_si(self, received):
        """
        Converts sensodrive data to actual SI Units
        :param received: sensodrive unit filled dictionary
        :return:
        """
        print(received[1].ID)
        message = {}
        if received[1].ID == STEERINGWHEEL_MESSAGE_RECEIVE_ID:
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

        elif received[1].ID == PEDAL_MESSAGE_RECEIVE_ID:
            # pedals
            message['throttle'] = float(
                int.from_bytes(received[1].DATA[2:4], byteorder='little') - 1100) / 2460.0
            message['brake'] = float(
                int.from_bytes(received[1].DATA[4:6], byteorder='little') - 1) / 500

        elif received[1].ID == STATE_MESSAGE_RECEIVE_ID:
            self._current_state_hex = received[1].DATA[0]
            message['sensodrive_motorstate'] = received[1].DATA[0]

        return message

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