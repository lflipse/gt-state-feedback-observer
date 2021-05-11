import pygame
import time
import numpy as np
import multiprocessing as mp
from Experiment.SensoDrive.PCANBasic import *
import math

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

class Experiment:
    def __init__(self, screen_width, screen_height, robot, human):
        ## PCAN initializatie tjak
        self.steering_wheel_message = TPCANMsg()
        self.state_message = TPCANMsg()
        self.pedal_message = TPCANMsg()
        self.sensodrive_initialization_message = TPCANMsg()
        self.state_change_message = TPCANMsg()
        self._pcan_channel = PCAN_USBBUS1
        self._steering_data = dict()
        self._steering_data['torque']  = 0
        self._steering_data['friction'] = 0
        self._steering_data['damping'] = 0
        self._steering_data['spring_stiffness'] = 0
        self.current_values = dict()
        self._time_step_in_ns = None
        self._time = None
        self.settings_dict = None
        self.pcan_object = None
        self.pcan_initialization_result = None

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

        self.screen_width = screen_width
        self.screen_height = screen_height
        self.robot = robot
        self.human = human

        # Initialize the entire thing
        # Initialize the game
        pygame.init()
        self.initialize()

        # I guess depends on screen dimensions, full hd for now
        # screen = pygame.display.set_mode((1920, 1080))
        self.screen = pygame.display.set_mode((screen_width, screen_height))

        # Title and Icon
        pygame.display.set_caption("Experiment")
        icon = pygame.image.load("images/ufo.png")
        pygame.display.set_icon(icon)

        playerImg = pygame.image.load("images/player.png")
        self.playerImg = pygame.transform.scale(playerImg, (50, 50))
        self.playerX = 370
        self.playerY = 480
        self.bg_color = (255, 255, 255)
        self.border_color = (0, 0, 0)
        self.rect_width = 80
        self.rect_height = 50
        self.rect_surf = self.create_rect(self.rect_width, self.rect_height, 4, self.bg_color, self.border_color)

    def create_rect(self, width, height, border, color, border_color):
        surf = pygame.Surface((width + border * 2, height + border * 2), pygame.SRCALPHA)
        pygame.draw.rect(surf, color, (border, border, width, height), 0)
        for i in range(1, border):
            pygame.draw.rect(surf, border_color, (border - i, border - i, width + 5, height + 5), 1)
        return surf

    def translate_to_position(self, r):
        # Translate from angle between -30 to 30 degrees to
        angle_deg = r * 180 * np.pi / 30
        x_r = angle_deg/60 * self.screen_width + 0.5 * self.screen_width
        return x_r

    def player(self, x, y):
        self.screen.blit(self.playerImg, (x, y))

    def rectangle(self, rect_surf, r, y):
        self.screen.blit(rect_surf, (r, y))

    def do_experiment(self, r, N, h):
        # Game loop
        running = True

        y = np.zeros((N + 1, 4))

        # Estimator vectors
        Qhhat = np.zeros((N + 1, 2, 2))
        Phhat = np.zeros((N + 1, 2, 2))
        Ph = np.zeros((N, 2, 2))
        Pr = np.zeros((N, 2, 2))
        Qh = np.zeros((N, 2, 2))
        Qr = np.zeros((N, 2, 2))
        Lhhat = np.zeros((N, 2))
        uhhat = np.zeros(N)

        # Real vectors
        Lh = np.zeros((N, 2))
        Lr = np.zeros((N, 2))
        ref = np.zeros((N, 2))
        e = np.zeros((N, 2))
        x = np.zeros((N, 2))
        xdot = np.zeros((N, 2))
        xhhat = np.zeros((N, 2))
        ur = np.zeros(N)
        uhbar = np.zeros(N)
        vh = np.zeros(N)
        uh = np.zeros(N)


        for i in range(N):
            t1 = time.time()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    exit("Quited the game")

            # Measure stuff
            # Calcuate derivative(s) of reference
            if i > 0:
                ref[i, :] = np.array([r[i], (r[i] - r[i - 1]) / h])
                xdot[i, :] = (x[i, :] - x[i - 1, :]) / h
            else:
                ref[i, :] = np.array([r[i], (r[i]) / (2 * h)])
                xdot[i, :] = (x[i, :]) / (2 * h)

            e[i, :] = x[i, :] - ref[i, :]  # Assuming x is measured

            if i > 0:
                # Calculated control inputs
                # uh[i], Lh = self.human.compute_controls()
                # Pr[i, :], Phhat[i, :], Qhhat[i, :, :] = self.robot.update_gain()
                a = 1

            # Integrate dynamics



            ## PCAN communication loop
            self.write_message_steering_wheel(self.pcan_object, self.steering_wheel_message,
                                              self._steering_data)

            # receive data from Sensodrive (wheel, pedals)
            received = self.pcan_object.Read(self._pcan_channel)

            # request state data
            endstops_bytes = int.to_bytes(int(math.degrees(180)), 2, byteorder='little',
                                          signed=True)
            self.state_message.DATA[2] = endstops_bytes[0]
            self.state_message.DATA[3] = endstops_bytes[1]

            self.pcan_object.Write(self._pcan_channel, self.state_message)
            received2 = self.pcan_object.Read(self._pcan_channel)

            if received[0] or received2[0] == PCAN_ERROR_OK:
                mydata1 = self._sensodrive_data_to_si(received)

                mydata2 =self._sensodrive_data_to_si(received2)
                playerX = self.translate_to_position(mydata1['steering_angle'])
            else:
                playerX = 0

            self.screen.fill(self.bg_color)
            x_r = self.translate_to_position(r[i])
            self.rectangle(self.rect_surf, x_r, self.playerY)

            self.player(playerX, self.playerY)
            pygame.display.update()

            d = time.time() - t1
            if d < h:
                time.sleep(h - d)
            else:
                print('having a delay')

                # self._sensodrive_data_to_si(received3)


        print("Finished the game")


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

    def write_message_pedals(self, pcan_object, pcan_message):
        """
        Writes a correctly structured CAN message to the sensodrive which will return a message containing the
        inputs of the pedals.
        :param pcan_object:
        :param pcan_message:
        :return:
        """
        pcan_message.ID = 0x20C
        pcan_message.LEN = 1
        pcan_message.MSG_TYPE = PCAN_MESSAGE_STANDARD

        pcan_message.DATA[0] = 0x1

        pcan_object.Write(self._pcan_channel, pcan_message)

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
        self.pcan_object.Read(self._pcan_channel)

        # do not switch mode
        self.state_message = self.sensodrive_initialization_message
        self.state_message.DATA[0] = 0x11

        # Set the data structure for the steeringwheel message with the just applied values
        # self.steering_wheel_parameters = self._map_si_to_sensodrive(self.settings_dict)

        self.pcan_object.Write(self._pcan_channel, self.sensodrive_initialization_message)
        time.sleep(0.02)
        response = self.pcan_object.Read(self._pcan_channel)

        self._current_state_hex = response[1].DATA[0]

        self.state_message = self.sensodrive_initialization_message
        self.state_message.DATA[0] = 0x11

    def _sensodrive_data_to_si(self, received):
        """
        Converts sensodrive data to actual SI Units
        :param received: sensodrive unit filled dictionary
        :return:
        """

        if received[1].ID == STEERINGWHEEL_MESSAGE_RECEIVE_ID:
            # steering wheel

            # steering angle
            increments = int.from_bytes(received[1].DATA[0:4], byteorder='little', signed=True)
            self.current_values['steering_angle'] = math.radians(
                float(increments) * 0.009)  # we get increments, convert to deg, convert to rad

            # steering rate
            steering_rate = int.from_bytes(received[1].DATA[4:6], byteorder='little', signed=True)
            self.current_values['steering_rate'] = float(steering_rate) * (
                    2.0 * math.pi) / 60.0  # we get rev/min, convert to rad/s

            # torque
            torque = int.from_bytes(received[1].DATA[6:], byteorder='little', signed=True)
            self.current_values['measured_torque'] = float(torque) / 1000.0  # we get mNm convert to Nm

        elif received[1].ID == PEDAL_MESSAGE_RECEIVE_ID:
            # pedals
            self.current_values['throttle'] = float(
                int.from_bytes(received[1].DATA[2:4], byteorder='little') - 1100) / 2460.0
            self.current_values['brake'] = float(
                int.from_bytes(received[1].DATA[4:6], byteorder='little') - 1) / 500

        elif received[1].ID == STATE_MESSAGE_RECEIVE_ID:
            #
            self._current_state_hex = received[1].DATA[0]
            self.current_values['sensodrive_motorstate'] = received[1].DATA[0]

        return self.current_values